// SPDX-License-Identifier: GPL-2.0
/*
 *  phy-rtk-rle0599.c RTK rle0599 usb2.0 phy driver
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/usb.h>
#include <linux/phy/phy.h>
#include <soc/realtek/rtk_chip.h>

#define RTK_USB_RLE0599_PHY_NAME "rtk-usb-phy-rle0599"

struct reg_addr {
	void __iomem *reg_wrap_vstatus_out2;
	void __iomem *reg_ehci_insnreg05;
};

struct phy_data_addr {
	u8 addr;
	u8 data;
};

struct rtk_phy_data {
	int page0_size;
	struct phy_data_addr *page0;
	int page1_size;
	struct phy_data_addr *page1;

	bool check_efuse;
	u8 efuse_usb_dp_dm;
	bool do_toggle;
	bool use_default_parameter;
};

struct rtk_phy {
	struct device *dev;

	enum rtd_chip_id chip_id;
	enum rtd_chip_revision chip_revision;

	int num_phy;
	struct reg_addr *reg_addr;
	struct rtk_phy_data *phy_data;
	bool initialized;

	struct dentry *debug_dir;
};

/*
 * PHY tuning values are a property of the SoC, not of the board, so they
 * belong here rather than in every device tree that instantiates the PHY.
 */
struct rtk_phy_cfg {
	int num_phy;
	int page0_size;
	const struct phy_data_addr *page0;
	int page1_size;
	const struct phy_data_addr *page1;
	bool check_efuse;
	bool do_toggle;
	bool use_default_parameter;
};

static const struct phy_data_addr rtd1195_page0[] = {
	{0xe0, 0xa0}, {0xe1, 0x30}, {0xe2, 0x9a}, {0xe3, 0x8d},
	{0xe4, 0xd6}, {0xe5, 0x1d}, {0xe6, 0xc0}, {0xe7, 0xb1},
	{0xf0, 0xfc}, {0xf1, 0x9c}, {0xf2, 0x00}, {0xf3, 0x11},
	{0xf4, 0x9b}, {0xf5, 0x81}, {0xf6, 0x00}, {0xf7, 0x02},
};

static const struct phy_data_addr rtd1195_page1[] = {
	{0xe0, 0x35}, {0xe1, 0xaf}, {0xe2, 0x60}, {0xe3, 0x00},
	{0xe4, 0x00}, {0xe5, 0x0f}, {0xe6, 0x18}, {0xe7, 0xe3},
};

static const struct rtk_phy_cfg rtd1195_phy_cfg = {
	.num_phy = 1,
	.page0 = rtd1195_page0,
	.page0_size = ARRAY_SIZE(rtd1195_page0),
	.page1 = rtd1195_page1,
	.page1_size = ARRAY_SIZE(rtd1195_page1),
};

static u8 efuse_usb_dp_dm_table[0x10] = {0xe0, 0x80, 0x84, 0x88, 0x8c, 0x90, 0x94, 0x98,
					 0x9c, 0xa0, 0xa4, 0xa8, 0xac, 0xb0, 0xb4, 0xb8};

#define OFFEST_PHY_READ 0x20

#define USB_ST_BUSY		BIT(17)

static DEFINE_SPINLOCK(rtk_phy_lock);

#define phy_reg_read(addr)		__raw_readl(addr)
#define phy_reg_write(addr, val)	do { smp_wmb(); __raw_writel(val, addr); } while (0)
#define PHY_IO_TIMEOUT_MSEC		(50)

static inline int utmi_wait_register(void __iomem *reg, u32 mask, u32 result)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(PHY_IO_TIMEOUT_MSEC);

	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if ((phy_reg_read(reg) & mask) == result)
			return 0;
		udelay(100);
	}
	pr_err("%s: can't program USB phy\n", __func__);
	return -ETIMEDOUT;
}

static char rtk_phy_read(struct rtk_phy *rtk_phy, char addr)
{
	unsigned int val;
	struct reg_addr *phy_reg = rtk_phy->reg_addr;
	void __iomem *reg_ehci_insnreg05 = phy_reg->reg_ehci_insnreg05;

	utmi_wait_register(reg_ehci_insnreg05, USB_ST_BUSY, 0);

	/* VCtrl = low nibble of addr, VLoadM = 1 */
	val = (1 << 13) |	/* Port num */
		 (1 << 12) |	/* vload */
		 ((addr & 0x0f) << 8);	/* vcontrol */
	phy_reg_write(reg_ehci_insnreg05, val);
	utmi_wait_register(reg_ehci_insnreg05, USB_ST_BUSY, 0);

	/* VCtrl = low nibble of addr, VLoadM = 0 */
	val &= ~(1 << 12);
	phy_reg_write(reg_ehci_insnreg05, val);
	utmi_wait_register(reg_ehci_insnreg05, USB_ST_BUSY, 0);

	/* VCtrl = high nibble of addr, VLoadM = 1 */
	val = (1 << 13) |	/* Port num */
		 (1 << 12) |	/* vload */
		 ((addr & 0xf0) << 4);	/* vcontrol */
	phy_reg_write(reg_ehci_insnreg05, val);
	utmi_wait_register(reg_ehci_insnreg05, USB_ST_BUSY, 0);

	/* VCtrl = high nibble of addr, VLoadM = 0 */
	val &= ~(1 << 12);
	phy_reg_write(reg_ehci_insnreg05, val);
	utmi_wait_register(reg_ehci_insnreg05, USB_ST_BUSY, 0);

	smp_rmb();
	val = phy_reg_read(reg_ehci_insnreg05);

	return (char)(val & 0xff);
}

static int rtk_phy_write(struct rtk_phy *rtk_phy, char addr, char data)
{
	unsigned int val;
	struct reg_addr *phy_reg = rtk_phy->reg_addr;
	void __iomem *reg_wrap_vstatus_out2 = phy_reg->reg_wrap_vstatus_out2;
	void __iomem *reg_ehci_insnreg05 = phy_reg->reg_ehci_insnreg05;

	/* write data to VStatusOut2 (data output to phy) */
	phy_reg_write(reg_wrap_vstatus_out2, (u32)data);

	utmi_wait_register(reg_ehci_insnreg05, USB_ST_BUSY, 0);

	/* VCtrl = low nibble of addr, VLoadM = 1 */
	val = (1 << 13) |	/* Port num */
		 (1 << 12) |	/* vload */
		 ((addr & 0x0f) << 8);	/* vcontrol */
	phy_reg_write(reg_ehci_insnreg05, val);
	utmi_wait_register(reg_ehci_insnreg05, USB_ST_BUSY, 0);

	/* VCtrl = low nibble of addr, VLoadM = 0 */
	val &= ~(1 << 12);
	phy_reg_write(reg_ehci_insnreg05, val);
	utmi_wait_register(reg_ehci_insnreg05, USB_ST_BUSY, 0);

	/* VCtrl = high nibble of addr, VLoadM = 1 */
	val = (1 << 13) |	/* Port num */
		 (1 << 12) |	/* vload */
		 ((addr & 0xf0) << 4);	/* vcontrol */
	phy_reg_write(reg_ehci_insnreg05, val);
	utmi_wait_register(reg_ehci_insnreg05, USB_ST_BUSY, 0);

	/* VCtrl = high nibble of addr, VLoadM = 0 */
	val &= ~(1 << 12);
	phy_reg_write(reg_ehci_insnreg05, val);
	utmi_wait_register(reg_ehci_insnreg05, USB_ST_BUSY, 0);

	return 0;
}

static int rtk_phy_set_page(struct rtk_phy *rtk_phy, int page)
{
	return rtk_phy_write(rtk_phy, 0xf4, page == 0 ? 0x9b : 0xbb);
}

static int updated_phy_parameter_by_efuse(struct rtk_phy *rtk_phy)
{
	u8 value = 0;
	int size = 4;
	int mask = (BIT(size) - 1);
	struct rtk_phy_data *phy_data = rtk_phy->phy_data;
	struct phy_data_addr *phy_page0_default_setting = phy_data->page0;
	int i;

	/*
	 * The DP/DM trim lives at efuse offset 0x1dc. Reading it needs an
	 * nvmem cell that this device tree does not describe yet, so fall
	 * back to entry 0 of the table, which is the untrimmed default.
	 */
	dev_dbg(rtk_phy->dev, "no efuse trim available, using default %x\n", value);
	phy_data->efuse_usb_dp_dm = value;

	for (i = 0; i < phy_data->page0_size; i++) {
		if ((phy_page0_default_setting + i)->addr == 0xe0) {
			(phy_page0_default_setting + i)->data = efuse_usb_dp_dm_table[value & mask];
			dev_dbg(rtk_phy->dev, "Set addr %x value %x\n",
				(phy_page0_default_setting + i)->addr,
				(phy_page0_default_setting + i)->data);
		}
	}
	return 0;
}

static void do_rtk_phy_toggle(struct rtk_phy *rtk_phy, bool connect)
{
	struct rtk_phy_data *phy_data = rtk_phy->phy_data;
	struct phy_data_addr *phy_page0_default_setting;
	struct phy_data_addr *phy_page1_default_setting;
	int i;

	if (!phy_data->do_toggle)
		return;

	phy_page0_default_setting = phy_data->page0;
	phy_page1_default_setting = phy_data->page1;

	/* Set page 0 */
	rtk_phy_set_page(rtk_phy, 0);
	for (i = 0; i < phy_data->page0_size; i++) {
		if ((phy_page0_default_setting + i)->addr == 0xE7) {
			if (connect) {
				rtk_phy_write(rtk_phy, (phy_page0_default_setting + i)->addr,
					      (phy_page0_default_setting + i)->data & (~(BIT(4) | BIT(5) | BIT(6))));
			} else {
				rtk_phy_write(rtk_phy, (phy_page0_default_setting + i)->addr,
					      (phy_page0_default_setting + i)->data | (BIT(4) | BIT(5) | BIT(6)));
			}
			dev_dbg(rtk_phy->dev, "%s %sconnect to set Page0 0xE7 = %x\n", __func__,
				connect ? "" : "dis",
				rtk_phy_read(rtk_phy, (phy_page0_default_setting + i)->addr));
		}
	}

	/* Set page 1 */
	rtk_phy_set_page(rtk_phy, 1);
	for (i = 0; i < phy_data->page1_size; i++) {
		if ((phy_page1_default_setting + i)->addr == 0xe0) {
			dev_info(rtk_phy->dev, "%s ########## to toggle Page1 addr 0xe0 BIT(2)\n", __func__);
			rtk_phy_write(rtk_phy, (phy_page1_default_setting + i)->addr,
				      ((phy_page1_default_setting + i)->data) & (~BIT(2)));
			mdelay(1);
			rtk_phy_write(rtk_phy, (phy_page1_default_setting + i)->addr,
				      (phy_page1_default_setting + i)->data);
		}
	}
}

static int rtk_phy_connect(struct phy *phy, int port)
{
	struct rtk_phy *rtk_phy = phy_get_drvdata(phy);

	dev_dbg(rtk_phy->dev, "%s port=%d\n", __func__, port);
	do_rtk_phy_toggle(rtk_phy, true);

	return 0;
}

static int rtk_phy_disconnect(struct phy *phy, int port)
{
	struct rtk_phy *rtk_phy = phy_get_drvdata(phy);

	dev_dbg(rtk_phy->dev, "%s port=%d\n", __func__, port);
	do_rtk_phy_toggle(rtk_phy, false);

	return 0;
}

static int rtk_phy_init(struct phy *phy)
{
	struct rtk_phy *rtk_phy = phy_get_drvdata(phy);
	struct rtk_phy_data *phy_data = rtk_phy->phy_data;
	struct phy_data_addr *phy_page0_default_setting = phy_data->page0;
	struct phy_data_addr *phy_page1_default_setting = phy_data->page1;
	unsigned long flags;
	int i;
	int ret = 0;

	spin_lock_irqsave(&rtk_phy_lock, flags);

	if (rtk_phy->initialized)
		goto out;

	if (phy_data->use_default_parameter) {
		dev_info(rtk_phy->dev, "%s phy use default parameter\n", __func__);
		goto do_toggle;
	}

	dev_info(rtk_phy->dev, "Init RTK USB phy-rle0599\n");

	if (phy_data->check_efuse)
		updated_phy_parameter_by_efuse(rtk_phy);

	/* Set page 0 */
	rtk_phy_set_page(rtk_phy, 0);
	for (i = 0; i < phy_data->page0_size; i++) {
		if (rtk_phy_write(rtk_phy, (phy_page0_default_setting + i)->addr,
				  (phy_page0_default_setting + i)->data)) {
			dev_err(rtk_phy->dev, "%s: page0 Error: addr=0x%x, value=0x%x\n",
				__func__,
				(phy_page0_default_setting + i)->addr,
				(phy_page0_default_setting + i)->data);
			ret = -EINVAL;
			goto out;
		}
	}

	/* Set page 1 */
	rtk_phy_set_page(rtk_phy, 1);
	for (i = 0; i < phy_data->page1_size; i++) {
		if (rtk_phy_write(rtk_phy, (phy_page1_default_setting + i)->addr,
				  (phy_page1_default_setting + i)->data)) {
			dev_err(rtk_phy->dev, "%s: page1 Error: addr=0x%x, value=0x%x\n",
				__func__,
				(phy_page1_default_setting + i)->addr,
				(phy_page1_default_setting + i)->data);
			ret = -EINVAL;
			goto out;
		}
	}

do_toggle:
	do_rtk_phy_toggle(rtk_phy, false);

	rtk_phy->initialized = true;

	dev_info(rtk_phy->dev, "Initialized RTK USB PHY rle0599\n");
out:
	spin_unlock_irqrestore(&rtk_phy_lock, flags);
	return ret;
}

static int rtk_phy_exit(struct phy *phy)
{
	struct rtk_phy *rtk_phy = phy_get_drvdata(phy);

	/* Todo */
	rtk_phy->initialized = false;

	return 0;
}

static const struct phy_ops ops = {
	.init		= rtk_phy_init,
	.exit		= rtk_phy_exit,
	.connect	= rtk_phy_connect,
	.disconnect	= rtk_phy_disconnect,
	.owner		= THIS_MODULE,
};

#ifdef CONFIG_DEBUG_FS
static struct dentry *create_phy_debug_root(void)
{
	struct dentry *phy_debug_root;

	phy_debug_root = debugfs_lookup("phy", usb_debug_root);
	if (!phy_debug_root)
		phy_debug_root = debugfs_create_dir("phy", usb_debug_root);

	return phy_debug_root;
}

static int rtk_rle0599_parameter_show(struct seq_file *s, void *unused)
{
	struct rtk_phy *rtk_phy = s->private;
	struct rtk_phy_data *phy_data = rtk_phy->phy_data;
	struct phy_data_addr *phy_page0_default_setting = phy_data->page0;
	struct phy_data_addr *phy_page1_default_setting = phy_data->page1;
	unsigned long flags;
	int i;

	seq_puts(s, "Page 0:\n");
	spin_lock_irqsave(&rtk_phy_lock, flags);
	/* Set page 0 */
	rtk_phy_set_page(rtk_phy, 0);

	for (i = 0; i < phy_data->page0_size; i++) {
		seq_printf(s, "Page 0: addr = 0x%x, value = 0x%x\n",
			   (phy_page0_default_setting + i)->addr,
			rtk_phy_read(rtk_phy, (phy_page0_default_setting + i)->addr - OFFEST_PHY_READ));
	}

	seq_puts(s, "Page 1:\n");
	/* Set page 1 */
	rtk_phy_set_page(rtk_phy, 1);

	for (i = 0; i < phy_data->page1_size; i++) {
		seq_printf(s, "Page 1: addr = 0x%x, value = 0x%x\n",
			   (phy_page1_default_setting + i)->addr,
			rtk_phy_read(rtk_phy, (phy_page1_default_setting + i)->addr - OFFEST_PHY_READ));
	}

	spin_unlock_irqrestore(&rtk_phy_lock, flags);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(rtk_rle0599_parameter);

static int rtk_rle0599_set_parameter_show(struct seq_file *s, void *unused)
{
	seq_puts(s, "Set Phy parameter by following command\n");
	seq_puts(s, "echo \"page addr value\" > set_parameter\n");
	seq_puts(s, "echo \"page0 0xE1 0x30\" > set_parameter\n");
	seq_puts(s, "echo \"page1 0xE1 0xEF\" > set_parameter\n");

	return 0;
}

static int rtk_rle0599_set_parameter_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtk_rle0599_set_parameter_show, inode->i_private);
}

static ssize_t rtk_rle0599_set_parameter_write(struct file *file,
					       const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct rtk_phy *rtk_phy = s->private;
	struct rtk_phy_data *phy_data = rtk_phy->phy_data;
	struct phy_data_addr *phy_page0_default_setting = phy_data->page0;
	struct phy_data_addr *phy_page1_default_setting = phy_data->page1;
	unsigned long flags;
	char buffer[40];
	char *buf = buffer;
	int i;
	u32 addr;
	u32 value;

	if (copy_from_user(&buffer, ubuf, min_t(size_t, sizeof(buffer) - 1, count)))
		return -EFAULT;

	spin_lock_irqsave(&rtk_phy_lock, flags);
	if (!strncmp(buf, "page0", 5)) {
		buf = buf + 5;
		buf = skip_spaces(buf);
		if (sscanf(buf, "%x %x", &addr, &value) != 2) {
			spin_unlock_irqrestore(&rtk_phy_lock, flags);
			return -EINVAL;
		}

		rtk_phy_set_page(rtk_phy, 0);
		for (i = 0; i < phy_data->page0_size; i++) {
			if ((phy_page0_default_setting + i)->addr == addr) {
				(phy_page0_default_setting + i)->data = value;
				if (rtk_phy_write(rtk_phy, (phy_page0_default_setting + i)->addr,
						  (phy_page0_default_setting + i)->data))
					dev_err(rtk_phy->dev, "%s: page0 Error: addr=0x%x, value=0x%x\n",
						__func__,
						(phy_page0_default_setting + i)->addr,
						(phy_page0_default_setting + i)->data);
			}
		}
	} else if (!strncmp(buf, "page1", 5)) {
		buf = buf + 5;
		buf = skip_spaces(buf);
		if (sscanf(buf, "%x %x", &addr, &value) != 2) {
			spin_unlock_irqrestore(&rtk_phy_lock, flags);
			return -EINVAL;
		}

		rtk_phy_set_page(rtk_phy, 1);
		for (i = 0; i < phy_data->page1_size; i++) {
			if ((phy_page1_default_setting + i)->addr == addr) {
				(phy_page1_default_setting + i)->data = value;
				if (rtk_phy_write(rtk_phy, (phy_page1_default_setting + i)->addr,
						  (phy_page1_default_setting + i)->data))
					dev_err(rtk_phy->dev, "%s: page1 Error: addr=0x%x, value=0x%x\n",
						__func__,
						(phy_page1_default_setting + i)->addr,
						(phy_page1_default_setting + i)->data);
			}
		}
	} else {
		dev_err(rtk_phy->dev, "UNKNOWN input (%s)", buf);
	}

	spin_unlock_irqrestore(&rtk_phy_lock, flags);
	return count;
}

static const struct file_operations rtk_rle0599_set_parameter_fops = {
	.open			= rtk_rle0599_set_parameter_open,
	.write			= rtk_rle0599_set_parameter_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static int rtk_rle0599_toggle_show(struct seq_file *s, void *unused)
{
	seq_puts(s, "echo 1 to toggle Page1 addr 0xe0 BIT(2)\n");

	return 0;
}

static int rtk_rle0599_toggle_open(struct inode *inode, struct file *file)
{
	return single_open(file, rtk_rle0599_toggle_show, inode->i_private);
}

static ssize_t rtk_rle0599_toggle_write(struct file *file,
					const char __user *ubuf, size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct rtk_phy *rtk_phy = s->private;
	char buf[32];

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "1", 1))
		do_rtk_phy_toggle(rtk_phy, false);

	return count;
}

static const struct file_operations rtk_rle0599_toggle_fops = {
	.open			= rtk_rle0599_toggle_open,
	.write			= rtk_rle0599_toggle_write,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static inline void create_debug_files(struct rtk_phy *rtk_phy)
{
	struct dentry *phy_debug_root;

	phy_debug_root = create_phy_debug_root();
	if (!phy_debug_root)
		return;

	rtk_phy->debug_dir = debugfs_create_dir(dev_name(rtk_phy->dev), phy_debug_root);

	debugfs_create_file("parameter", 0444, rtk_phy->debug_dir, rtk_phy,
			    &rtk_rle0599_parameter_fops);

	debugfs_create_file("set_parameter", 0644, rtk_phy->debug_dir, rtk_phy,
			    &rtk_rle0599_set_parameter_fops);

	debugfs_create_file("toggle", 0644, rtk_phy->debug_dir, rtk_phy,
			    &rtk_rle0599_toggle_fops);
}

static inline void remove_debug_files(struct rtk_phy *rtk_phy)
{
	debugfs_remove_recursive(rtk_phy->debug_dir);
}
#else
static inline void create_debug_files(struct rtk_phy *rtk_phy) { }
static inline void remove_debug_files(struct rtk_phy *rtk_phy) { }
#endif /* CONFIG_DEBUG_FS */

static int rtk_usb_rle0599_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct rtk_phy *rtk_phy;
	struct reg_addr *addr;
	struct rtk_phy_data *phy_data;
	struct phy *generic_phy;
	struct phy_provider *phy_provider;
	const struct rtk_phy_cfg *cfg;

	rtk_phy = devm_kzalloc(dev, sizeof(*rtk_phy), GFP_KERNEL);
	if (!rtk_phy)
		return -ENOMEM;

	rtk_phy->dev = dev;

	cfg = of_device_get_match_data(dev);
	if (!cfg) {
		dev_err(dev, "phy config is not assigned!\n");
		return -EINVAL;
	}

	rtk_phy->num_phy = cfg->num_phy;

	addr = devm_kzalloc(dev, sizeof(*addr), GFP_KERNEL);
	if (!addr)
		return -ENOMEM;

	addr->reg_wrap_vstatus_out2 = of_iomap(dev->of_node, 0);
	addr->reg_ehci_insnreg05 = of_iomap(dev->of_node, 1);
	rtk_phy->reg_addr = addr;

	dev_dbg(dev, "%s reg_wrap_vstatus_out2=%p\n", __func__, addr->reg_wrap_vstatus_out2);
	dev_dbg(dev, "%s reg_ehci_insnreg05=%p\n", __func__, addr->reg_ehci_insnreg05);

	phy_data = devm_kzalloc(dev, sizeof(*phy_data), GFP_KERNEL);
	if (!phy_data)
		return -ENOMEM;

	/* copied, because the efuse and debugfs paths rewrite the values */
	phy_data->page0_size = cfg->page0_size;
	phy_data->page0 = devm_kmemdup(dev, cfg->page0,
				       sizeof(*cfg->page0) * cfg->page0_size,
				       GFP_KERNEL);
	if (!phy_data->page0)
		return -ENOMEM;

	phy_data->page1_size = cfg->page1_size;
	phy_data->page1 = devm_kmemdup(dev, cfg->page1,
				       sizeof(*cfg->page1) * cfg->page1_size,
				       GFP_KERNEL);
	if (!phy_data->page1)
		return -ENOMEM;

	rtk_phy->chip_id = get_rtd_chip_id();
	rtk_phy->chip_revision = get_rtd_chip_revision();

	dev_info(dev, "%s: Chip %x revision is %x\n", __func__,
		 rtk_phy->chip_id, rtk_phy->chip_revision);

	rtk_phy->phy_data = phy_data;

	phy_data->do_toggle = cfg->do_toggle;
	phy_data->check_efuse = cfg->check_efuse;
	phy_data->use_default_parameter = cfg->use_default_parameter;

	platform_set_drvdata(pdev, rtk_phy);

	generic_phy = devm_phy_create(dev, NULL, &ops);
	if (IS_ERR(generic_phy))
		return PTR_ERR(generic_phy);

	phy_set_drvdata(generic_phy, rtk_phy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	create_debug_files(rtk_phy);

	dev_info(dev, "Probe RTK USB 2.0 RLE0599 PHY\n");

	return 0;
}

static void rtk_usb_rle0599_phy_remove(struct platform_device *pdev)
{
	struct rtk_phy *rtk_phy = platform_get_drvdata(pdev);

	remove_debug_files(rtk_phy);
}

static const struct of_device_id usb_phy_rle0599_rtk_dt_ids[] = {
	{ .compatible = "realtek,rtd1195-rle0599-usb2phy",
	  .data = &rtd1195_phy_cfg },
	{},
};
MODULE_DEVICE_TABLE(of, usb_phy_rle0599_rtk_dt_ids);

static struct platform_driver rtk_usb_rle0599_phy_driver = {
	.probe		= rtk_usb_rle0599_phy_probe,
	.remove		= rtk_usb_rle0599_phy_remove,
	.driver		= {
		.name	= RTK_USB_RLE0599_PHY_NAME,
		.of_match_table = usb_phy_rle0599_rtk_dt_ids,
	},
};

module_platform_driver(rtk_usb_rle0599_phy_driver);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" RTK_USB_RLE0599_PHY_NAME);
MODULE_DESCRIPTION("Realtek RLE0599 USB 2.0 PHY driver");
