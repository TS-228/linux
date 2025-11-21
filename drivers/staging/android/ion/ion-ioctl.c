// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2011 Google, Inc.
 */

#include <linux/kernel.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#include "ion.h"

#if defined(CONFIG_ION_RTK)
extern int ion_phys(struct ion_client *client, struct ion_handle *handle,
	phys_addr_t *addr, size_t *len);
#if defined(CONFIG_ARM)
#define COMPAT_ION_IOC_PHYS _IOWR(ION_IOC_MAGIC, 8, struct ion_phys_data)
#endif /* CONFIG_ARM 32bit */

#endif /* CONFIG_ION_RTK */

union ion_ioctl_arg {
	struct ion_allocation_data allocation;
	struct ion_heap_query query;
#if defined(CONFIG_ION_RTK)
	struct ion_phys_data phys;
#endif /* CONFIG_ION_RTK */
};

static int validate_ioctl_arg(unsigned int cmd, union ion_ioctl_arg *arg)
{
	switch (cmd) {
	case ION_IOC_HEAP_QUERY:
		if (arg->query.reserved0 ||
		    arg->query.reserved1 ||
		    arg->query.reserved2)
			return -EINVAL;
		break;
	default:
		break;
	}

	return 0;
}

/* fix up the cases where the ioctl direction bits are incorrect */
static unsigned int ion_ioctl_dir(unsigned int cmd)
{
	switch (cmd) {
	default:
		return _IOC_DIR(cmd);
	}
}

long ion_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	unsigned int dir;
	union ion_ioctl_arg data;

	dir = ion_ioctl_dir(cmd);

	if (_IOC_SIZE(cmd) > sizeof(data))
		return -EINVAL;

	/*
	 * The copy_from_user is unconditional here for both read and write
	 * to do the validate. If there is no write for the ioctl, the
	 * buffer is cleared
	 */
	if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd)))
		return -EFAULT;

	ret = validate_ioctl_arg(cmd, &data);
	if (ret) {
		pr_warn_once("%s: ioctl validate failed\n", __func__);
		return ret;
	}

	if (!(dir & _IOC_WRITE))
		memset(&data, 0, sizeof(data));

	switch (cmd) {
	case ION_IOC_ALLOC:
	{
		int fd;

		fd = ion_alloc(data.allocation.len,
			       data.allocation.heap_id_mask,
			       data.allocation.flags);
		if (fd < 0)
			return fd;

		data.allocation.fd = fd;

		mutex_lock(&client->lock);
		handle = ion_handle_get_by_id_nolock(client,
						     data.handle.handle);
		if (IS_ERR(handle)) {
			mutex_unlock(&client->lock);
			return PTR_ERR(handle);
		}
		ion_free_nolock(client, handle);
		ion_handle_put_nolock(handle);
		mutex_unlock(&client->lock);
		break;
	}
	/* 20130208 charleslin: add ioctl to get physical address */
#if defined(CONFIG_ION_RTK)
#if defined(CONFIG_ARM)
    case COMPAT_ION_IOC_PHYS:
#endif /* CONFIG_ARM 32bit */
	case ION_IOC_PHYS:
	{
		int ret;
		phys_addr_t addr;
		size_t len;
		struct ion_handle *handle;

		if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
			return -EFAULT;

		mutex_lock(&client->lock);
		handle = ion_handle_get_by_id_nolock(client, data.phys.handle);
		if (IS_ERR(handle))
			mutex_unlock(&client->lock);
			return PTR_ERR(handle);
		ret = ion_phys(client, handle, &addr, &len);

		ion_handle_put_nolock(handle);
		mutex_unlock(&client->lock);
		pr_debug("%s: addr:%lx len:%lx\n", __func__, addr, len);
		data.phys.addr = addr;
		data.phys.len = len;
		if(ret != 0)
			return ret;
		if (copy_to_user((void __user *)arg, &data.phys, sizeof(data.phys)))
			return -EFAULT;
		break;
	}
#endif /* CONFIG_ION_RTK */
	case ION_IOC_SHARE:
	case ION_IOC_MAP:
	{
		struct ion_handle *handle;

		handle = ion_handle_get_by_id(client, data.handle.handle);
		if (IS_ERR(handle))
			return PTR_ERR(handle);
		data.fd.fd = ion_share_dma_buf_fd(client, handle);
		ion_handle_put(handle);
		if (data.fd.fd < 0)
			ret = data.fd.fd;
		break;
	}
	case ION_IOC_IMPORT:
	{
		struct ion_handle *handle;

		handle = ion_import_dma_buf_fd(client, data.fd.fd);
		if (IS_ERR(handle))
			ret = PTR_ERR(handle);
		else
			data.handle.handle = handle->id;
		break;
	}
	case ION_IOC_SYNC:
	{
		ret = ion_sync_for_device(client, data.fd.fd);
		break;
	}
	case ION_IOC_CUSTOM:
	{
		if (!dev->custom_ioctl)
			return -ENOTTY;
		ret = dev->custom_ioctl(client, data.custom.cmd,
						data.custom.arg);
		break;
	}
	case ION_IOC_HEAP_QUERY:
		ret = ion_query_heaps(&data.query);
		break;
	default:
		return -ENOTTY;
	}

	if (dir & _IOC_READ) {
		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd)))
			return -EFAULT;
	}
	return ret;
}
