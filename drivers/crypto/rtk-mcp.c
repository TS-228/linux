// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek MCP (Media Content Protection) crypto accelerator driver.
 *
 * Supports AES-128 ECB/CBC on the RTD1195/RTD1295/RTD1395 SoC family.
 * The hardware uses a descriptor ring with 56-byte descriptors and a
 * simple GO/poll completion model (SCPU MCP window at +0x100).
 *
 * Copyright (c) 2026, Stephan
 */

#include <crypto/aes.h>
#include <crypto/algapi.h>
#include <crypto/internal/skcipher.h>
#include <crypto/scatterwalk.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

/* ---- Register offsets from CP_REG_BASE (SCPU MCP window at +0x100) ---- */
#define K_MCP_CTRL		0x100
#define K_MCP_STATUS		0x104
#define K_MCP_EN		0x108
#define K_MCP_BASE		0x10c
#define K_MCP_LIMIT		0x110
#define K_MCP_RDPTR		0x114
#define K_MCP_WRPTR		0x118
#define K_MCP_DES_COUNT		0x134

/* CTRL bits (vendor rtk_mcp.h / u-boot mcp.c) */
#define MCP_CTRL_WRITE_DATA	0x01
#define MCP_CTRL_GO		0x02
#define MCP_CTRL_IDLE		0x04
#define MCP_CTRL_CLEAR		0x10

/* STATUS bits */
#define MCP_STATUS_RING_EMPTY	0x02
#define MCP_STATUS_ERROR	0x04
#define MCP_STATUS_DONE_MASK	(MCP_STATUS_RING_EMPTY | MCP_STATUS_ERROR)

/* Mode word encoding (from vendor u-boot mcp.c) */
#define MCP_MODE_AES		0x0005
#define MCP_MODE_ECB		0x0000
#define MCP_MODE_CBC		0x0040
#define MCP_MODE_ENCRYPT	0x0020

/* Descriptor: 14 x u32 = 56 bytes */
#define MCP_DESC_WORDS		14
#define MCP_DESC_SIZE		(MCP_DESC_WORDS * 4)

struct mcp_descriptor {
	__le32 mode;
	__le32 key[6];
	__le32 ini_key[4];
	__le32 src_addr;
	__le32 dst_addr;
	__le32 length;
} __packed;

struct rtk_mcp_dev {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rst;
	struct mutex engine_lock;

	/* Descriptor ring: 2 slots (HW needs WRPTR != LIMIT) */
	struct mcp_descriptor *ring;
	dma_addr_t ring_dma;

	/* Bounce buffers for src/dst data */
	void *buf_src;
	dma_addr_t buf_src_dma;
	void *buf_dst;
	dma_addr_t buf_dst_dma;
};

#define MCP_BUF_SIZE		(64 * 1024)

struct rtk_mcp_ctx {
	struct rtk_mcp_dev *mdev;
	u32 key[AES_KEYSIZE_128 / 4];
	unsigned int keylen;
};

/* Global device pointer (single instance) */
static struct rtk_mcp_dev *g_mdev;

static inline void mcp_write(struct rtk_mcp_dev *mdev, u32 reg, u32 val)
{
	writel_relaxed(val, mdev->base + reg);
}

static inline u32 mcp_read(struct rtk_mcp_dev *mdev, u32 reg)
{
	return readl_relaxed(mdev->base + reg);
}

/*
 * AES normally completes within microseconds. The vendor driver polls for
 * up to (0x3ff << 2) iterations of up to 1ms each (~4s worst case) for
 * every operation regardless; mirror that budget for robustness against
 * occasional slow completions.
 */
#define MCP_POLL_BUSY_LOOPS	1000
#define MCP_POLL_SLEEP_LOOPS	(0x3ff << 2)

static int rtk_mcp_run(struct rtk_mcp_dev *mdev)
{
	u32 status = 0;
	int timeout, clear_timeout;

	mcp_write(mdev, K_MCP_DES_COUNT, 0);
	mcp_write(mdev, K_MCP_CTRL, MCP_CTRL_CLEAR | MCP_CTRL_WRITE_DATA);

	for (clear_timeout = 0; clear_timeout < 30; clear_timeout++) {
		if (!(mcp_read(mdev, K_MCP_CTRL) & MCP_CTRL_CLEAR))
			break;
		cpu_relax();
	}

	mcp_write(mdev, K_MCP_EN, 0xfe);
	mcp_write(mdev, K_MCP_STATUS, 0xfe);

	mcp_write(mdev, K_MCP_BASE, mdev->ring_dma);
	mcp_write(mdev, K_MCP_LIMIT, mdev->ring_dma + MCP_DESC_SIZE * 2);
	mcp_write(mdev, K_MCP_RDPTR, mdev->ring_dma);
	mcp_write(mdev, K_MCP_WRPTR, mdev->ring_dma + MCP_DESC_SIZE);

	dma_wmb();
	mcp_write(mdev, K_MCP_CTRL, MCP_CTRL_GO | MCP_CTRL_WRITE_DATA);

	for (timeout = 0; timeout < MCP_POLL_BUSY_LOOPS; timeout++) {
		u32 ctrl = mcp_read(mdev, K_MCP_CTRL);

		status = mcp_read(mdev, K_MCP_STATUS);
		if (!(ctrl & MCP_CTRL_GO) || (status & MCP_STATUS_DONE_MASK))
			goto done;
		udelay(1);
	}

	for (timeout = 0; timeout < MCP_POLL_SLEEP_LOOPS; timeout++) {
		u32 ctrl = mcp_read(mdev, K_MCP_CTRL);

		status = mcp_read(mdev, K_MCP_STATUS);
		if (!(ctrl & MCP_CTRL_GO) || (status & MCP_STATUS_DONE_MASK))
			break;
		usleep_range(1000, 2000);
	}

done:
	mcp_write(mdev, K_MCP_CTRL, MCP_CTRL_GO);
	mcp_write(mdev, K_MCP_STATUS, 0xfe);

	if (status & MCP_STATUS_ERROR)
		return -EIO;
	if (!(status & MCP_STATUS_RING_EMPTY))
		return -ETIMEDOUT;

	return 0;
}

static int rtk_mcp_setkey(struct crypto_skcipher *tfm, const u8 *key,
			   unsigned int keylen)
{
	struct rtk_mcp_ctx *ctx = crypto_skcipher_ctx(tfm);

	if (keylen != AES_KEYSIZE_128)
		return -EINVAL;

	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;

	return 0;
}

/*
 * The MCP engine treats key/IV words as big-endian in memory. The crypto
 * API key/IV are byte arrays — on LE, reading them as u32 and writing to
 * an LE descriptor field gives the wrong byte order. We need: memory
 * bytes = source bytes interpreted as BE words.
 */
static void mcp_store_be_words(__le32 *dst, const u8 *src, unsigned int nwords)
{
	const u32 *sp = (const u32 *)src;
	unsigned int i;

	for (i = 0; i < nwords; i++)
		dst[i] = cpu_to_le32(swab32(sp[i]));
}

static int rtk_mcp_crypt(struct skcipher_request *req, u32 mode)
{
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct rtk_mcp_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct rtk_mcp_dev *mdev = ctx->mdev;
	struct mcp_descriptor desc;
	u8 last_block[AES_BLOCK_SIZE];
	unsigned int done;
	int ret;

	if (!req->cryptlen || (req->cryptlen & (AES_BLOCK_SIZE - 1)))
		return -EINVAL;

	memset(&desc, 0, sizeof(desc));
	desc.mode = cpu_to_le32(mode);
	mcp_store_be_words(desc.key, (const u8 *)ctx->key, AES_KEYSIZE_128 / 4);

	if (mode & MCP_MODE_CBC)
		mcp_store_be_words(desc.ini_key, req->iv, AES_BLOCK_SIZE / 4);

	mutex_lock(&mdev->engine_lock);

	done = 0;

	while (done < req->cryptlen) {
		unsigned int chunk = min_t(unsigned int,
					   req->cryptlen - done, MCP_BUF_SIZE);

		scatterwalk_map_and_copy(mdev->buf_src, req->src, done,
					 chunk, 0);

		/* Build descriptor for this chunk */
		desc.src_addr = cpu_to_le32(mdev->buf_src_dma);
		desc.dst_addr = cpu_to_le32(mdev->buf_dst_dma);
		desc.length = cpu_to_le32(chunk);

		memcpy(&mdev->ring[0], &desc, MCP_DESC_SIZE);
		memset(&mdev->ring[1], 0, MCP_DESC_SIZE);
		dma_wmb();

		ret = rtk_mcp_run(mdev);
		if (ret)
			goto out;

		dma_rmb();

		/*
		 * Capture the ciphertext tail block from our bounce buffers
		 * before writing the result to req->dst — for in-place
		 * requests req->dst aliases req->src, so req->src/req->dst
		 * can no longer be trusted to hold the ciphertext once dst
		 * is written.
		 */
		if (mode & MCP_MODE_CBC) {
			const void *tail = (mode & MCP_MODE_ENCRYPT) ?
				mdev->buf_dst + chunk - AES_BLOCK_SIZE :
				mdev->buf_src + chunk - AES_BLOCK_SIZE;

			memcpy(last_block, tail, AES_BLOCK_SIZE);
			mcp_store_be_words(desc.ini_key, last_block,
					   AES_BLOCK_SIZE / 4);
		}

		scatterwalk_map_and_copy(mdev->buf_dst, req->dst, done,
					 chunk, 1);

		done += chunk;
	}

	/* Write back IV for CBC (last ciphertext block, per crypto API) */
	if ((mode & MCP_MODE_CBC) && req->iv)
		memcpy(req->iv, last_block, AES_BLOCK_SIZE);

	ret = 0;

out:
	mutex_unlock(&mdev->engine_lock);
	return ret;
}

static int rtk_mcp_ecb_encrypt(struct skcipher_request *req)
{
	return rtk_mcp_crypt(req, MCP_MODE_AES | MCP_MODE_ECB | MCP_MODE_ENCRYPT);
}

static int rtk_mcp_ecb_decrypt(struct skcipher_request *req)
{
	return rtk_mcp_crypt(req, MCP_MODE_AES | MCP_MODE_ECB);
}

static int rtk_mcp_cbc_encrypt(struct skcipher_request *req)
{
	return rtk_mcp_crypt(req, MCP_MODE_AES | MCP_MODE_CBC | MCP_MODE_ENCRYPT);
}

static int rtk_mcp_cbc_decrypt(struct skcipher_request *req)
{
	return rtk_mcp_crypt(req, MCP_MODE_AES | MCP_MODE_CBC);
}

static int rtk_mcp_init_tfm(struct crypto_skcipher *tfm)
{
	struct rtk_mcp_ctx *ctx = crypto_skcipher_ctx(tfm);

	ctx->mdev = g_mdev;
	if (!ctx->mdev)
		return -ENODEV;

	return 0;
}

static struct skcipher_alg rtk_mcp_algs[] = {
	{
		.base.cra_name		= "ecb(aes)",
		.base.cra_driver_name	= "rtk-mcp-ecb-aes",
		.base.cra_priority	= 300,
		.base.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY,
		.base.cra_blocksize	= AES_BLOCK_SIZE,
		.base.cra_ctxsize	= sizeof(struct rtk_mcp_ctx),
		.base.cra_module	= THIS_MODULE,
		.min_keysize		= AES_KEYSIZE_128,
		.max_keysize		= AES_KEYSIZE_128,
		.setkey			= rtk_mcp_setkey,
		.encrypt		= rtk_mcp_ecb_encrypt,
		.decrypt		= rtk_mcp_ecb_decrypt,
		.init			= rtk_mcp_init_tfm,
	},
	{
		.base.cra_name		= "cbc(aes)",
		.base.cra_driver_name	= "rtk-mcp-cbc-aes",
		.base.cra_priority	= 300,
		.base.cra_flags		= CRYPTO_ALG_KERN_DRIVER_ONLY,
		.base.cra_blocksize	= AES_BLOCK_SIZE,
		.base.cra_ctxsize	= sizeof(struct rtk_mcp_ctx),
		.base.cra_module	= THIS_MODULE,
		.min_keysize		= AES_KEYSIZE_128,
		.max_keysize		= AES_KEYSIZE_128,
		.ivsize			= AES_BLOCK_SIZE,
		.setkey			= rtk_mcp_setkey,
		.encrypt		= rtk_mcp_cbc_encrypt,
		.decrypt		= rtk_mcp_cbc_decrypt,
		.init			= rtk_mcp_init_tfm,
	},
};

static int rtk_mcp_probe(struct platform_device *pdev)
{
	struct rtk_mcp_dev *mdev;
	int ret;

	mdev = devm_kzalloc(&pdev->dev, sizeof(*mdev), GFP_KERNEL);
	if (!mdev)
		return -ENOMEM;

	mdev->dev = &pdev->dev;
	mutex_init(&mdev->engine_lock);

	mdev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mdev->base))
		return PTR_ERR(mdev->base);

	mdev->clk = devm_clk_get_optional(&pdev->dev, NULL);
	if (IS_ERR(mdev->clk))
		return PTR_ERR(mdev->clk);

	mdev->rst = devm_reset_control_get_optional_shared(&pdev->dev, NULL);
	if (IS_ERR(mdev->rst))
		return PTR_ERR(mdev->rst);

	ret = clk_prepare_enable(mdev->clk);
	if (ret)
		return ret;

	ret = reset_control_deassert(mdev->rst);
	if (ret) {
		clk_disable_unprepare(mdev->clk);
		return ret;
	}

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "DMA mask failed\n");
		goto err_clk;
	}

	/* Allocate DMA-coherent descriptor ring (2 slots) */
	mdev->ring = dma_alloc_coherent(&pdev->dev,
					MCP_DESC_SIZE * 2,
					&mdev->ring_dma, GFP_KERNEL);
	if (!mdev->ring) {
		ret = -ENOMEM;
		goto err_clk;
	}

	/* Allocate bounce buffers */
	mdev->buf_src = dma_alloc_coherent(&pdev->dev, MCP_BUF_SIZE,
					   &mdev->buf_src_dma, GFP_KERNEL);
	mdev->buf_dst = dma_alloc_coherent(&pdev->dev, MCP_BUF_SIZE,
					   &mdev->buf_dst_dma, GFP_KERNEL);
	if (!mdev->buf_src || !mdev->buf_dst) {
		ret = -ENOMEM;
		goto err_bufs;
	}

	platform_set_drvdata(pdev, mdev);
	g_mdev = mdev;

	ret = crypto_register_skciphers(rtk_mcp_algs,
					ARRAY_SIZE(rtk_mcp_algs));
	if (ret) {
		dev_err(&pdev->dev, "failed to register skcipher algorithms\n");
		goto err_bufs;
	}

	dev_info(&pdev->dev,
		 "Realtek MCP crypto: AES-128 ECB/CBC (ring@0x%pad)\n",
		 &mdev->ring_dma);
	return 0;

err_bufs:
	if (mdev->buf_src)
		dma_free_coherent(&pdev->dev, MCP_BUF_SIZE,
				  mdev->buf_src, mdev->buf_src_dma);
	if (mdev->buf_dst)
		dma_free_coherent(&pdev->dev, MCP_BUF_SIZE,
				  mdev->buf_dst, mdev->buf_dst_dma);
	dma_free_coherent(&pdev->dev, MCP_DESC_SIZE * 2,
			  mdev->ring, mdev->ring_dma);
err_clk:
	reset_control_assert(mdev->rst);
	clk_disable_unprepare(mdev->clk);
	return ret;
}

static void rtk_mcp_remove(struct platform_device *pdev)
{
	struct rtk_mcp_dev *mdev = platform_get_drvdata(pdev);

	crypto_unregister_skciphers(rtk_mcp_algs,
				    ARRAY_SIZE(rtk_mcp_algs));

	g_mdev = NULL;

	dma_free_coherent(&pdev->dev, MCP_BUF_SIZE,
			  mdev->buf_src, mdev->buf_src_dma);
	dma_free_coherent(&pdev->dev, MCP_BUF_SIZE,
			  mdev->buf_dst, mdev->buf_dst_dma);
	dma_free_coherent(&pdev->dev, MCP_DESC_SIZE * 2,
			  mdev->ring, mdev->ring_dma);

	reset_control_assert(mdev->rst);
	clk_disable_unprepare(mdev->clk);
}

static const struct of_device_id rtk_mcp_of_match[] = {
	{ .compatible = "Realtek,rtk-mcp" },
	{ .compatible = "realtek,rtd1195-mcp" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtk_mcp_of_match);

static struct platform_driver rtk_mcp_driver = {
	.probe	= rtk_mcp_probe,
	.remove	= rtk_mcp_remove,
	.driver	= {
		.name		= "rtk-mcp-crypto",
		.of_match_table	= rtk_mcp_of_match,
	},
};
module_platform_driver(rtk_mcp_driver);

MODULE_AUTHOR("Stephan");
MODULE_DESCRIPTION("Realtek MCP crypto accelerator (AES-128 ECB/CBC)");
MODULE_LICENSE("GPL");
