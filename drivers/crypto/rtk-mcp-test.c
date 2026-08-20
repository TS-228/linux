// SPDX-License-Identifier: GPL-2.0-only
/*
 * Quick load-time test for the rtk-mcp crypto driver.
 * Runs NIST AES-128-ECB/CBC known-answer tests via the kernel crypto API.
 * Load: insmod rtk-mcp-test.ko   (prints PASS/FAIL to dmesg, then stays loaded)
 */

#include <crypto/skcipher.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/scatterlist.h>
#include <linux/vmalloc.h>

struct test_vec {
	const char *name;
	const char *alg;
	const u8 key[16];
	const u8 iv[16];
	const u8 plain[16];
	const u8 cipher[16];
};

static const struct test_vec vectors[] = {
	{
		.name	= "AES-128-ECB encrypt",
		.alg	= "ecb(aes)",
		.key	= { 0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
			    0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c },
		.plain	= { 0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
			    0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a },
		.cipher	= { 0x3a,0xd7,0x7b,0xb4,0x0d,0x7a,0x36,0x60,
			    0xa8,0x9e,0xca,0xf3,0x24,0x66,0xef,0x97 },
	},
	{
		.name	= "AES-128-CBC encrypt",
		.alg	= "cbc(aes)",
		.key	= { 0x2b,0x7e,0x15,0x16,0x28,0xae,0xd2,0xa6,
			    0xab,0xf7,0x15,0x88,0x09,0xcf,0x4f,0x3c },
		.iv	= { 0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
			    0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f },
		.plain	= { 0x6b,0xc1,0xbe,0xe2,0x2e,0x40,0x9f,0x96,
			    0xe9,0x3d,0x7e,0x11,0x73,0x93,0x17,0x2a },
		.cipher	= { 0x76,0x49,0xab,0xac,0x81,0x19,0xb2,0x46,
			    0xce,0xe9,0x8e,0x9b,0x12,0xe9,0x19,0x7d },
	},
};

static int run_test(const struct test_vec *tv, bool encrypt)
{
	struct crypto_skcipher *tfm;
	struct skcipher_request *req;
	struct scatterlist sg_src, sg_dst;
	DECLARE_CRYPTO_WAIT(wait);
	u8 *src, *dst;
	u8 iv[16];
	int ret;
	const u8 *input = encrypt ? tv->plain : tv->cipher;
	const u8 *expected = encrypt ? tv->cipher : tv->plain;
	const char *dir = encrypt ? "encrypt" : "decrypt";

	tfm = crypto_alloc_skcipher(tv->alg, 0, 0);
	if (IS_ERR(tfm)) {
		pr_err("rtk-mcp-test: alloc %s failed: %ld\n",
		       tv->alg, PTR_ERR(tfm));
		return PTR_ERR(tfm);
	}

	/* Check we got the HW driver */
	pr_info("rtk-mcp-test: %s %s using driver=%s\n",
		tv->name, dir,
		crypto_skcipher_driver_name(tfm));

	ret = crypto_skcipher_setkey(tfm, tv->key, 16);
	if (ret) {
		pr_err("rtk-mcp-test: setkey failed: %d\n", ret);
		goto out_tfm;
	}

	req = skcipher_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		ret = -ENOMEM;
		goto out_tfm;
	}

	src = kmalloc(16, GFP_KERNEL);
	dst = kmalloc(16, GFP_KERNEL);
	if (!src || !dst) {
		ret = -ENOMEM;
		goto out_mem;
	}

	memcpy(src, input, 16);
	memcpy(iv, tv->iv, 16);

	sg_init_one(&sg_src, src, 16);
	sg_init_one(&sg_dst, dst, 16);

	skcipher_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				      crypto_req_done, &wait);
	skcipher_request_set_crypt(req, &sg_src, &sg_dst, 16,
				   crypto_skcipher_ivsize(tfm) ? iv : NULL);

	if (encrypt)
		ret = crypto_wait_req(crypto_skcipher_encrypt(req), &wait);
	else
		ret = crypto_wait_req(crypto_skcipher_decrypt(req), &wait);

	if (ret) {
		pr_err("rtk-mcp-test: %s %s failed: %d\n",
		       tv->name, dir, ret);
		goto out_mem;
	}

	if (memcmp(dst, expected, 16) == 0) {
		pr_info("rtk-mcp-test: %s %s PASSED\n", tv->name, dir);
		ret = 0;
	} else {
		pr_err("rtk-mcp-test: %s %s FAILED\n", tv->name, dir);
		print_hex_dump(KERN_ERR, "  got:      ", DUMP_PREFIX_NONE,
			       16, 1, dst, 16, false);
		print_hex_dump(KERN_ERR, "  expected: ", DUMP_PREFIX_NONE,
			       16, 1, expected, 16, false);
		ret = -EINVAL;
	}

out_mem:
	kfree(src);
	kfree(dst);
	skcipher_request_free(req);
out_tfm:
	crypto_free_skcipher(tfm);
	return ret;
}

/*
 * MCP_BUF_SIZE in rtk-mcp.c is 64KiB: skcipher requests larger than that
 * are split into multiple hardware transfers, chained via IV. This is the
 * one code path the small NIST vectors above never exercise. Cross-check
 * against the CPU reference implementation at sizes that land exactly on,
 * one block over, and several multiples past that boundary.
 */
#define MCP_TEST_BUF_SIZE	(64 * 1024)

static int do_skcipher(struct crypto_skcipher *tfm, const u8 *src, u8 *dst,
			size_t len, u8 *iv, bool encrypt)
{
	struct skcipher_request *req;
	struct scatterlist sg_src, sg_dst;
	DECLARE_CRYPTO_WAIT(wait);
	int ret;

	req = skcipher_request_alloc(tfm, GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	sg_init_one(&sg_src, src, len);
	sg_init_one(&sg_dst, dst, len);
	skcipher_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				      crypto_req_done, &wait);
	skcipher_request_set_crypt(req, &sg_src, &sg_dst, len,
				   crypto_skcipher_ivsize(tfm) ? iv : NULL);

	ret = crypto_wait_req(encrypt ? crypto_skcipher_encrypt(req)
				      : crypto_skcipher_decrypt(req), &wait);
	skcipher_request_free(req);
	return ret;
}

static int run_large_cbc_test(const char *hw_driver, const char *cpu_driver,
			       size_t size)
{
	struct crypto_skcipher *hw_tfm, *cpu_tfm;
	u8 key[16], iv_orig[16], iv_hw[16], iv_cpu[16];
	u8 *src, *hw_ct, *cpu_ct, *hw_pt;
	int ret;

	hw_tfm = crypto_alloc_skcipher(hw_driver, 0, 0);
	if (IS_ERR(hw_tfm)) {
		pr_err("rtk-mcp-test: alloc %s failed: %ld\n",
		       hw_driver, PTR_ERR(hw_tfm));
		return PTR_ERR(hw_tfm);
	}
	cpu_tfm = crypto_alloc_skcipher(cpu_driver, 0, 0);
	if (IS_ERR(cpu_tfm)) {
		pr_err("rtk-mcp-test: alloc %s failed: %ld\n",
		       cpu_driver, PTR_ERR(cpu_tfm));
		crypto_free_skcipher(hw_tfm);
		return PTR_ERR(cpu_tfm);
	}

	src = vmalloc(size);
	hw_ct = vmalloc(size);
	cpu_ct = vmalloc(size);
	hw_pt = vmalloc(size);
	if (!src || !hw_ct || !cpu_ct || !hw_pt) {
		ret = -ENOMEM;
		goto out;
	}

	get_random_bytes(key, sizeof(key));
	get_random_bytes(iv_orig, sizeof(iv_orig));
	get_random_bytes(src, size);

	ret = crypto_skcipher_setkey(hw_tfm, key, sizeof(key));
	if (!ret)
		ret = crypto_skcipher_setkey(cpu_tfm, key, sizeof(key));
	if (ret) {
		pr_err("rtk-mcp-test: large cbc %zu: setkey failed: %d\n",
		       size, ret);
		goto out;
	}

	/* Encrypt with both HW and CPU from the same IV; ciphertext and the
	 * post-op IV (per crypto API: last ciphertext block) must match.
	 */
	memcpy(iv_hw, iv_orig, sizeof(iv_orig));
	memcpy(iv_cpu, iv_orig, sizeof(iv_orig));
	ret = do_skcipher(hw_tfm, src, hw_ct, size, iv_hw, true);
	if (ret) {
		pr_err("rtk-mcp-test: large cbc %zu: HW encrypt failed: %d\n",
		       size, ret);
		goto out;
	}
	ret = do_skcipher(cpu_tfm, src, cpu_ct, size, iv_cpu, true);
	if (ret) {
		pr_err("rtk-mcp-test: large cbc %zu: CPU encrypt failed: %d\n",
		       size, ret);
		goto out;
	}

	if (memcmp(hw_ct, cpu_ct, size) || memcmp(iv_hw, iv_cpu, 16)) {
		pr_err("rtk-mcp-test: large cbc %zu bytes encrypt MISMATCH vs %s\n",
		       size, cpu_driver);
		ret = -EINVAL;
		goto out;
	}

	/* Round-trip: HW-decrypt its own (in-place) ciphertext from the
	 * original IV; must recover the plaintext, with the output IV again
	 * matching the last ciphertext block. This is exactly the in-place
	 * decrypt path that had the "wrong output IV" bug.
	 */
	memcpy(hw_pt, hw_ct, size);
	memcpy(iv_hw, iv_orig, sizeof(iv_orig));
	ret = do_skcipher(hw_tfm, hw_pt, hw_pt, size, iv_hw, false);
	if (ret) {
		pr_err("rtk-mcp-test: large cbc %zu: HW in-place decrypt failed: %d\n",
		       size, ret);
		goto out;
	}

	if (memcmp(hw_pt, src, size)) {
		pr_err("rtk-mcp-test: large cbc %zu bytes in-place decrypt MISMATCH\n",
		       size);
		ret = -EINVAL;
		goto out;
	}
	if (memcmp(iv_hw, cpu_ct + size - 16, 16)) {
		pr_err("rtk-mcp-test: large cbc %zu bytes decrypt output IV MISMATCH\n",
		       size);
		ret = -EINVAL;
		goto out;
	}

	ret = 0;
out:
	vfree(src);
	vfree(hw_ct);
	vfree(cpu_ct);
	vfree(hw_pt);
	crypto_free_skcipher(hw_tfm);
	crypto_free_skcipher(cpu_tfm);
	if (ret == 0)
		pr_info("rtk-mcp-test: large cbc %zu bytes (%s vs %s) PASSED\n",
			size, hw_driver, cpu_driver);
	return ret;
}

static int __init rtk_mcp_test_init(void)
{
	int i, ret, fail = 0;

	pr_info("rtk-mcp-test: running %zu test vectors\n",
		ARRAY_SIZE(vectors));

	for (i = 0; i < ARRAY_SIZE(vectors); i++) {
		ret = run_test(&vectors[i], true);
		if (ret)
			fail++;
		ret = run_test(&vectors[i], false);
		if (ret)
			fail++;
	}

	pr_info("rtk-mcp-test: large-buffer cbc(aes) chunk-boundary tests vs cbc(aes-generic)\n");
	{
		static const size_t cbc_sizes[] = {
			MCP_TEST_BUF_SIZE,		/* exactly 1 chunk */
			MCP_TEST_BUF_SIZE + 16,	/* 1 block into chunk 2 */
			2 * MCP_TEST_BUF_SIZE,		/* exactly 2 chunks */
			3 * MCP_TEST_BUF_SIZE + 128,	/* partial last chunk */
		};

		for (i = 0; i < ARRAY_SIZE(cbc_sizes); i++) {
			ret = run_large_cbc_test("rtk-mcp-cbc-aes",
						 "cbc(aes-generic)",
						 cbc_sizes[i]);
			if (ret)
				fail++;
		}
	}

	if (fail)
		pr_err("rtk-mcp-test: %d tests FAILED\n", fail);
	else
		pr_info("rtk-mcp-test: ALL TESTS PASSED\n");

	return 0;
}

static void __exit rtk_mcp_test_exit(void)
{
}

module_init(rtk_mcp_test_init);
module_exit(rtk_mcp_test_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Realtek MCP crypto driver test");
