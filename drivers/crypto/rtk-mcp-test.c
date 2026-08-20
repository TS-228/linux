// SPDX-License-Identifier: GPL-2.0-only
/*
 * Quick load-time test for the rtk-mcp crypto driver.
 * Runs NIST AES-128-ECB/CBC known-answer tests via the kernel crypto API.
 * Load: insmod rtk-mcp-test.ko   (prints PASS/FAIL to dmesg, then stays loaded)
 */

#include <crypto/skcipher.h>
#include <crypto/hash.h>
#include <crypto/sha1.h>
#include <crypto/sha2.h>
#include <linux/module.h>
#include <linux/scatterlist.h>

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

struct sha_test_vec {
	const char *name;
	const char *alg;
	const u8 *data;
	u32 data_len;
	const u8 *digest;
	unsigned int digest_len;
};

static const u8 sha1_empty[] = {
	0xda, 0x39, 0xa3, 0xee, 0x5e, 0x6b, 0x4b, 0x0d,
	0x32, 0x55, 0xbf, 0xef, 0x95, 0x60, 0x18, 0x90,
	0xaf, 0xd8, 0x07, 0x09,
};

static const u8 sha1_abc[] = {
	0xa9, 0x99, 0x3e, 0x36, 0x47, 0x06, 0x81, 0x6a,
	0xba, 0x3e, 0x25, 0x71, 0x78, 0x50, 0xc2, 0x6c,
	0x9c, 0xd0, 0xd8, 0x9d,
};

static const u8 sha256_empty[] = {
	0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14,
	0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
	0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c,
	0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55,
};

static const u8 sha256_abc[] = {
	0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea,
	0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
	0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c,
	0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad,
};

static const u8 abc_data[] = { 'a', 'b', 'c' };

static const struct sha_test_vec sha_vectors[] = {
	{
		.name = "sha1(empty)",
		.alg = "sha1",
		.data = (const u8 *)"",
		.data_len = 0,
		.digest = sha1_empty,
		.digest_len = 20,
	},
	{
		.name = "sha1(abc)",
		.alg = "sha1",
		.data = abc_data,
		.data_len = 3,
		.digest = sha1_abc,
		.digest_len = 20,
	},
	{
		.name = "sha256(empty)",
		.alg = "sha256",
		.data = (const u8 *)"",
		.data_len = 0,
		.digest = sha256_empty,
		.digest_len = 32,
	},
	{
		.name = "sha256(abc)",
		.alg = "sha256",
		.data = abc_data,
		.data_len = 3,
		.digest = sha256_abc,
		.digest_len = 32,
	},
};

static int run_sha_digest(const struct sha_test_vec *tv)
{
	struct crypto_ahash *tfm;
	struct ahash_request *req;
	struct scatterlist sg;
	DECLARE_CRYPTO_WAIT(wait);
	u8 *out;
	u8 *data;
	int ret;

	data = (u8 *)tv->data;

	tfm = crypto_alloc_ahash(tv->alg, 0, 0);
	if (IS_ERR(tfm)) {
		pr_err("rtk-mcp-test: alloc %s failed: %ld\n",
		       tv->alg, PTR_ERR(tfm));
		return PTR_ERR(tfm);
	}

	out = kmalloc(tv->digest_len, GFP_KERNEL);
	if (!out) {
		ret = -ENOMEM;
		goto out_tfm;
	}

	req = ahash_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		ret = -ENOMEM;
		goto out_buf;
	}

	ahash_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				 crypto_req_done, &wait);

	sg_init_one(&sg, data, tv->data_len);
	ahash_request_set_crypt(req, &sg, out, tv->data_len);

	ret = crypto_wait_req(crypto_ahash_digest(req), &wait);
	if (ret)
		goto out_req;

	if (!memcmp(out, tv->digest, tv->digest_len)) {
		pr_info("rtk-mcp-test: %s digest PASSED\n", tv->name);
		ret = 0;
	} else {
		pr_err("rtk-mcp-test: %s digest FAILED\n", tv->name);
		print_hex_dump(KERN_ERR, "  got:      ", DUMP_PREFIX_NONE,
			       tv->digest_len, 1, out, tv->digest_len, false);
		print_hex_dump(KERN_ERR, "  expected: ", DUMP_PREFIX_NONE,
			       tv->digest_len, 1, tv->digest, tv->digest_len, false);
		ret = -EINVAL;
	}

out_req:
	ahash_request_free(req);
out_buf:
	kfree(out);
out_tfm:
	crypto_free_ahash(tfm);
	return ret;
}

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

	pr_info("rtk-mcp-test: running %zu SHA vectors\n",
		ARRAY_SIZE(sha_vectors));
	for (i = 0; i < ARRAY_SIZE(sha_vectors); i++) {
		ret = run_sha_digest(&sha_vectors[i]);
		if (ret)
			fail++;
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
