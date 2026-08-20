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

static int do_ahash_digest(struct crypto_ahash *tfm, const u8 *data,
			    size_t len, u8 *out)
{
	struct ahash_request *req;
	struct scatterlist sg;
	DECLARE_CRYPTO_WAIT(wait);
	int ret;

	req = ahash_request_alloc(tfm, GFP_KERNEL);
	if (!req)
		return -ENOMEM;

	sg_init_one(&sg, data, len ? len : 1);
	ahash_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				   crypto_req_done, &wait);
	ahash_request_set_crypt(req, &sg, out, len);
	ret = crypto_wait_req(crypto_ahash_digest(req), &wait);
	ahash_request_free(req);
	return ret;
}

/*
 * The driver buffers the whole message before hashing, capped at
 * MCP_BUF_SIZE (64KiB) in rtk-mcp.c. Verify correctness right up to that
 * cap against the CPU reference, and confirm the kernel-API-legal case of
 * a too-large request is rejected cleanly (-E2BIG) rather than corrupting
 * memory or hanging.
 */
static int run_sha_boundary_test(const char *hw_alg, const char *cpu_alg,
				  size_t size)
{
	struct crypto_ahash *hw_tfm, *cpu_tfm;
	u8 *data, *hw_digest, *cpu_digest;
	unsigned int digest_size;
	int ret;

	hw_tfm = crypto_alloc_ahash(hw_alg, 0, 0);
	if (IS_ERR(hw_tfm))
		return PTR_ERR(hw_tfm);
	cpu_tfm = crypto_alloc_ahash(cpu_alg, 0, 0);
	if (IS_ERR(cpu_tfm)) {
		ret = PTR_ERR(cpu_tfm);
		crypto_free_ahash(hw_tfm);
		return ret;
	}

	digest_size = crypto_ahash_digestsize(hw_tfm);
	data = vmalloc(size ? size : 1);
	hw_digest = kmalloc(digest_size, GFP_KERNEL);
	cpu_digest = kmalloc(digest_size, GFP_KERNEL);
	if (!data || !hw_digest || !cpu_digest) {
		ret = -ENOMEM;
		goto out;
	}
	if (size)
		get_random_bytes(data, size);

	ret = do_ahash_digest(hw_tfm, data, size, hw_digest);
	if (ret) {
		pr_err("rtk-mcp-test: sha boundary %s %zu bytes: HW failed: %d\n",
		       hw_alg, size, ret);
		goto out;
	}
	ret = do_ahash_digest(cpu_tfm, data, size, cpu_digest);
	if (ret) {
		pr_err("rtk-mcp-test: sha boundary %s %zu bytes: CPU failed: %d\n",
		       hw_alg, size, ret);
		goto out;
	}

	if (memcmp(hw_digest, cpu_digest, digest_size)) {
		pr_err("rtk-mcp-test: sha boundary %s %zu bytes MISMATCH vs %s\n",
		       hw_alg, size, cpu_alg);
		ret = -EINVAL;
		goto out;
	}

	ret = 0;
out:
	vfree(data);
	kfree(hw_digest);
	kfree(cpu_digest);
	crypto_free_ahash(hw_tfm);
	crypto_free_ahash(cpu_tfm);
	if (ret == 0)
		pr_info("rtk-mcp-test: sha boundary %s %zu bytes PASSED\n",
			hw_alg, size);
	return ret;
}

/* One byte over the driver's buffering cap must be rejected, not hang or
 * corrupt memory -- this is a legal request under the ahash API (no size
 * limit there), just one this driver's design can't service.
 */
static int run_sha_too_large_test(const char *hw_alg, size_t size)
{
	struct crypto_ahash *tfm;
	u8 *data, *digest;
	int ret;

	tfm = crypto_alloc_ahash(hw_alg, 0, 0);
	if (IS_ERR(tfm))
		return PTR_ERR(tfm);

	data = vmalloc(size);
	digest = kmalloc(crypto_ahash_digestsize(tfm), GFP_KERNEL);
	if (!data || !digest) {
		ret = -ENOMEM;
		goto out;
	}
	get_random_bytes(data, size);

	ret = do_ahash_digest(tfm, data, size, digest);
	if (ret == -E2BIG) {
		pr_info("rtk-mcp-test: sha oversize %s %zu bytes correctly rejected (-E2BIG) PASSED\n",
			hw_alg, size);
		ret = 0;
	} else {
		pr_err("rtk-mcp-test: sha oversize %s %zu bytes: expected -E2BIG, got %d\n",
		       hw_alg, size, ret);
		ret = ret ? ret : -EINVAL; /* ret==0 would itself be wrong here */
	}
out:
	vfree(data);
	kfree(digest);
	crypto_free_ahash(tfm);
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

	pr_info("rtk-mcp-test: sha1/sha256 size-boundary tests vs generic\n");
	{
		static const size_t sha_sizes[] = {
			0, 1, 55, 56, 63, 64, 65, 127, 128, 999,
			MCP_TEST_BUF_SIZE - 1, MCP_TEST_BUF_SIZE,
		};

		for (i = 0; i < ARRAY_SIZE(sha_sizes); i++) {
			ret = run_sha_boundary_test("rtk-mcp-sha1", "sha1-generic",
						    sha_sizes[i]);
			if (ret)
				fail++;
			ret = run_sha_boundary_test("rtk-mcp-sha256", "sha256-generic",
						    sha_sizes[i]);
			if (ret)
				fail++;
		}

		ret = run_sha_too_large_test("rtk-mcp-sha1", MCP_TEST_BUF_SIZE + 1);
		if (ret)
			fail++;
		ret = run_sha_too_large_test("rtk-mcp-sha256", MCP_TEST_BUF_SIZE + 1);
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
