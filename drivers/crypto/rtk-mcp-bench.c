// SPDX-License-Identifier: GPL-2.0-only
/*
 * Benchmark: Realtek MCP hardware crypto vs ARM CPU software crypto.
 *
 * Tests AES-128-ECB, AES-128-CBC, SHA-1, SHA-256 at various data sizes.
 * For each algorithm, runs the MCP hardware driver then the generic
 * software fallback, printing throughput in MiB/s.
 *
 * Load with: insmod rtk-mcp-bench.ko
 * Results appear in dmesg.
 */

#include <crypto/aes.h>
#include <crypto/hash.h>
#include <crypto/skcipher.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/time.h>

#define BENCH_ITERATIONS	50
#define MAX_DATA_SIZE		(32 * 1024)

static const u8 bench_key[16] = {
	0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
	0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
};

static const u8 bench_iv[16] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
};

static u64 ktime_delta_us(ktime_t start, ktime_t end)
{
	return ktime_to_us(ktime_sub(end, start));
}

static void bench_skcipher(const char *driver, const char *label,
			   int data_size, int iterations)
{
	struct crypto_skcipher *tfm;
	struct skcipher_request *req;
	struct scatterlist sg_src, sg_dst;
	u8 *src, *dst, *iv;
	ktime_t t0, t1;
	u64 elapsed_us, throughput;
	int i, ret;

	tfm = crypto_alloc_skcipher(driver, 0, 0);
	if (IS_ERR(tfm)) {
		pr_err("bench: alloc %s failed: %ld\n", driver, PTR_ERR(tfm));
		return;
	}

	ret = crypto_skcipher_setkey(tfm, bench_key, sizeof(bench_key));
	if (ret) {
		pr_err("bench: setkey %s failed: %d\n", driver, ret);
		crypto_free_skcipher(tfm);
		return;
	}

	src = kmalloc(data_size, GFP_KERNEL);
	dst = kmalloc(data_size, GFP_KERNEL);
	iv = kmalloc(16, GFP_KERNEL);
	req = skcipher_request_alloc(tfm, GFP_KERNEL);
	if (!src || !dst || !iv || !req) {
		pr_err("bench: OOM for %s\n", driver);
		goto out;
	}

	get_random_bytes(src, data_size);

	sg_init_one(&sg_src, src, data_size);
	sg_init_one(&sg_dst, dst, data_size);

	t0 = ktime_get();
	for (i = 0; i < iterations; i++) {
		memcpy(iv, bench_iv, 16);
		skcipher_request_set_crypt(req, &sg_src, &sg_dst,
					   data_size, iv);
		ret = crypto_skcipher_encrypt(req);
		if (ret) {
			pr_err("bench: %s encrypt failed: %d (iter %d)\n",
			       driver, ret, i);
			goto out;
		}
	}
	t1 = ktime_get();

	elapsed_us = ktime_delta_us(t0, t1);
	if (elapsed_us == 0)
		elapsed_us = 1;

	/* throughput = (iterations * data_size) / elapsed_us  in MiB/s */
	throughput = (u64)iterations * data_size * 1000000ULL;
	do_div(throughput, elapsed_us);
	do_div(throughput, 1048576);

	pr_info("bench: %-28s %5d bytes x %4d = %4llu MiB/s (%llu us) [%s]\n",
		label, data_size, iterations,
		throughput, elapsed_us,
		crypto_skcipher_driver_name(tfm));

out:
	skcipher_request_free(req);
	kfree(iv);
	kfree(dst);
	kfree(src);
	crypto_free_skcipher(tfm);
}

static void bench_ahash(const char *driver, const char *label,
			 int data_size, int iterations)
{
	struct crypto_ahash *tfm;
	struct ahash_request *req;
	struct scatterlist sg;
	u8 *src, *digest;
	ktime_t t0, t1;
	u64 elapsed_us, throughput;
	unsigned int digest_size;
	int i, ret;

	tfm = crypto_alloc_ahash(driver, 0, 0);
	if (IS_ERR(tfm)) {
		pr_err("bench: alloc %s failed: %ld\n", driver, PTR_ERR(tfm));
		return;
	}

	digest_size = crypto_ahash_digestsize(tfm);
	src = kmalloc(data_size ? data_size : 1, GFP_KERNEL);
	digest = kzalloc(digest_size, GFP_KERNEL);
	req = ahash_request_alloc(tfm, GFP_KERNEL);
	if (!src || !digest || !req) {
		pr_err("bench: OOM for %s\n", driver);
		goto out;
	}

	if (data_size)
		get_random_bytes(src, data_size);

	sg_init_one(&sg, src, data_size ? data_size : 1);

	t0 = ktime_get();
	for (i = 0; i < iterations; i++) {
		ret = crypto_ahash_init(req);
		if (ret)
			goto fail;

		if (data_size) {
			ahash_request_set_crypt(req, &sg, NULL, data_size);
			ret = crypto_ahash_update(req);
			if (ret)
				goto fail;
		}

		ahash_request_set_crypt(req, NULL, digest, 0);
		ret = crypto_ahash_final(req);
		if (ret)
			goto fail;
	}
	t1 = ktime_get();

	elapsed_us = ktime_delta_us(t0, t1);
	if (elapsed_us == 0)
		elapsed_us = 1;

	throughput = (u64)iterations * data_size * 1000000ULL;
	do_div(throughput, elapsed_us);
	do_div(throughput, 1048576);

	pr_info("bench: %-28s %5d bytes x %4d = %4llu MiB/s (%llu us) [%s]\n",
		label, data_size, iterations,
		throughput, elapsed_us,
		crypto_ahash_driver_name(tfm));

	goto out;
fail:
	pr_err("bench: %s hash failed: %d (iter %d)\n", driver, ret, i);
out:
	ahash_request_free(req);
	kfree(digest);
	kfree(src);
	crypto_free_ahash(tfm);
}

/*
 * Sizes span small (setup-overhead-dominated) through the driver's 64KiB
 * internal bounce-buffer cap, so the comparison reflects the same chunking
 * the driver actually does for large skcipher requests.
 */
static const int sizes[] = { 256, 1024, 4096, 16384, 32768, 65536 };

/* Same range, minus 65536: the hash path caps at MCP_BUF_SIZE (64KiB) and
 * takes the whole message as one bounce-buffer copy, so nothing above that
 * is a meaningful comparison point for this driver.
 */
static const int hash_sizes[] = { 256, 1024, 4096, 16384, 32768 };

static int __init rtk_mcp_bench_init(void)
{
	int i;

	pr_info("bench: === Realtek MCP vs CPU benchmark ===\n");
	pr_info("bench: CPU reference drivers: aes-generic (unoptimized C),\n");
	pr_info("bench:   *-neonbs/-neon/-asm (best available ARM NEON/asm code)\n");

	for (i = 0; i < ARRAY_SIZE(sizes); i++) {
		bench_skcipher("rtk-mcp-ecb-aes", "ecb(aes) [MCP]",
			       sizes[i], BENCH_ITERATIONS);
		bench_skcipher("ecb-aes-neonbs", "ecb(aes) [CPU/neon-bitslice]",
			       sizes[i], BENCH_ITERATIONS);
		bench_skcipher("ecb(aes-generic)", "ecb(aes) [CPU/generic]",
			       sizes[i], BENCH_ITERATIONS);
	}

	pr_info("bench: ---\n");

	for (i = 0; i < ARRAY_SIZE(sizes); i++) {
		bench_skcipher("rtk-mcp-cbc-aes", "cbc(aes) [MCP]",
			       sizes[i], BENCH_ITERATIONS);
		bench_skcipher("cbc-aes-neonbs", "cbc(aes) [CPU/neon-bitslice]",
			       sizes[i], BENCH_ITERATIONS);
		bench_skcipher("cbc(aes-generic)", "cbc(aes) [CPU/generic]",
			       sizes[i], BENCH_ITERATIONS);
	}

	pr_info("bench: ---\n");

	for (i = 0; i < ARRAY_SIZE(hash_sizes); i++) {
		bench_ahash("rtk-mcp-sha1", "sha1 [MCP]",
			    hash_sizes[i], BENCH_ITERATIONS);
		bench_ahash("sha1-neon", "sha1 [CPU/neon]",
			    hash_sizes[i], BENCH_ITERATIONS);
		bench_ahash("sha1-asm", "sha1 [CPU/asm]",
			    hash_sizes[i], BENCH_ITERATIONS);
		bench_ahash("sha1-generic", "sha1 [CPU/generic]",
			    hash_sizes[i], BENCH_ITERATIONS);
	}

	pr_info("bench: ---\n");

	for (i = 0; i < ARRAY_SIZE(hash_sizes); i++) {
		bench_ahash("rtk-mcp-sha256", "sha256 [MCP]",
			    hash_sizes[i], BENCH_ITERATIONS);
		bench_ahash("sha256-neon", "sha256 [CPU/neon]",
			    hash_sizes[i], BENCH_ITERATIONS);
		bench_ahash("sha256-asm", "sha256 [CPU/asm]",
			    hash_sizes[i], BENCH_ITERATIONS);
		bench_ahash("sha256-generic", "sha256 [CPU/generic]",
			    hash_sizes[i], BENCH_ITERATIONS);
	}

	pr_info("bench: === done ===\n");
	return 0;
}

static void __exit rtk_mcp_bench_exit(void)
{
}

module_init(rtk_mcp_bench_init);
module_exit(rtk_mcp_bench_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Realtek MCP vs CPU crypto benchmark");
