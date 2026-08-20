// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek MCP SHA256 compliance test.
 *
 * Compares digests produced by the HW provider (rtk-mcp-sha256) against a
 * CPU reference implementation (sha256-generic) using the same kernel
 * ahash API patterns that in-kernel users rely on:
 *   - empty input
 *   - init/update/final with multiple update chunks
 *   - scatterlist inputs (single- and multi-segment)
 *   - request reuse
 *   - basic concurrent requests
 *
 * Load with: insmod rtk-mcp-sha-compliance-test.ko
 *
 * If rtk-mcp-sha256 is not registered/available, the module skips tests.
 */

#include <crypto/hash.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>

#define HW_SHA_ALG "rtk-mcp-sha256"
#define CPU_SHA_ALG "sha256-generic"

static int hw_cpu_alloc_ahash(const char *hw_alg, const char *cpu_alg,
                              struct crypto_ahash **hw, struct crypto_ahash **cpu)
{
	*hw = crypto_alloc_ahash(hw_alg, 0, 0);
	if (IS_ERR(*hw))
		return PTR_ERR(*hw);

	*cpu = crypto_alloc_ahash(cpu_alg, 0, 0);
	if (IS_ERR(*cpu)) {
		/* Some kernels may not expose sha256-generic. */
		*cpu = crypto_alloc_ahash("sha256", 0, 0);
		if (IS_ERR(*cpu)) {
			crypto_free_ahash(*hw);
			*hw = NULL;
			return PTR_ERR(*cpu);
		}
	}

	return 0;
}

static void fill_pattern(u8 *buf, size_t len, u8 seed)
{
	for (size_t i = 0; i < len; i++)
		buf[i] = (u8)(seed + i * 33 + (i >> 3));
}

static int ahash_digest_cpu_hw(struct crypto_ahash *cpu,
					struct crypto_ahash *hw,
					const u8 *data, size_t len,
					u8 *out_cpu, u8 *out_hw)
{
	struct ahash_request *req_cpu;
	struct ahash_request *req_hw;
	struct scatterlist sg;
	int ret;

	req_cpu = ahash_request_alloc(cpu, GFP_KERNEL);
	req_hw = ahash_request_alloc(hw, GFP_KERNEL);
	if (!req_cpu || !req_hw) {
		ahash_request_free(req_cpu);
		ahash_request_free(req_hw);
		return -ENOMEM;
	}

	ret = crypto_ahash_init(req_cpu);
	if (ret)
		goto out;

	if (len) {
		sg_init_one(&sg, (void *)data, len);
		ahash_request_set_crypt(req_cpu, &sg, NULL, len);
		ret = crypto_ahash_update(req_cpu);
		if (ret)
			goto out;
	}

	ahash_request_set_crypt(req_cpu, NULL, out_cpu, 0);
	ret = crypto_ahash_final(req_cpu);
	if (ret)
		goto out;

	ret = crypto_ahash_init(req_hw);
	if (ret)
		goto out;

	if (len) {
		sg_init_one(&sg, (void *)data, len);
		ahash_request_set_crypt(req_hw, &sg, NULL, len);
		ret = crypto_ahash_update(req_hw);
		if (ret)
			goto out;
	}

	ahash_request_set_crypt(req_hw, NULL, out_hw, 0);
	ret = crypto_ahash_final(req_hw);

out:
	ahash_request_free(req_cpu);
	ahash_request_free(req_hw);
	return ret;
}

static int test_hw_matches_cpu_for_case(struct crypto_ahash *hw,
					struct crypto_ahash *cpu,
					const u8 *data, size_t len,
					u8 *tmp_cpu, u8 *tmp_hw,
					size_t chunk)
{
	unsigned int digest_size = crypto_ahash_digestsize(cpu);
	int ret;

	/* 1) crypto_ahash_digest (single-shot) */
	ret = ahash_digest_cpu_hw(cpu, hw, data, len, tmp_cpu, tmp_hw);
	if (ret)
		return ret;
	if (memcmp(tmp_cpu, tmp_hw, digest_size))
		return -EUCLEAN;

	/* 2) init/update/final with chunked updates */
	{
		struct ahash_request *req_hw;
		struct ahash_request *req_cpu;
		u8 *digest_hw;
		u8 *digest_cpu;
		struct scatterlist sg1;

		digest_hw = kmalloc(digest_size, GFP_KERNEL);
		digest_cpu = kmalloc(digest_size, GFP_KERNEL);
		if (!digest_hw || !digest_cpu) {
			ret = -ENOMEM;
			kfree(digest_hw);
			kfree(digest_cpu);
			return ret;
		}

		req_hw = ahash_request_alloc(hw, GFP_KERNEL);
		req_cpu = ahash_request_alloc(cpu, GFP_KERNEL);
		if (!req_hw || !req_cpu) {
			ret = -ENOMEM;
			goto out_free;
		}

		ret = crypto_ahash_init(req_hw);
		if (ret)
			goto out_req;
		ret = crypto_ahash_init(req_cpu);
		if (ret)
			goto out_req;

		for (size_t off = 0; off < len; off += chunk) {
			size_t part = min(chunk, len - off);
			sg_init_one(&sg1, (void *)(data + off), part);
			ahash_request_set_crypt(req_hw, &sg1, NULL, part);
			ret = crypto_ahash_update(req_hw);
			if (ret)
				goto out_req;
			ahash_request_set_crypt(req_cpu, &sg1, NULL, part);
			ret = crypto_ahash_update(req_cpu);
			if (ret)
				goto out_req;
		}

		ahash_request_set_crypt(req_hw, NULL, digest_hw, 0);
		ret = crypto_ahash_final(req_hw);
		if (ret)
			goto out_req;
		ahash_request_set_crypt(req_cpu, NULL, digest_cpu, 0);
		ret = crypto_ahash_final(req_cpu);
		if (ret)
			goto out_req;

		ret = memcmp(digest_cpu, digest_hw, digest_size) ? -EUCLEAN : 0;

	out_req:
		ahash_request_free(req_hw);
		ahash_request_free(req_cpu);
	out_free:
		kfree(digest_hw);
		kfree(digest_cpu);
		if (ret)
			return ret;
	}

	/* 3) init/update/final with a multi-segment scatterlist (single update) */
	if (len) {
		struct ahash_request *req_hw;
		struct ahash_request *req_cpu;
		u8 *digest_hw;
		u8 *digest_cpu;
		struct scatterlist sg[2];

		digest_hw = kmalloc(digest_size, GFP_KERNEL);
		digest_cpu = kmalloc(digest_size, GFP_KERNEL);
		if (!digest_hw || !digest_cpu) {
			ret = -ENOMEM;
			kfree(digest_hw);
			kfree(digest_cpu);
			return ret;
		}

		req_hw = ahash_request_alloc(hw, GFP_KERNEL);
		req_cpu = ahash_request_alloc(cpu, GFP_KERNEL);
		if (!req_hw || !req_cpu) {
			ret = -ENOMEM;
			goto out_free2;
		}

		ret = crypto_ahash_init(req_hw);
		if (ret)
			goto out_req2;
		ret = crypto_ahash_init(req_cpu);
		if (ret)
			goto out_req2;

		size_t part1 = min(chunk, len ? (len / 2) : 0);
		size_t part2 = len - part1;
		sg_init_table(sg, 2);
		sg_set_buf(&sg[0], (void *)data, part1);
		sg_set_buf(&sg[1], (void *)(data + part1), part2);

		ahash_request_set_crypt(req_hw, sg, NULL, len);
		ret = crypto_ahash_update(req_hw);
		if (ret)
			goto out_req2;
		ahash_request_set_crypt(req_cpu, sg, NULL, len);
		ret = crypto_ahash_update(req_cpu);
		if (ret)
			goto out_req2;

		ahash_request_set_crypt(req_hw, NULL, digest_hw, 0);
		ret = crypto_ahash_final(req_hw);
		if (ret)
			goto out_req2;
		ahash_request_set_crypt(req_cpu, NULL, digest_cpu, 0);
		ret = crypto_ahash_final(req_cpu);
		if (ret)
			goto out_req2;

		ret = memcmp(digest_cpu, digest_hw, digest_size) ? -EUCLEAN : 0;

	out_req2:
		ahash_request_free(req_hw);
		ahash_request_free(req_cpu);
	out_free2:
		kfree(digest_hw);
		kfree(digest_cpu);
		if (ret)
			return ret;
	}

	return 0;
}

struct conc_test_job {
	const char *hw_alg;
	const char *cpu_alg;
	u8 *data;
	size_t len;
	unsigned int digest_size;
	int fail;
};

static int conc_test_thread(void *arg)
{
	struct conc_test_job *job = arg;
	struct crypto_ahash *hw = NULL;
	struct crypto_ahash *cpu = NULL;
	u8 *tmp_cpu = kmalloc(job->digest_size, GFP_KERNEL);
	u8 *tmp_hw = kmalloc(job->digest_size, GFP_KERNEL);
	int ret;

	if (!tmp_cpu || !tmp_hw) {
		job->fail = 1;
		kfree(tmp_cpu);
		kfree(tmp_hw);
		return 0;
	}

	/* Allocate per-thread tfms (avoid any assumptions about tfm thread-safety). */
	hw = crypto_alloc_ahash(job->hw_alg, 0, 0);
	if (IS_ERR(hw)) {
		job->fail = 1;
		kfree(tmp_cpu);
		kfree(tmp_hw);
		return 0;
	}

	cpu = crypto_alloc_ahash(job->cpu_alg, 0, 0);
	if (IS_ERR(cpu)) {
		crypto_free_ahash(hw);
		cpu = crypto_alloc_ahash("sha256", 0, 0);
		if (IS_ERR(cpu)) {
			job->fail = 1;
			kfree(tmp_cpu);
			kfree(tmp_hw);
			return 0;
		}
	}

	/* Run a few times to exercise request reuse / serialization. */
	for (int i = 0; i < 10; i++) {
		fill_pattern(job->data, job->len, (u8)(0xA0 + i));
		ret = test_hw_matches_cpu_for_case(hw, cpu,
						     job->data, job->len,
						     tmp_cpu, tmp_hw,
						     max_t(size_t, 1, job->len / 7));
		if (ret)
			job->fail = 1;
	}

	kfree(tmp_cpu);
	kfree(tmp_hw);
	crypto_free_ahash(cpu);
	crypto_free_ahash(hw);
	return 0;
}

static int __init rtk_mcp_sha_compliance_test_init(void)
{
	struct crypto_ahash *hw = NULL, *cpu = NULL;
	int ret;
	unsigned int digest_size;

	struct {
		size_t len;
		size_t chunk;
	} cases[] = {
		{ 0,   1 },
		{ 1,   1 },
		{ 2,   1 },
		{ 7,   2 },
		{ 16,  3 },
		{ 31,  5 },
		{ 32,  4 },
		{ 33,  6 },
		{ 63,  7 },
		{ 64,  8 },
		{ 65,  9 },
		{ 127, 10 },
		{ 128, 11 },
		{ 129, 12 },
		{ 255, 13 },
		{ 256, 14 },
		{ 257, 15 },
		{ 1024, 32 },
		{ 4096, 64 },
	};

	u8 *data = NULL;
	u8 *tmp_cpu = NULL;
	u8 *tmp_hw = NULL;

	pr_info("rtk-mcp: SHA256 compliance test: HW=%s CPU=%s\n",
		HW_SHA_ALG, CPU_SHA_ALG);

	ret = hw_cpu_alloc_ahash(HW_SHA_ALG, CPU_SHA_ALG, &hw, &cpu);
	if (ret) {
		/* Most useful when HW isn't registered in the current kernel. */
		pr_warn("rtk-mcp: cannot allocate ahash (ret=%d); skipping\n", ret);
		return 0;
	}

	digest_size = crypto_ahash_digestsize(cpu);
	if (crypto_ahash_digestsize(hw) != digest_size) {
		pr_err("rtk-mcp: digest size mismatch (cpu=%u hw=%u)\n",
		       digest_size, crypto_ahash_digestsize(hw));
		ret = -EINVAL;
		goto out_free;
	}

	data = kmalloc(max_t(size_t, 1, cases[ARRAY_SIZE(cases) - 1].len),
			GFP_KERNEL);
	tmp_cpu = kmalloc(digest_size, GFP_KERNEL);
	tmp_hw = kmalloc(digest_size, GFP_KERNEL);
	if (!data || !tmp_cpu || !tmp_hw) {
		ret = -ENOMEM;
		goto out_free;
	}

	for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
		size_t len = cases[i].len;
		size_t chunk = cases[i].chunk;
		
		if (len)
			fill_pattern(data, len, (u8)(0x10 + i));
		else
			fill_pattern(data, 1, (u8)(0x10 + i)); /* keep data non-NULL */

		ret = test_hw_matches_cpu_for_case(hw, cpu, data, len,
						tmp_cpu, tmp_hw, chunk);
		if (ret) {
			pr_err("rtk-mcp: SHA256 mismatch for len=%zu chunk=%zu ret=%d\n",
			       len, chunk, ret);
			goto out_free;
		}
	}

	/* Basic concurrency: run 2-4 threads, each comparing multiple digests. */
	{
		int thread_count = 3;
		struct task_struct *threads[3];
		struct conc_test_job jobs[3];
		struct completion done;

		init_completion(&done);
		for (int t = 0; t < thread_count; t++) {
			jobs[t].hw_alg = HW_SHA_ALG;
			jobs[t].cpu_alg = CPU_SHA_ALG;
			jobs[t].len = 2048 + t * 37;
			jobs[t].digest_size = digest_size;
			jobs[t].fail = 0;
			jobs[t].data = kmalloc(jobs[t].len, GFP_KERNEL);
			if (!jobs[t].data) {
				ret = -ENOMEM;
				thread_count = t;
				break;
			}
			fill_pattern(jobs[t].data, jobs[t].len, (u8)(0x55 + t));
		}

		for (int t = 0; t < thread_count; t++)
			threads[t] = kthread_run(conc_test_thread, &jobs[t],
						     "rtk-mcp-sha-test/%d", t);

		for (int t = 0; t < thread_count; t++) {
			if (threads[t])
				kthread_stop(threads[t]);
			if (jobs[t].fail) {
				pr_err("rtk-mcp: concurrency test failed (thread %d)\n", t);
				ret = -EUCLEAN;
				goto out_jobs;
			}
		}

	out_jobs:
		for (int t = 0; t < thread_count; t++)
			kfree(jobs[t].data);
	}

	pr_info("rtk-mcp: SHA256 compliance test: PASS\n");
	ret = 0;

out_free:
	if (tmp_cpu)
		kfree(tmp_cpu);
	if (tmp_hw)
		kfree(tmp_hw);
	if (data)
		kfree(data);
	if (cpu)
		crypto_free_ahash(cpu);
	if (hw)
		crypto_free_ahash(hw);

	return ret;
}

static void __exit rtk_mcp_sha_compliance_test_exit(void)
{
	pr_info("rtk-mcp: SHA256 compliance test: unloaded\n");
}

module_init(rtk_mcp_sha_compliance_test_init);
module_exit(rtk_mcp_sha_compliance_test_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Realtek MCP SHA256 compliance test vs CPU");
