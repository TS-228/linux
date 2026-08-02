// SPDX-License-Identifier: GPL-2.0-only

#include <linux/mm.h>
#include <linux/io.h>

#ifdef CONFIG_RTK_TRACER
#include <linux/rtk_trace.h>
#endif

bool ioremap_allowed(phys_addr_t phys_addr, size_t size, unsigned long prot)
{
	unsigned long last_addr = phys_addr + size - 1;

	/* Don't allow outside PHYS_MASK */
	if (last_addr & ~PHYS_MASK)
		return false;

	/* Don't allow RAM to be mapped. */
#ifndef CONFIG_RTK_MEM_REMAP
	if (WARN_ON(pfn_is_map_memory(__phys_to_pfn(phys_addr))))
		return false;
#endif

	return true;
}

#ifdef CONFIG_RTK_TRACER
bool iounmap_allowed(void *addr)
{
	remove_vmap_info(addr, 1);
	return true;
}
#endif

/*
 * Must be called after early_fixmap_init
 */
void __init early_ioremap_init(void)
{
	early_ioremap_setup();
}

bool arch_memremap_can_ram_remap(resource_size_t offset, size_t size,
				 unsigned long flags)
{
	unsigned long pfn = PHYS_PFN(offset);

	return pfn_is_map_memory(pfn);
}
