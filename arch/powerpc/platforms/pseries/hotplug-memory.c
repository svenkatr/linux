/*
 * pseries Memory Hotplug infrastructure.
 *
 * Copyright (C) 2008 Badari Pulavarty, IBM Corporation
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 */

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/memblock.h>
#include <linux/vmalloc.h>
#include <linux/memory.h>
#include <linux/memory_hotplug.h>

#include <asm/firmware.h>
#include <asm/machdep.h>
#include <asm/prom.h>
#include <asm/sparsemem.h>

static unsigned long get_memblock_size(void)
{
	struct device_node *np;
	unsigned int memblock_size = MIN_MEMORY_BLOCK_SIZE;
	struct resource r;

	np = of_find_node_by_path("/ibm,dynamic-reconfiguration-memory");
	if (np) {
		const __be64 *size;

		size = of_get_property(np, "ibm,lmb-size", NULL);
		if (size)
			memblock_size = be64_to_cpup(size);
		of_node_put(np);
	} else  if (machine_is(pseries)) {
		/* This fallback really only applies to pseries */
		unsigned int memzero_size = 0;

		np = of_find_node_by_path("/memory@0");
		if (np) {
			if (!of_address_to_resource(np, 0, &r))
				memzero_size = resource_size(&r);
			of_node_put(np);
		}

		if (memzero_size) {
			/* We now know the size of memory@0, use this to find
			 * the first memoryblock and get its size.
			 */
			char buf[64];

			sprintf(buf, "/memory@%x", memzero_size);
			np = of_find_node_by_path(buf);
			if (np) {
				if (!of_address_to_resource(np, 0, &r))
					memblock_size = resource_size(&r);
				of_node_put(np);
			}
		}
	}
	return memblock_size;
}

/* WARNING: This is going to override the generic definition whenever
 * pseries is built-in regardless of what platform is active at boot
 * time. This is fine for now as this is the only "option" and it
 * should work everywhere. If not, we'll have to turn this into a
 * ppc_md. callback
 */
unsigned long memory_block_size_bytes(void)
{
	return get_memblock_size();
}

#ifdef CONFIG_MEMORY_HOTREMOVE
static int pseries_remove_memory(u64 start, u64 size)
{
	int ret;

	/* Remove htab bolted mappings for this section of memory */
	start = (unsigned long)__va(start);
	ret = remove_section_mapping(start, start + size);

	/* Ensure all vmalloc mappings are flushed in case they also
	 * hit that section of memory
	 */
	vm_unmap_aliases();

	return ret;
}

static int pseries_remove_memblock(unsigned long base, unsigned int memblock_size)
{
	unsigned long block_sz, start_pfn;
	int sections_per_block;
	int i, nid;

	start_pfn = base >> PAGE_SHIFT;

	lock_device_hotplug();

	if (!pfn_valid(start_pfn))
		goto out;

	block_sz = memory_block_size_bytes();
	sections_per_block = block_sz / MIN_MEMORY_BLOCK_SIZE;
	nid = memory_add_physaddr_to_nid(base);

	for (i = 0; i < sections_per_block; i++) {
		remove_memory(nid, base, MIN_MEMORY_BLOCK_SIZE);
		base += MIN_MEMORY_BLOCK_SIZE;
	}

out:
	/* Update memory regions for memory remove */
	memblock_remove(base, memblock_size);
	unlock_device_hotplug();
	return 0;
}

static int pseries_remove_mem_node(struct device_node *np)
{
	const char *type;
	const unsigned int *regs;
	unsigned long base;
	unsigned int lmb_size;
	int ret = -EINVAL;

	/*
	 * Check to see if we are actually removing memory
	 */
	type = of_get_property(np, "device_type", NULL);
	if (type == NULL || strcmp(type, "memory") != 0)
		return 0;

	/*
	 * Find the bae address and size of the memblock
	 */
	regs = of_get_property(np, "reg", NULL);
	if (!regs)
		return ret;

	base = *(unsigned long *)regs;
	lmb_size = regs[3];

	pseries_remove_memblock(base, lmb_size);
	return 0;
}
#else
static inline int pseries_remove_memblock(unsigned long base,
					  unsigned int memblock_size)
{
	return -EOPNOTSUPP;
}
static inline int pseries_remove_mem_node(struct device_node *np)
{
	return -EOPNOTSUPP;
}
#endif /* CONFIG_MEMORY_HOTREMOVE */

static int pseries_add_mem_node(struct device_node *np)
{
	const char *type;
	const unsigned int *regs;
	unsigned long base;
	unsigned int lmb_size;
	int ret = -EINVAL;

	/*
	 * Check to see if we are actually adding memory
	 */
	type = of_get_property(np, "device_type", NULL);
	if (type == NULL || strcmp(type, "memory") != 0)
		return 0;

	/*
	 * Find the base and size of the memblock
	 */
	regs = of_get_property(np, "reg", NULL);
	if (!regs)
		return ret;

	base = *(unsigned long *)regs;
	lmb_size = regs[3];

	/*
	 * Update memory region to represent the memory add
	 */
	ret = memblock_add(base, lmb_size);
	return (ret < 0) ? -EINVAL : 0;
}

static int pseries_update_drconf_memory(struct of_prop_reconfig *pr)
{
	struct of_drconf_cell *new_drmem, *old_drmem;
	unsigned long memblock_size;
	u32 entries;
	u32 *p;
	int i, rc = -EINVAL;

	memblock_size = get_memblock_size();
	if (!memblock_size)
		return -EINVAL;

	p = (u32 *)of_get_property(pr->dn, "ibm,dynamic-memory", NULL);
	if (!p)
		return -EINVAL;

	/* The first int of the property is the number of lmb's described
	 * by the property. This is followed by an array of of_drconf_cell
	 * entries. Get the niumber of entries and skip to the array of
	 * of_drconf_cell's.
	 */
	entries = *p++;
	old_drmem = (struct of_drconf_cell *)p;

	p = (u32 *)pr->prop->value;
	p++;
	new_drmem = (struct of_drconf_cell *)p;

	for (i = 0; i < entries; i++) {
		if ((old_drmem[i].flags & DRCONF_MEM_ASSIGNED) &&
		    (!(new_drmem[i].flags & DRCONF_MEM_ASSIGNED))) {
			rc = pseries_remove_memblock(old_drmem[i].base_addr,
						     memblock_size);
			break;
		} else if ((!(old_drmem[i].flags & DRCONF_MEM_ASSIGNED)) &&
			   (new_drmem[i].flags & DRCONF_MEM_ASSIGNED)) {
			rc = memblock_add(old_drmem[i].base_addr,
					  memblock_size);
			rc = (rc < 0) ? -EINVAL : 0;
			break;
		}
	}

	return rc;
}

static int pseries_memory_notifier(struct notifier_block *nb,
				   unsigned long action, void *node)
{
	struct of_prop_reconfig *pr;
	int err = 0;

	switch (action) {
	case OF_RECONFIG_ATTACH_NODE:
		err = pseries_add_mem_node(node);
		break;
	case OF_RECONFIG_DETACH_NODE:
		err = pseries_remove_mem_node(node);
		break;
	case OF_RECONFIG_UPDATE_PROPERTY:
		pr = (struct of_prop_reconfig *)node;
		if (!strcmp(pr->prop->name, "ibm,dynamic-memory"))
			err = pseries_update_drconf_memory(pr);
		break;
	}
	return notifier_from_errno(err);
}

static struct notifier_block pseries_mem_nb = {
	.notifier_call = pseries_memory_notifier,
};

static int __init pseries_memory_hotplug_init(void)
{
	if (firmware_has_feature(FW_FEATURE_LPAR))
		of_reconfig_notifier_register(&pseries_mem_nb);

#ifdef CONFIG_MEMORY_HOTREMOVE
	ppc_md.remove_memory = pseries_remove_memory;
#endif

	return 0;
}
machine_device_initcall(pseries, pseries_memory_hotplug_init);
