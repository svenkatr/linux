/*
 * PowerNV setup code.
 *
 * Copyright 2011 IBM Corp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#undef DEBUG

#include <linux/cpu.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/reboot.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/interrupt.h>
#include <linux/bug.h>
#include <linux/pci.h>

#include <asm/machdep.h>
#include <asm/firmware.h>
#include <asm/xics.h>
#include <asm/rtas.h>
#include <asm/opal.h>
#include <asm/kexec.h>

#include "powernv.h"

static void __init pnv_setup_arch(void)
{
	/* Initialize SMP */
	pnv_smp_init();

	/* Setup PCI */
	pnv_pci_init();

	/* Setup RTC and NVRAM callbacks */
	if (firmware_has_feature(FW_FEATURE_OPAL))
		opal_nvram_init();

	/* Enable NAP mode */
	powersave_nap = 1;

	/* XXX PMCS */
}

static void __init pnv_init_early(void)
{
	/*
	 * Initialize the LPC bus now so that legacy serial
	 * ports can be found on it
	 */
	opal_lpc_init();

#ifdef CONFIG_HVC_OPAL
	if (firmware_has_feature(FW_FEATURE_OPAL))
		hvc_opal_init_early();
	else
#endif
		add_preferred_console("hvc", 0, NULL);
}

static void __init pnv_init_IRQ(void)
{
	xics_init();

	WARN_ON(!ppc_md.get_irq);
}

static void pnv_show_cpuinfo(struct seq_file *m)
{
	struct device_node *root;
	const char *model = "";

	root = of_find_node_by_path("/");
	if (root)
		model = of_get_property(root, "model", NULL);
	seq_printf(m, "machine\t\t: PowerNV %s\n", model);
	if (firmware_has_feature(FW_FEATURE_OPALv3))
		seq_printf(m, "firmware\t: OPAL v3\n");
	else if (firmware_has_feature(FW_FEATURE_OPALv2))
		seq_printf(m, "firmware\t: OPAL v2\n");
	else if (firmware_has_feature(FW_FEATURE_OPAL))
		seq_printf(m, "firmware\t: OPAL v1\n");
	else
		seq_printf(m, "firmware\t: BML\n");
	of_node_put(root);
}

static void  __noreturn pnv_restart(char *cmd)
{
	long rc = OPAL_BUSY;

	opal_notifier_disable();

	while (rc == OPAL_BUSY || rc == OPAL_BUSY_EVENT) {
		rc = opal_cec_reboot();
		if (rc == OPAL_BUSY_EVENT)
			opal_poll_events(NULL);
		else
			mdelay(10);
	}
	for (;;)
		opal_poll_events(NULL);
}

static void __noreturn pnv_power_off(void)
{
	long rc = OPAL_BUSY;

	opal_notifier_disable();

	while (rc == OPAL_BUSY || rc == OPAL_BUSY_EVENT) {
		rc = opal_cec_power_down(0);
		if (rc == OPAL_BUSY_EVENT)
			opal_poll_events(NULL);
		else
			mdelay(10);
	}
	for (;;)
		opal_poll_events(NULL);
}

static void __noreturn pnv_halt(void)
{
	pnv_power_off();
}

static void pnv_progress(char *s, unsigned short hex)
{
}

static int pnv_dma_set_mask(struct device *dev, u64 dma_mask)
{
	if (dev_is_pci(dev))
		return pnv_pci_dma_set_mask(to_pci_dev(dev), dma_mask);
	return __dma_set_mask(dev, dma_mask);
}

static void pnv_shutdown(void)
{
	/* Let the PCI code clear up IODA tables */
	pnv_pci_shutdown();

	/*
	 * Stop OPAL activity: Unregister all OPAL interrupts so they
	 * don't fire up while we kexec and make sure all potentially
	 * DMA'ing ops are complete (such as dump retrieval).
	 */
	opal_shutdown();
}

#ifdef CONFIG_KEXEC
static void pnv_kexec_wait_secondaries_down(void)
{
	int my_cpu, i, notified = -1;

	my_cpu = get_cpu();

	for_each_online_cpu(i) {
		uint8_t status;
		int64_t rc;

		if (i == my_cpu)
			continue;

		for (;;) {
			rc = opal_query_cpu_status(get_hard_smp_processor_id(i),
						   &status);
			if (rc != OPAL_SUCCESS || status != OPAL_THREAD_STARTED)
				break;
			barrier();
			if (i != notified) {
				printk(KERN_INFO "kexec: waiting for cpu %d "
				       "(physical %d) to enter OPAL\n",
				       i, paca[i].hw_cpu_id);
				notified = i;
			}
		}
	}
}

static void pnv_kexec_cpu_down(int crash_shutdown, int secondary)
{
	xics_kexec_teardown_cpu(secondary);

	/* On OPAL v3, we return all CPUs to firmware */

	if (!firmware_has_feature(FW_FEATURE_OPALv3))
		return;

	if (secondary) {
		/* Return secondary CPUs to firmware on OPAL v3 */
		mb();
		get_paca()->kexec_state = KEXEC_STATE_REAL_MODE;
		mb();

		/* Return the CPU to OPAL */
		opal_return_cpu();
	} else if (crash_shutdown) {
		/*
		 * On crash, we don't wait for secondaries to go
		 * down as they might be unreachable or hung, so
		 * instead we just wait a bit and move on.
		 */
		mdelay(1);
	} else {
		/* Primary waits for the secondaries to have reached OPAL */
		pnv_kexec_wait_secondaries_down();
	}
}
#endif /* CONFIG_KEXEC */

static void __init pnv_setup_machdep_opal(void)
{
	ppc_md.get_boot_time = opal_get_boot_time;
	ppc_md.get_rtc_time = opal_get_rtc_time;
	ppc_md.set_rtc_time = opal_set_rtc_time;
	ppc_md.restart = pnv_restart;
	ppc_md.power_off = pnv_power_off;
	ppc_md.halt = pnv_halt;
	ppc_md.machine_check_exception = opal_machine_check;
	ppc_md.mce_check_early_recovery = opal_mce_check_early_recovery;
}

#ifdef CONFIG_PPC_POWERNV_RTAS
static void __init pnv_setup_machdep_rtas(void)
{
	if (rtas_token("get-time-of-day") != RTAS_UNKNOWN_SERVICE) {
		ppc_md.get_boot_time = rtas_get_boot_time;
		ppc_md.get_rtc_time = rtas_get_rtc_time;
		ppc_md.set_rtc_time = rtas_set_rtc_time;
	}
	ppc_md.restart = rtas_restart;
	ppc_md.power_off = rtas_power_off;
	ppc_md.halt = rtas_halt;
}
#endif /* CONFIG_PPC_POWERNV_RTAS */

static int __init pnv_probe(void)
{
	unsigned long root = of_get_flat_dt_root();

	if (!of_flat_dt_is_compatible(root, "ibm,powernv"))
		return 0;

	hpte_init_native();

	if (firmware_has_feature(FW_FEATURE_OPAL))
		pnv_setup_machdep_opal();
#ifdef CONFIG_PPC_POWERNV_RTAS
	else if (rtas.base)
		pnv_setup_machdep_rtas();
#endif /* CONFIG_PPC_POWERNV_RTAS */

	pr_debug("PowerNV detected !\n");

	return 1;
}

define_machine(powernv) {
	.name			= "PowerNV",
	.probe			= pnv_probe,
	.init_early		= pnv_init_early,
	.setup_arch		= pnv_setup_arch,
	.init_IRQ		= pnv_init_IRQ,
	.show_cpuinfo		= pnv_show_cpuinfo,
	.progress		= pnv_progress,
	.machine_shutdown	= pnv_shutdown,
	.power_save             = power7_idle,
	.calibrate_decr		= generic_calibrate_decr,
	.dma_set_mask		= pnv_dma_set_mask,
#ifdef CONFIG_KEXEC
	.kexec_cpu_down		= pnv_kexec_cpu_down,
#endif
};
