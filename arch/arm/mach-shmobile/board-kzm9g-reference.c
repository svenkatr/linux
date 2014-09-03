/*
 * KZM-A9-GT board support - Reference Device Tree Implementation
 *
 * Copyright (C) 2012	Horms Solutions Ltd.
 *
 * Based on board-kzm9g.c
 * Copyright (C) 2012	Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/of_platform.h>
#include <mach/sh73a0.h>
#include <mach/common.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>

static void __init kzm_init(void)
{
	sh73a0_add_standard_devices_dt();

#ifdef CONFIG_CACHE_L2X0
	/* Early BRESP enable, Shared attribute override enable, 64K*8way */
	l2x0_init(IOMEM(0xf0100000), 0x40460000, 0x82000fff);
#endif
}

static const char *kzm9g_boards_compat_dt[] __initdata = {
	"renesas,kzm9g-reference",
	NULL,
};

DT_MACHINE_START(KZM9G_DT, "kzm9g-reference")
	.smp		= smp_ops(sh73a0_smp_ops),
	.map_io		= sh73a0_map_io,
	.init_early	= sh73a0_init_delay,
	.nr_irqs	= NR_IRQS_LEGACY,
	.init_machine	= kzm_init,
	.dt_compat	= kzm9g_boards_compat_dt,
MACHINE_END
