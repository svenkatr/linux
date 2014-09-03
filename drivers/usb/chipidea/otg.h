/*
 * Copyright (C) 2013-2014 Freescale Semiconductor, Inc.
 *
 * Author: Peter Chen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DRIVERS_USB_CHIPIDEA_OTG_H
#define __DRIVERS_USB_CHIPIDEA_OTG_H

static inline void ci_clear_otg_interrupt(struct ci_hdrc *ci, u32 bits)
{
	/* Only clear request bits */
	hw_write(ci, OP_OTGSC, OTGSC_INT_STATUS_BITS, bits);
}

static inline void ci_enable_otg_interrupt(struct ci_hdrc *ci, u32 bits)
{
	hw_write(ci, OP_OTGSC, bits | OTGSC_INT_STATUS_BITS, bits);
}

static inline void ci_disable_otg_interrupt(struct ci_hdrc *ci, u32 bits)
{
	hw_write(ci, OP_OTGSC, bits | OTGSC_INT_STATUS_BITS, 0);
}

int ci_hdrc_otg_init(struct ci_hdrc *ci);
void ci_hdrc_otg_destroy(struct ci_hdrc *ci);
enum ci_role ci_otg_role(struct ci_hdrc *ci);
void ci_handle_vbus_change(struct ci_hdrc *ci);

#endif /* __DRIVERS_USB_CHIPIDEA_OTG_H */
