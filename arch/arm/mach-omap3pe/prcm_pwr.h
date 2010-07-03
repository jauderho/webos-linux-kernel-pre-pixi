/*
 * Copyright (C) 2008 Palm, Inc.
 * Author: Wolfgang Reissnegger
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *
 */

#ifndef _PRCM_PWR_H_
#define _PRCM_PWR_H_

#include <linux/types.h>

/******************************************************************************/

struct system_power_state {
#ifndef CONFIG_ARCH_OMAP3410
	u8 gfx_state;
	u8 neon_state;
#endif
	u32 mpu_state;
	u32 core_state;
	u8 iva2_state;
	u8 dss_state;
	u8 cam_state;
	u8 sgx_state;
	u8 per_state;
	u8 usbhost_state;
};

/******************************************************************************/

int prcm_get_power_domain_state(u32 domainid, u8 *result);
int prcm_set_power_domain_state(u32 domainid, u8 new_state, u8 mode);
int prcm_get_pre_power_domain_state(u32 domainid, u8 *result);
int prcm_force_power_domain_state(u32 domain, u8 state);
int prcm_get_devices_not_idle(u32 domainid, u32 *result);
int prcm_get_initiators_not_standby(u32 domainid, u32 *result);
int prcm_set_domain_power_configuration(u32 domainid, u8 idlemode,
					u8 standbymode, u8 autoidleenable);
void clear_prepwstst(void);
int prcm_transition_domain_to(u32 domid, u8 state);
int prcm_set_chip_power_mode(struct system_power_state *target_state);

#endif
