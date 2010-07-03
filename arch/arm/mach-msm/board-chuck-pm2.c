/* arch/arm/mach-msm/pm2.c
 *
 * MSM Power Management Routines
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/pm.h>
//#include <linux/pm_qos_params.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/uaccess.h>
#include <linux/fastpath.h>
#include <asm/arch/msm_iomap.h>
#include <asm/arch/system.h>
#include <asm/io.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif
#include <linux/wake_sources.h>

#include "smd_private.h"
#include "acpuclock.h"
#include "clock.h"
#include "proc_comm.h"
#include "idle.h"
#include "irq.h"
#include "gpio.h"
#include "timer.h"
#include "pm.h"

/******************************************************************************
 * Debug Definitions
 *****************************************************************************/

enum {
	MSM_PM_DEBUG_SUSPEND = 1U << 0,
	MSM_PM_DEBUG_POWER_COLLAPSE = 1U << 1,
	MSM_PM_DEBUG_STATE = 1U << 2,
	MSM_PM_DEBUG_CLOCK = 1U << 3,
	MSM_PM_DEBUG_RESET_VECTOR = 1U << 4,
	MSM_PM_DEBUG_SMSM_STATE = 1U << 5,
	MSM_PM_DEBUG_IDLE = 1U << 6,
	MSM_PM_DEBUG_SWFI = 1U << 7,
};

static int msm_pm_debug_mask = 0;
module_param_named(
	debug_mask, msm_pm_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);

#define MSM_PM_DPRINTK(mask, level, message, ...) \
	do { \
		if ((mask) & msm_pm_debug_mask) \
			printk(level message, ## __VA_ARGS__); \
	} while (0)

#define MSM_PM_DEBUG_PRINT_STATE(tag) \
	do { \
		MSM_PM_DPRINTK(MSM_PM_DEBUG_STATE, \
			KERN_INFO, "%s: " \
			"APPS_CLK_SLEEP_EN %x, APPS_PWRDOWN %x, " \
			"SMSM_POWER_MASTER_DEM %x, SMSM_MODEM_STATE %x, " \
			"SMSM_APPS_DEM %x\n", \
			tag, \
			readl(APPS_CLK_SLEEP_EN), readl(APPS_PWRDOWN), \
			smsm_get_state(SMSM_POWER_MASTER_DEM), \
			smsm_get_state(SMSM_MODEM_STATE), \
			smsm_get_state(SMSM_APPS_DEM)); \
	} while (0)

#define MSM_PM_DEBUG_PRINT_SLEEP_INFO() \
	do { \
		if (msm_pm_debug_mask & MSM_PM_DEBUG_SMSM_STATE) \
			smsm_print_sleep_info(msm_pm_smem_data->sleep_time, \
				msm_pm_smem_data->resources_used, \
				msm_pm_smem_data->irq_mask, \
				msm_pm_smem_data->wakeup_reason, \
				msm_pm_smem_data->pending_irqs); \
	} while (0)


#ifdef CONFIG_MSM_POWER_DEBUG
extern void get_enabled_clocks(char *msg);
extern void get_enabled_vregs(char *msg);
#endif

extern void chuck_mux_sleep_config(void);
extern void chuck_mux_wake_config(void);

/******************************************************************************
 * Sleep Modes and Parameters
 *****************************************************************************/

static int msm_pm_sleep_mode = CONFIG_MSM7X00A_SLEEP_MODE;
module_param_named(
	sleep_mode, msm_pm_sleep_mode,
	int, S_IRUGO | S_IWUSR | S_IWGRP
);

static int msm_pm_idle_sleep_mode = CONFIG_MSM7X00A_IDLE_SLEEP_MODE;
module_param_named(
	idle_sleep_mode, msm_pm_idle_sleep_mode,
	int, S_IRUGO | S_IWUSR | S_IWGRP
);

static int msm_pm_idle_sleep_min_time = CONFIG_MSM7X00A_IDLE_SLEEP_MIN_TIME;
module_param_named(
	idle_sleep_min_time, msm_pm_idle_sleep_min_time,
	int, S_IRUGO | S_IWUSR | S_IWGRP
);

#define MSM_PM_MODE_ATTR_SUSPEND_ENABLED "suspend_enabled"
#define MSM_PM_MODE_ATTR_IDLE_ENABLED "idle_enabled"
#define MSM_PM_MODE_ATTR_LATENCY "latency"
#define MSM_PM_MODE_ATTR_RESIDENCY "residency"
#define MSM_PM_MODE_ATTR_NR (4)

static char *msm_pm_sleep_mode_labels[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND] = " ",
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = "power_collapse",
	[MSM_PM_SLEEP_MODE_APPS_SLEEP] = "apps_sleep",
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] =
		"ramp_down_and_wfi",
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = "wfi",
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] =
		"power_collapse_no_xo_shutdown",
};

static struct msm_pm_platform_data *msm_pm_modes;

void __init msm_pm_set_platform_data(struct msm_pm_platform_data *data)
{
	msm_pm_modes = data;
}


/******************************************************************************
 * Sleep Limitations
 *****************************************************************************/
enum {
	SLEEP_LIMIT_NONE = 0,
	SLEEP_LIMIT_NO_TCXO_SHUTDOWN = 2
};


/******************************************************************************
 * Configure Hardware for Power Down/Up
 *****************************************************************************/

#define APPS_CLK_SLEEP_EN (MSM_CSR_BASE + 0x11c)
#define APPS_PWRDOWN      (MSM_CSR_BASE + 0x440)
#define APPS_STANDBY_CTL  (MSM_CSR_BASE + 0x108)

/*
 * Configure hardware registers in preparation for Apps power down.
 */
static void msm_pm_config_hw_before_power_down(void)
{
	writel(0x1f, APPS_CLK_SLEEP_EN);
	writel(1, APPS_PWRDOWN);
	writel(0, APPS_STANDBY_CTL);
}

/*
 * Clear hardware registers after Apps powers up.
 */
static void msm_pm_config_hw_after_power_up(void)
{
	writel(0, APPS_PWRDOWN);
	writel(0, APPS_CLK_SLEEP_EN);
}

/*
 * Configure hardware registers in preparation for SWFI.
 */
static void msm_pm_config_hw_before_swfi(void)
{
#ifdef CONFIG_ARCH_MSM_SCORPION
	writel(0x1f, APPS_CLK_SLEEP_EN);
#else
	writel(0x0f, APPS_CLK_SLEEP_EN);
#endif
}


/******************************************************************************
 * Restart Definitions
 *****************************************************************************/

#define RESTART_REASON_SHUTDOWN 0x6f656d00
#define RESTART_REASON_RECOVER  0x6f656d11
#define RESTART_REASON_PANIC    0x6f656d22
#define RESTART_REASON_DFU      0x6f656d33
#define RESTART_REASON_REBOOT   0x6f656d44
#define RESTART_REASON_UNKNOWN  0x77665501

static uint32_t restart_reason = 0x776655AA;


/*
 * Respond to timing out waiting for Modem
 *
 * NOTE: The function never returns.
 */
void msm_pm_timeout(void)
{
#if defined(CONFIG_MSM_PM_TIMEOUT_RESET_CHIP)
	printk(KERN_EMERG "%s(): resetting chip\n", __func__);
	restart_reason = RESTART_REASON_PANIC;
	msm_proc_comm(PCOM_RESET_CHIP, &restart_reason, 0);
#elif defined(CONFIG_MSM_PM_TIMEOUT_RESET_MODEM)
	printk(KERN_EMERG "%s(): resetting modem\n", __func__);
	msm_proc_comm_reset_modem_now();
#elif defined(CONFIG_MSM_PM_TIMEOUT_HALT)
	printk(KERN_EMERG "%s(): halting\n", __func__);
#endif
	for (;;)
		;
}


/******************************************************************************
 * State Polling Definitions
 *****************************************************************************/

struct msm_pm_polled_group {
	uint32_t group_id;

	uint32_t bits_all_set;
	uint32_t bits_all_clear;
	uint32_t bits_any_set;
	uint32_t bits_any_clear;

	uint32_t value_read;
};

/*
 * Return true if all bits indicated by flag are set in source.
 */
static inline bool msm_pm_all_set(uint32_t source, uint32_t flag)
{
	return (source & flag) == flag;
}

/*
 * Return true if any bit indicated by flag are set in source.
 */
static inline bool msm_pm_any_set(uint32_t source, uint32_t flag)
{
	return !flag || (source & flag);
}

/*
 * Return true if all bits indicated by flag are cleared in source.
 */
static inline bool msm_pm_all_clear(uint32_t source, uint32_t flag)
{
	return (~source & flag) == flag;
}

/*
 * Return true if any bit indicated by flag are cleared in source.
 */
static inline bool msm_pm_any_clear(uint32_t source, uint32_t flag)
{
	return !flag || (~source & flag);
}

/*
 * Poll the shared memory states as indicated by the poll groups.
 *
 * nr_grps: number of groups in the array
 * grps: array of groups
 *
 * The function returns when conditions specified by any of the poll
 * groups become true.  The conditions specified by a poll group are
 * deemed true when 1) at least one bit from bits_any_set is set OR one
 * bit from bits_any_clear is cleared; and 2) all bits in bits_all_set
 * are set; and 3) all bits in bits_all_clear are cleared.
 *
 * Return value:
 *      >=0: index of the poll group whose conditions have become true
 *      -ETIMEDOUT: timed out
 */
static int msm_pm_poll_state(int nr_grps, struct msm_pm_polled_group *grps)
{
	int i, k;

	for (i = 0; i < 500000; i++)
		for (k = 0; k < nr_grps; k++) {
			bool all_set, all_clear;
			bool any_set, any_clear;

			grps[k].value_read = smsm_get_state(grps[k].group_id);

			all_set = msm_pm_all_set(grps[k].value_read,
					grps[k].bits_all_set);
			all_clear = msm_pm_all_clear(grps[k].value_read,
					grps[k].bits_all_clear);
			any_set = msm_pm_any_set(grps[k].value_read,
					grps[k].bits_any_set);
			any_clear = msm_pm_any_clear(grps[k].value_read,
					grps[k].bits_any_clear);

			if (all_set && all_clear && (any_set || any_clear))
				return k;
		}

	printk(KERN_ERR "%s failed:\n", __func__);
	for (k = 0; k < nr_grps; k++)
		printk(KERN_ERR "(%x, %x, %x, %x) %x\n",
			grps[k].bits_all_set, grps[k].bits_all_clear,
			grps[k].bits_any_set, grps[k].bits_any_clear,
			grps[k].value_read);

	return -ETIMEDOUT;
}


/******************************************************************************
 * Suspend Max Sleep Time
 *****************************************************************************/

#define SCLK_HZ (32768)
#define MSM_PM_SLEEP_TICK_LIMIT (0x6DDD000)

#ifdef CONFIG_MSM_SLEEP_TIME_OVERRIDE
static int msm_pm_sleep_time_override;
module_param_named(sleep_time_override,
	msm_pm_sleep_time_override, int, S_IRUGO | S_IWUSR | S_IWGRP);
#endif

static uint32_t msm_pm_max_sleep_time;

/*
 * Convert time from nanoseconds to slow clock ticks, then cap it to the
 * specified limit
 */
static int64_t msm_pm_convert_and_cap_time(int64_t time_ns, int64_t limit)
{
	do_div(time_ns, NSEC_PER_SEC / SCLK_HZ);
	return (time_ns > limit) ? limit : time_ns;
}

/*
 * Set the sleep time for suspend.  0 means infinite sleep time.
 */
void msm_pm_set_max_sleep_time(int64_t max_sleep_time_ns)
{
	unsigned long flags;

	local_irq_save(flags);
	if (max_sleep_time_ns == 0) {
		msm_pm_max_sleep_time = 0;
	} else {
		msm_pm_max_sleep_time = (uint32_t)msm_pm_convert_and_cap_time(
			max_sleep_time_ns, MSM_PM_SLEEP_TICK_LIMIT);

		if (msm_pm_max_sleep_time == 0)
			msm_pm_max_sleep_time = 1;
	}

	MSM_PM_DPRINTK(MSM_PM_DEBUG_SUSPEND, KERN_INFO,
		"%s(): Requested %lld ns Giving %u sclk ticks\n", __func__,
		max_sleep_time_ns, msm_pm_max_sleep_time);
	local_irq_restore(flags);
}
EXPORT_SYMBOL(msm_pm_set_max_sleep_time);


/******************************************************************************
 * CONFIG_MSM_IDLE_STATS
 *****************************************************************************/

#ifdef CONFIG_MSM_IDLE_STATS
enum msm_pm_time_stats_id {
	MSM_PM_STAT_REQUESTED_IDLE,
	MSM_PM_STAT_IDLE_SPIN,
	MSM_PM_STAT_IDLE_WFI,
	MSM_PM_STAT_IDLE_RAMP_DOWN_AND_WFI,
	MSM_PM_STAT_IDLE_SLEEP,
	MSM_PM_STAT_IDLE_FAILED_SLEEP,
	MSM_PM_STAT_IDLE_POWER_COLLAPSE,
	MSM_PM_STAT_IDLE_FAILED_POWER_COLLAPSE,
	MSM_PM_STAT_SUSPEND,
	MSM_PM_STAT_FAILED_SUSPEND,
	MSM_PM_STAT_NOT_IDLE,
	MSM_PM_STAT_COUNT
};

static struct msm_pm_time_stats {
	const char *name;
	int64_t first_bucket_time;
	int bucket[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t min_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int64_t max_time[CONFIG_MSM_IDLE_STATS_BUCKET_COUNT];
	int count;
	int64_t total_time;
} msm_pm_stats[MSM_PM_STAT_COUNT] = {
	[MSM_PM_STAT_REQUESTED_IDLE].name = "idle-request",
	[MSM_PM_STAT_REQUESTED_IDLE].first_bucket_time =
		CONFIG_MSM_IDLE_STATS_FIRST_BUCKET,

	[MSM_PM_STAT_IDLE_SPIN].name = "idle-spin",
	[MSM_PM_STAT_IDLE_SPIN].first_bucket_time =
		CONFIG_MSM_IDLE_STATS_FIRST_BUCKET,

	[MSM_PM_STAT_IDLE_WFI].name = "idle-wfi",
	[MSM_PM_STAT_IDLE_WFI].first_bucket_time =
		CONFIG_MSM_IDLE_STATS_FIRST_BUCKET,

	[MSM_PM_STAT_IDLE_RAMP_DOWN_AND_WFI].name = "idle-ramp-down-wfi",
	[MSM_PM_STAT_IDLE_RAMP_DOWN_AND_WFI].first_bucket_time =
		CONFIG_MSM_IDLE_STATS_FIRST_BUCKET,

	[MSM_PM_STAT_IDLE_SLEEP].name = "idle-sleep",
	[MSM_PM_STAT_IDLE_SLEEP].first_bucket_time =
		CONFIG_MSM_IDLE_STATS_FIRST_BUCKET,

	[MSM_PM_STAT_IDLE_FAILED_SLEEP].name = "idle-failed-sleep",
	[MSM_PM_STAT_IDLE_FAILED_SLEEP].first_bucket_time =
		CONFIG_MSM_IDLE_STATS_FIRST_BUCKET,

	[MSM_PM_STAT_IDLE_POWER_COLLAPSE].name = "idle-power-collapse",
	[MSM_PM_STAT_IDLE_POWER_COLLAPSE].first_bucket_time =
		CONFIG_MSM_IDLE_STATS_FIRST_BUCKET,

	[MSM_PM_STAT_IDLE_FAILED_POWER_COLLAPSE].name =
		"idle-failed-power-collapse",
	[MSM_PM_STAT_IDLE_FAILED_POWER_COLLAPSE].first_bucket_time =
		CONFIG_MSM_IDLE_STATS_FIRST_BUCKET,

	[MSM_PM_STAT_SUSPEND].name = "suspend",
	[MSM_PM_STAT_SUSPEND].first_bucket_time =
		CONFIG_MSM_SUSPEND_STATS_FIRST_BUCKET,

	[MSM_PM_STAT_FAILED_SUSPEND].name = "failed-suspend",
	[MSM_PM_STAT_FAILED_SUSPEND].first_bucket_time =
		CONFIG_MSM_IDLE_STATS_FIRST_BUCKET,

	[MSM_PM_STAT_NOT_IDLE].name = "not-idle",
	[MSM_PM_STAT_NOT_IDLE].first_bucket_time =
		CONFIG_MSM_IDLE_STATS_FIRST_BUCKET,
};

static uint32_t msm_pm_sleep_limit = SLEEP_LIMIT_NONE;
static DECLARE_BITMAP(msm_pm_clocks_no_tcxo_shutdown, NR_CLKS);

/*
 * Add the given time data to the statistics collection.
 */
static void msm_pm_add_stat(enum msm_pm_time_stats_id id, int64_t t)
{
	int i;
	int64_t bt;

	msm_pm_stats[id].total_time += t;
	msm_pm_stats[id].count++;

	bt = t;
	do_div(bt, msm_pm_stats[id].first_bucket_time);

	if (bt < 1ULL << (CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT *
				(CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1)))
		i = DIV_ROUND_UP(fls((uint32_t)bt),
					CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT);
	else
		i = CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1;

	msm_pm_stats[id].bucket[i]++;

	if (t < msm_pm_stats[id].min_time[i] || !msm_pm_stats[id].max_time[i])
		msm_pm_stats[id].min_time[i] = t;
	if (t > msm_pm_stats[id].max_time[i])
		msm_pm_stats[id].max_time[i] = t;
}

/*
 * Helper function of snprintf where buf is auto-incremented, size is auto-
 * decremented, and there is no return value.
 *
 * NOTE: buf and size must be l-values (e.g. variables)
 */
#define SNPRINTF(buf, size, format, ...) \
	do { \
		if (size > 0) { \
			int ret; \
			ret = snprintf(buf, size, format, ## __VA_ARGS__); \
			if (ret > size) { \
				buf += size; \
				size = 0; \
			} else { \
				buf += ret; \
				size -= ret; \
			} \
		} \
	} while (0)

/*
 * Write out the power management statistics.
 */
static int msm_pm_read_proc
	(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int i;
	char *p = page;
	char clk_name[16];

	if (count < 1024) {
		*start = (char *) 0;
		*eof = 0;
		return 0;
	}

	if (!off) {
		SNPRINTF(p, count, "Clocks against last TCXO shutdown:\n");
		for_each_bit(i, msm_pm_clocks_no_tcxo_shutdown, NR_CLKS) {
			clk_name[0] = '\0';
			msm_clock_get_name(i, clk_name, sizeof(clk_name));
			SNPRINTF(p, count, "  %s (id=%d)\n", clk_name, i);
		}

		SNPRINTF(p, count, "Last power collapse voted ");
		if (msm_pm_sleep_limit == SLEEP_LIMIT_NONE)
			SNPRINTF(p, count, "for TCXO shutdown\n\n");
		else
			SNPRINTF(p, count, "against TCXO shutdown\n\n");

		*start = (char *) 1;
		*eof = 0;
	} else if (--off < ARRAY_SIZE(msm_pm_stats)) {
		int64_t bucket_time;
		int64_t s;
		uint32_t ns;

		s = msm_pm_stats[off].total_time;
		ns = do_div(s, NSEC_PER_SEC);
		SNPRINTF(p, count,
			"%s:\n"
			"  count: %7d\n"
			"  total_time: %lld.%09u\n",
			msm_pm_stats[off].name,
			msm_pm_stats[off].count,
			s, ns);

		bucket_time = msm_pm_stats[off].first_bucket_time;
		for (i = 0; i < CONFIG_MSM_IDLE_STATS_BUCKET_COUNT - 1; i++) {
			s = bucket_time;
			ns = do_div(s, NSEC_PER_SEC);
			SNPRINTF(p, count,
				"   <%6lld.%09u: %7d (%lld-%lld)\n",
				s, ns, msm_pm_stats[off].bucket[i],
				msm_pm_stats[off].min_time[i],
				msm_pm_stats[off].max_time[i]);

			bucket_time <<= CONFIG_MSM_IDLE_STATS_BUCKET_SHIFT;
		}

		SNPRINTF(p, count, "  >=%6lld.%09u: %7d (%lld-%lld)\n",
			s, ns, msm_pm_stats[off].bucket[i],
			msm_pm_stats[off].min_time[i],
			msm_pm_stats[off].max_time[i]);

		*start = (char *) 1;
		*eof = (off + 1 >= ARRAY_SIZE(msm_pm_stats));
	}

	return p - page;
}
#undef SNPRINTF

#define MSM_PM_STATS_RESET "reset"

/*
 * Reset the power management statistics values.
 */
static int msm_pm_write_proc(struct file *file, const char __user *buffer,
	unsigned long count, void *data)
{
	char buf[sizeof(MSM_PM_STATS_RESET)];
	int ret;
	unsigned long flags;
	int i;

	if (count < strlen(MSM_PM_STATS_RESET)) {
		ret = -EINVAL;
		goto write_proc_failed;
	}

	if (copy_from_user(buf, buffer, strlen(MSM_PM_STATS_RESET))) {
		ret = -EFAULT;
		goto write_proc_failed;
	}

	if (memcmp(buf, MSM_PM_STATS_RESET, strlen(MSM_PM_STATS_RESET))) {
		ret = -EINVAL;
		goto write_proc_failed;
	}

	local_irq_save(flags);
	for (i = 0; i < ARRAY_SIZE(msm_pm_stats); i++) {
		memset(msm_pm_stats[i].bucket,
			0, sizeof(msm_pm_stats[i].bucket));
		memset(msm_pm_stats[i].min_time,
			0, sizeof(msm_pm_stats[i].min_time));
		memset(msm_pm_stats[i].max_time,
			0, sizeof(msm_pm_stats[i].max_time));
		msm_pm_stats[i].count = 0;
		msm_pm_stats[i].total_time = 0;
	}

	msm_pm_sleep_limit = SLEEP_LIMIT_NONE;
	bitmap_zero(msm_pm_clocks_no_tcxo_shutdown, NR_CLKS);
	local_irq_restore(flags);

	return count;

write_proc_failed:
	return ret;
}
#undef MSM_PM_STATS_RESET
#endif /* CONFIG_MSM_IDLE_STATS */


/******************************************************************************
 * Shared Memory Bits
 *****************************************************************************/

#define DEM_MASTER_BITS_PER_CPU             6

/* Power Master State Bits - Per CPU */
#define DEM_MASTER_SMSM_RUN \
	(0x01UL << (DEM_MASTER_BITS_PER_CPU * SMSM_APPS_STATE))
#define DEM_MASTER_SMSM_RSA \
	(0x02UL << (DEM_MASTER_BITS_PER_CPU * SMSM_APPS_STATE))
#define DEM_MASTER_SMSM_PWRC_EARLY_EXIT \
	(0x04UL << (DEM_MASTER_BITS_PER_CPU * SMSM_APPS_STATE))
#define DEM_MASTER_SMSM_SLEEP_EXIT \
	(0x08UL << (DEM_MASTER_BITS_PER_CPU * SMSM_APPS_STATE))
#define DEM_MASTER_SMSM_READY \
	(0x10UL << (DEM_MASTER_BITS_PER_CPU * SMSM_APPS_STATE))
#define DEM_MASTER_SMSM_SLEEP \
	(0x20UL << (DEM_MASTER_BITS_PER_CPU * SMSM_APPS_STATE))

/* Power Slave State Bits */
#define DEM_SLAVE_SMSM_RUN                  (0x0001)
#define DEM_SLAVE_SMSM_PWRC                 (0x0002)
#define DEM_SLAVE_SMSM_PWRC_DELAY           (0x0004)
#define DEM_SLAVE_SMSM_PWRC_EARLY_EXIT      (0x0008)
#define DEM_SLAVE_SMSM_WFPI                 (0x0010)
#define DEM_SLAVE_SMSM_SLEEP                (0x0020)
#define DEM_SLAVE_SMSM_SLEEP_EXIT           (0x0040)
#define DEM_SLAVE_SMSM_MSGS_REDUCED         (0x0080)
#define DEM_SLAVE_SMSM_RESET                (0x0100)
#define DEM_SLAVE_SMSM_PWRC_SUSPEND         (0x0200)

/* Time Slave State Bits */
#define DEM_TIME_SLAVE_TIME_REQUEST         (0x0400)
#define DEM_TIME_SLAVE_TIME_POLL            (0x0800)
#define DEM_TIME_SLAVE_TIME_INIT            (0x1000)


/******************************************************************************
 *
 *****************************************************************************/
static struct msm_pm_smem_t *msm_pm_smem_data;
static uint32_t *msm_pm_reset_vector;
static atomic_t msm_pm_init_done = ATOMIC_INIT(0);

#ifdef CONFIG_WAKE_SOURCES
static struct wakeup_source msm_pm_wakeup_sources[] = {
	WAKEUP_SOURCE("UNKNOWN"),
	WAKEUP_SOURCE("SMD"),
	WAKEUP_SOURCE("INT"),
	WAKEUP_SOURCE("GPIO"),
	WAKEUP_SOURCE("BUTTON"),
	WAKEUP_SOURCE("TIMER"),
	WAKEUP_SOURCE("ALARM"),
	WAKEUP_SOURCE("OTHER"),
	WAKEUP_SOURCE("REMOTE"),
};

static struct wakeup_source *wakeup_source_unknown;
static struct wakeup_source *wakeup_source_smd;
static struct wakeup_source *wakeup_source_int;
static struct wakeup_source *wakeup_source_gpio;
static struct wakeup_source *wakeup_source_button;
static struct wakeup_source *wakeup_source_timer;
static struct wakeup_source *wakeup_source_alarm;
static struct wakeup_source *wakeup_source_other;
static struct wakeup_source *wakeup_source_remote;
#endif

void print_wakesource(uint32_t wakereason)
{
	printk("WAKE SOURCE(0x%x) = ", wakereason);

	if(wakereason & DEM_WAKEUP_REASON_NONE)
		printk("(Unknown)");
	if(wakereason & DEM_WAKEUP_REASON_SMD)
		printk("(SMD) name=%s prog=0x%x, proc=0x%x", 
			msm_pm_smem_data->smd_port_name,
			msm_pm_smem_data->rpc_prog,
			msm_pm_smem_data->rpc_proc);
	if(wakereason & DEM_WAKEUP_REASON_INT)
		printk("(INT)");
	if(wakereason & DEM_WAKEUP_REASON_GPIO)
		printk("(GPIO=%d)", msm_pm_smem_data->gpio);
	if(wakereason & DEM_WAKEUP_REASON_TIMER)
		printk("(TIMER)");
	if(wakereason & DEM_WAKEUP_REASON_ALARM)
		printk("(ALARM)");
	if(wakereason & DEM_WAKEUP_REASON_OTHER)
		printk("(OTHER)");
	if(wakereason & DEM_WAKEUP_REASON_REMOTE)
		printk("(REMOTE)");

	printk("\n");
}

#ifdef CONFIG_WAKE_SOURCES
void log_wakesource(uint32_t wakereason)
{
	/* use the wake source reporting mechanism */
	if (wakereason & DEM_WAKEUP_REASON_NONE)
		wakeup_event(wakeup_source_unknown, NULL);
	if (wakereason & DEM_WAKEUP_REASON_SMD)
		wakeup_event(wakeup_source_smd, "name=%s prog=0x%x proc=0x%x",
			msm_pm_smem_data->smd_port_name,
			msm_pm_smem_data->rpc_prog,
			msm_pm_smem_data->rpc_proc);
	if (wakereason & DEM_WAKEUP_REASON_INT)
		wakeup_event(wakeup_source_int, NULL);
	if (wakereason & DEM_WAKEUP_REASON_GPIO) {
		if (msm_pm_smem_data->gpio == 124)
			wakeup_event(wakeup_source_button, "POWER, GPIO=%d", msm_pm_smem_data->gpio);
		else
			wakeup_event(wakeup_source_gpio, "GPIO=%d", msm_pm_smem_data->gpio);
	}
	if (wakereason & DEM_WAKEUP_REASON_TIMER)
		wakeup_event(wakeup_source_timer, NULL);
	if (wakereason & DEM_WAKEUP_REASON_ALARM)
		wakeup_event(wakeup_source_alarm, NULL);
	if (wakereason & DEM_WAKEUP_REASON_OTHER)
		wakeup_event(wakeup_source_other, NULL);
	if (wakereason & DEM_WAKEUP_REASON_REMOTE)
		wakeup_event(wakeup_source_remote, NULL);
}
#endif

/*
 * Power collapse the Apps processor.  This function executes the handshake
 * protocol with Modem.
 *
 * Return value:
 *      -EAGAIN: modem reset occurred or early exit from power collapse
 *      -EBUSY: modem not ready for our power collapse -- no power loss
 *      -ETIMEDOUT: timed out waiting for modem's handshake -- no power loss
 *      0: success
 */
static int msm_pm_power_collapse
	(bool from_idle, uint32_t sleep_delay, uint32_t sleep_limit)
{
	struct msm_pm_polled_group state_grps[2];
	unsigned long saved_acpuclk_rate;
	uint32_t saved_vector[2];
	int collapsed = 0;
	int ret;
#ifdef CONFIG_MSM_ADM_OFF_AT_POWER_COLLAPSE
	unsigned id;
#endif
#ifdef CONFIG_MSM_UART3_OFF_AT_POWER_COLLAPSE
	unsigned uart_id;
#endif

	MSM_PM_DPRINTK(MSM_PM_DEBUG_SUSPEND|MSM_PM_DEBUG_POWER_COLLAPSE,
		KERN_INFO, "%s(): idle %d, delay %u, limit %u\n", __func__,
		(int)from_idle, sleep_delay, sleep_limit);

	if (!(smsm_get_state(SMSM_POWER_MASTER_DEM) & DEM_MASTER_SMSM_READY)) {
		MSM_PM_DPRINTK(
			MSM_PM_DEBUG_SUSPEND | MSM_PM_DEBUG_POWER_COLLAPSE,
			KERN_INFO, "%s(): master not ready\n", __func__);
		ret = -EBUSY;
		goto power_collapse_bail;
	}

	memset(msm_pm_smem_data, 0, sizeof(*msm_pm_smem_data));

	msm_irq_enter_sleep1(true, from_idle, &msm_pm_smem_data->irq_mask);
	msm_gpio_enter_sleep(from_idle);
	msm_gpio_store();
	chuck_mux_sleep_config();

	msm_pm_smem_data->sleep_time = sleep_delay;
	msm_pm_smem_data->resources_used = sleep_limit;

	smsm_change_state(SMSM_APPS_DEM,
		DEM_TIME_SLAVE_TIME_REQUEST | DEM_TIME_SLAVE_TIME_POLL,
		DEM_TIME_SLAVE_TIME_INIT);

	/* Enter PWRC/PWRC_SUSPEND */

	if (from_idle)
		smsm_change_state(SMSM_APPS_DEM, DEM_SLAVE_SMSM_RUN,
			DEM_SLAVE_SMSM_PWRC);
	else
		smsm_change_state(SMSM_APPS_DEM, DEM_SLAVE_SMSM_RUN,
			DEM_SLAVE_SMSM_PWRC | DEM_SLAVE_SMSM_PWRC_SUSPEND);

	MSM_PM_DEBUG_PRINT_STATE("msm_pm_power_collapse(): PWRC");
	MSM_PM_DEBUG_PRINT_SLEEP_INFO();

	memset(state_grps, 0, sizeof(state_grps));
	state_grps[0].group_id = SMSM_POWER_MASTER_DEM;
	state_grps[0].bits_all_set = DEM_MASTER_SMSM_RSA;
	state_grps[1].group_id = SMSM_MODEM_STATE;
	state_grps[1].bits_all_set = SMSM_RESET;

	ret = msm_pm_poll_state(ARRAY_SIZE(state_grps), state_grps);

	if (ret < 0) {
		printk(KERN_EMERG "%s(): power collapse entry "
			"timed out waiting for Modem's response\n", __func__);
		msm_pm_timeout();
	}

	if (ret == 1) {
		MSM_PM_DPRINTK(
			MSM_PM_DEBUG_SUSPEND|MSM_PM_DEBUG_POWER_COLLAPSE,
			KERN_INFO,
			"%s(): msm_pm_poll_state detected Modem reset\n",
			__func__);
		goto power_collapse_early_exit;
	}

	/* DEM Master in RSA */

	MSM_PM_DEBUG_PRINT_STATE("msm_pm_power_collapse(): PWRC RSA");

	ret = msm_irq_enter_sleep2(true, from_idle);
	if (ret < 0) {
		MSM_PM_DPRINTK(
			MSM_PM_DEBUG_SUSPEND|MSM_PM_DEBUG_POWER_COLLAPSE,
			KERN_INFO,
			"%s(): msm_irq_enter_sleep2 aborted, %d\n", __func__,
			ret);
		goto power_collapse_early_exit;
	}

#ifdef CONFIG_MSM_ADM_OFF_AT_POWER_COLLAPSE
	id = ADM_CLK;
	msm_proc_comm(PCOM_CLKCTL_RPC_DISABLE, &id, NULL);
#endif

#ifdef CONFIG_MSM_UART3_OFF_AT_POWER_COLLAPSE
	uart_id = UART3_CLK;
	msm_proc_comm(PCOM_CLKCTL_RPC_DISABLE, &uart_id, NULL);
#endif

#if CONFIG_MSM_POWER_DEBUG
	get_enabled_clocks("Clocks enabled before sleep\n");
	get_enabled_vregs("VREGs enabled before sleep\n");
#endif

	msm_pm_config_hw_before_power_down();
	MSM_PM_DEBUG_PRINT_STATE("msm_pm_power_collapse(): pre power down");

	saved_acpuclk_rate = acpuclk_power_collapse();
	MSM_PM_DPRINTK(MSM_PM_DEBUG_CLOCK, KERN_INFO,
		"%s(): change clock rate (old rate = %lu)\n", __func__,
		saved_acpuclk_rate);

	if (saved_acpuclk_rate == 0) {
		msm_pm_config_hw_after_power_up();
		goto power_collapse_early_exit;
	}

	saved_vector[0] = msm_pm_reset_vector[0];
	saved_vector[1] = msm_pm_reset_vector[1];
	msm_pm_reset_vector[0] = 0xE51FF004; /* ldr pc, 4 */
	msm_pm_reset_vector[1] = virt_to_phys(msm_pm_collapse_exit);

	MSM_PM_DPRINTK(MSM_PM_DEBUG_RESET_VECTOR, KERN_INFO,
		"%s(): vector %x %x -> %x %x\n", __func__,
		saved_vector[0], saved_vector[1],
		msm_pm_reset_vector[0], msm_pm_reset_vector[1]);

#ifdef CONFIG_WAKE_SOURCES
	if (!from_idle)
		clear_wakeup_events();
#endif

#ifdef CONFIG_CACHE_L2X0
	l2x0_suspend();
#endif

	collapsed = msm_pm_collapse();

#ifdef CONFIG_CACHE_L2X0
	l2x0_resume(collapsed);
#endif

	msm_pm_reset_vector[0] = saved_vector[0];
	msm_pm_reset_vector[1] = saved_vector[1];

	if (collapsed) {
		cpu_init();
		local_fiq_enable();
	}

	MSM_PM_DPRINTK(MSM_PM_DEBUG_SUSPEND | MSM_PM_DEBUG_POWER_COLLAPSE,
		KERN_INFO,
		"%s(): msm_pm_collapse returned %d\n", __func__, collapsed);

	MSM_PM_DPRINTK(MSM_PM_DEBUG_CLOCK, KERN_INFO,
		"%s(): restore clock rate to %lu\n", __func__,
		saved_acpuclk_rate);
	if (acpuclk_set_rate(saved_acpuclk_rate, SETRATE_PC) < 0)
		printk(KERN_ERR "%s(): failed to restore clock rate(%lu)\n",
			__func__, saved_acpuclk_rate);

#ifdef CONFIG_MSM_UART3_OFF_AT_POWER_COLLAPSE
	uart_id = UART3_CLK;
	msm_proc_comm(PCOM_CLKCTL_RPC_ENABLE, &uart_id, NULL);
#endif

#ifdef CONFIG_MSM_ADM_OFF_AT_POWER_COLLAPSE
	id = ADM_CLK;
	if (msm_proc_comm(PCOM_CLKCTL_RPC_ENABLE, &id, NULL) < 0 || id < 0)
		printk(KERN_ERR
			"%s(): failed to turn on ADM clock\n", __func__);
#endif

	if(!from_idle) {
		print_wakesource(msm_pm_smem_data->wakeup_reason);
#ifdef CONFIG_WAKE_SOURCES
		log_wakesource(msm_pm_smem_data->wakeup_reason);
#endif
	}

	msm_irq_exit_sleep1(msm_pm_smem_data->irq_mask,
		msm_pm_smem_data->wakeup_reason,
		msm_pm_smem_data->pending_irqs);

	msm_pm_config_hw_after_power_up();
	MSM_PM_DEBUG_PRINT_STATE("msm_pm_power_collapse(): post power up");

	memset(state_grps, 0, sizeof(state_grps));
	state_grps[0].group_id = SMSM_POWER_MASTER_DEM;
	state_grps[0].bits_any_set =
		DEM_MASTER_SMSM_RSA | DEM_MASTER_SMSM_PWRC_EARLY_EXIT;
	state_grps[1].group_id = SMSM_MODEM_STATE;
	state_grps[1].bits_all_set = SMSM_RESET;

	ret = msm_pm_poll_state(ARRAY_SIZE(state_grps), state_grps);

	if (ret < 0) {
		printk(KERN_EMERG "%s(): power collapse exit "
			"timed out waiting for Modem's response\n", __func__);
		msm_pm_timeout();
	}

	if (ret == 1) {
		MSM_PM_DPRINTK(
			MSM_PM_DEBUG_SUSPEND|MSM_PM_DEBUG_POWER_COLLAPSE,
			KERN_INFO,
			"%s(): msm_pm_poll_state detected Modem reset\n",
			__func__);
		goto power_collapse_early_exit;
	}


	/* Sanity check 
	 *
	 * Changed BUG_ON to printks as this does not seem to be a fatal issue.
	 * Between the last check with the modem on early exit conditions
	 * (interrupt, rpc, etc) and the actual power collapse, 
	 * modem could raise early exit that could be ignored by A11. 
	 *
	 * The following checks expose such problems,
	 * but the conditions seem to be harmless. As unhandled early exit conditions
	 * from the modem would cause modem to wake A11 up even if
	 * A11 goes into complete power collapse. Respective states for
	 * modem and A11 would be synchronized properly after this function
	 */


	if (collapsed) {
		//BUG_ON(!(state_grps[0].value_read & DEM_MASTER_SMSM_RSA));
		if(!(state_grps[0].value_read & DEM_MASTER_SMSM_RSA))
			printk(KERN_INFO"Power collpased, but state is: %d\n",state_grps[0].value_read);
	} else {
		//BUG_ON(!(state_grps[0].value_read &
		//		DEM_MASTER_SMSM_PWRC_EARLY_EXIT));
		if(!(state_grps[0].value_read & DEM_MASTER_SMSM_PWRC_EARLY_EXIT)){
			printk(KERN_INFO"Power collpase did not go through, but state is: %d\n",state_grps[0].value_read);
		}
		goto power_collapse_early_exit;
	}

	/* Enter WFPI */

	smsm_change_state(SMSM_APPS_DEM,
		DEM_SLAVE_SMSM_PWRC | DEM_SLAVE_SMSM_PWRC_SUSPEND,
		DEM_SLAVE_SMSM_WFPI);

	MSM_PM_DEBUG_PRINT_STATE("msm_pm_power_collapse(): WFPI");

	memset(state_grps, 0, sizeof(state_grps));
	state_grps[0].group_id = SMSM_POWER_MASTER_DEM;
	state_grps[0].bits_all_set = DEM_MASTER_SMSM_RUN;
	state_grps[1].group_id = SMSM_MODEM_STATE;
	state_grps[1].bits_all_set = SMSM_RESET;

	ret = msm_pm_poll_state(ARRAY_SIZE(state_grps), state_grps);

	if (ret < 0) {
		printk(KERN_EMERG "%s(): power collapse WFPI "
			"timed out waiting for Modem's response\n", __func__);
		msm_pm_timeout();
	}

	if (ret == 1) {
		MSM_PM_DPRINTK(
			MSM_PM_DEBUG_SUSPEND|MSM_PM_DEBUG_POWER_COLLAPSE,
			KERN_INFO,
			"%s(): msm_pm_poll_state detected Modem reset\n",
			__func__);
		ret = -EAGAIN;
		goto power_collapse_restore_gpio_bail;
	}

	/* DEM Master == RUN */

	MSM_PM_DEBUG_PRINT_STATE("msm_pm_power_collapse(): WFPI RUN");
	MSM_PM_DEBUG_PRINT_SLEEP_INFO();

	msm_irq_exit_sleep2(msm_pm_smem_data->irq_mask,
		msm_pm_smem_data->wakeup_reason,
		msm_pm_smem_data->pending_irqs);
	msm_irq_exit_sleep3(msm_pm_smem_data->irq_mask,
		msm_pm_smem_data->wakeup_reason,
		msm_pm_smem_data->pending_irqs);
	msm_gpio_exit_sleep();
	chuck_mux_wake_config();
	msm_gpio_restore();

	smsm_change_state(SMSM_APPS_DEM,
		DEM_SLAVE_SMSM_WFPI, DEM_SLAVE_SMSM_RUN);

	MSM_PM_DEBUG_PRINT_STATE("msm_pm_power_collapse(): RUN");

	smd_sleep_exit();

	if(!from_idle) {
		print_wakesource(msm_pm_smem_data->wakeup_reason);
#ifdef CONFIG_WAKE_SOURCES
		log_wakesource(msm_pm_smem_data->wakeup_reason);
#endif
	}

	return 0;

power_collapse_early_exit:
	/* Enter PWRC_EARLY_EXIT */

	smsm_change_state(SMSM_APPS_DEM,
		DEM_SLAVE_SMSM_PWRC | DEM_SLAVE_SMSM_PWRC_SUSPEND,
		DEM_SLAVE_SMSM_PWRC_EARLY_EXIT);

	MSM_PM_DEBUG_PRINT_STATE("msm_pm_power_collapse(): EARLY_EXIT");

	memset(state_grps, 0, sizeof(state_grps));
	state_grps[0].group_id = SMSM_POWER_MASTER_DEM;
	state_grps[0].bits_all_set = DEM_MASTER_SMSM_PWRC_EARLY_EXIT;
	state_grps[1].group_id = SMSM_MODEM_STATE;
	state_grps[1].bits_all_set = SMSM_RESET;

	ret = msm_pm_poll_state(ARRAY_SIZE(state_grps), state_grps);
	MSM_PM_DEBUG_PRINT_STATE("msm_pm_power_collapse(): EARLY_EXIT EE");

	if (ret < 0) {
		printk(KERN_EMERG "%s(): power collapse EARLY_EXIT "
			"timed out waiting for Modem's response\n", __func__);
		msm_pm_timeout();
	}

	if (ret == 1) {
		MSM_PM_DPRINTK(
			MSM_PM_DEBUG_SUSPEND|MSM_PM_DEBUG_POWER_COLLAPSE,
			KERN_INFO,
			"%s(): msm_pm_poll_state detected Modem reset\n",
			__func__);
	}

	/* DEM Master == RESET or PWRC_EARLY_EXIT */

	ret = -EAGAIN;

power_collapse_restore_gpio_bail:
	chuck_mux_wake_config();
	msm_gpio_exit_sleep();
	msm_gpio_restore();

	/* Enter RUN */
	smsm_change_state(SMSM_APPS_DEM,
		DEM_SLAVE_SMSM_PWRC | DEM_SLAVE_SMSM_PWRC_SUSPEND |
		DEM_SLAVE_SMSM_PWRC_EARLY_EXIT, DEM_SLAVE_SMSM_RUN);

	MSM_PM_DEBUG_PRINT_STATE("msm_pm_power_collapse(): RUN");

	if (collapsed)
		smd_sleep_exit();

power_collapse_bail:
	return ret;
}

/*
 * Apps-sleep the Apps processor.  This function execute the handshake
 * protocol with Modem.
 *
 * Return value:
 *      -ENOSYS: function not implemented yet
 */
static int msm_pm_apps_sleep(uint32_t sleep_delay, uint32_t sleep_limit)
{
	return -ENOSYS;
}

/*
 * Bring the Apps processor to SWFI.
 *
 * Return value:
 *      -EIO: could not ramp Apps processor clock
 *      0: success
 */
static int msm_pm_swfi(bool ramp_acpu)
{
	unsigned long saved_acpuclk_rate = 0;

	if (ramp_acpu) {
		saved_acpuclk_rate = acpuclk_wait_for_irq();
		MSM_PM_DPRINTK(MSM_PM_DEBUG_SWFI, KERN_INFO,
			"%s(): change clock rate (old rate = %lu)\n", __func__,
			saved_acpuclk_rate);

		if (!saved_acpuclk_rate)
			return -EIO;
	}

	msm_pm_config_hw_before_swfi();
	msm_arch_idle();

	if (ramp_acpu) {
		MSM_PM_DPRINTK(MSM_PM_DEBUG_SWFI, KERN_INFO,
			"%s(): restore clock rate to %lu\n", __func__,
			saved_acpuclk_rate);
		if (acpuclk_set_rate(saved_acpuclk_rate, SETRATE_SWFI) < 0)
			printk(KERN_ERR
				"%s(): failed to restore clock rate(%lu)\n",
				__func__, saved_acpuclk_rate);
	}

	return 0;
}


/******************************************************************************
 * External Idle/Suspend Functions
 *****************************************************************************/

/*
 * Put CPU in low power mode.
 */
void arch_idle(void)
{
	bool allow[MSM_PM_SLEEP_MODE_NR];
	uint32_t sleep_limit = SLEEP_LIMIT_NONE;

	//int latency_qos;
	int64_t timer_expiration;

	int low_power;
	int ret;
	int i;

#ifdef CONFIG_MSM_IDLE_STATS
	DECLARE_BITMAP(clk_ids, NR_CLKS);
	int64_t t1;
	static int64_t t2;
	int exit_stat;
#endif /* CONFIG_MSM_IDLE_STATS */


	if (!atomic_read(&msm_pm_init_done))
		return;

	//latency_qos = pm_qos_requirement(PM_QOS_CPU_DMA_LATENCY);
	timer_expiration = msm_timer_enter_idle();

#ifdef CONFIG_MSM_IDLE_STATS
	t1 = ktime_to_ns(ktime_get());
	msm_pm_add_stat(MSM_PM_STAT_NOT_IDLE, t1 - t2);
	msm_pm_add_stat(MSM_PM_STAT_REQUESTED_IDLE, timer_expiration);
#endif /* CONFIG_MSM_IDLE_STATS */

	for (i = 0; i < ARRAY_SIZE(allow); i++)
		allow[i] = true;

	switch (msm_pm_idle_sleep_mode) {
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		allow[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] =
			false;
		/* fall through */
	case MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT:
		allow[MSM_PM_SLEEP_MODE_APPS_SLEEP] = false;
		/* fall through */
	case MSM_PM_SLEEP_MODE_APPS_SLEEP:
		allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = false;
		allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = false;
		/* fall through */
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND:
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		break;
	default:
		printk(KERN_ERR "idle sleep mode is invalid: %d\n",
			msm_pm_idle_sleep_mode);
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_SPIN;
#endif /* CONFIG_MSM_IDLE_STATS */
		low_power = 0;
		goto arch_idle_exit;
	}

	if ((timer_expiration < msm_pm_idle_sleep_min_time) ||
		!msm_irq_idle_sleep_allowed()) {
		allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = false;
		allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = false;
		allow[MSM_PM_SLEEP_MODE_APPS_SLEEP] = false;
	}

#if 0
	for (i = 0; i < ARRAY_SIZE(allow); i++) {
		struct msm_pm_platform_data *mode = &msm_pm_modes[i];
		if (!mode->supported || !mode->idle_enabled ||
			mode->latency >= latency_qos ||
			mode->residency >= timer_expiration)
			allow[i] = false;
	}
#endif
#ifdef CONFIG_MSM_IDLE_STATS
	ret = msm_clock_require_tcxo(clk_ids, NR_CLKS);
#elif defined(CONFIG_CLOCK_BASED_SLEEP_LIMIT)
	ret = msm_clock_require_tcxo(NULL, 0);
#endif /* CONFIG_MSM_IDLE_STATS */

#ifdef CONFIG_CLOCK_BASED_SLEEP_LIMIT
	if (ret)
		sleep_limit = SLEEP_LIMIT_NO_TCXO_SHUTDOWN;
#endif

	MSM_PM_DPRINTK(MSM_PM_DEBUG_IDLE, KERN_INFO,
		"%s(): next timer %lld, sleep limit %u\n",
		__func__, timer_expiration, sleep_limit);

	for (i = 0; i < ARRAY_SIZE(allow); i++)
		MSM_PM_DPRINTK(MSM_PM_DEBUG_IDLE, KERN_INFO,
			"%s(): allow %s: %d\n", __func__,
			msm_pm_sleep_mode_labels[i], (int)allow[i]);

	if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] ||
		allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]) {
		uint32_t sleep_delay;

		sleep_delay = (uint32_t) msm_pm_convert_and_cap_time(
			timer_expiration, MSM_PM_SLEEP_TICK_LIMIT);
		if (sleep_delay == 0) /* 0 would mean infinite time */
			sleep_delay = 1;

		if (!allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE])
			sleep_limit = SLEEP_LIMIT_NO_TCXO_SHUTDOWN;

		ret = msm_pm_power_collapse(true, sleep_delay, sleep_limit);
		low_power = (ret != -EBUSY && ret != -ETIMEDOUT);

#ifdef CONFIG_MSM_IDLE_STATS
		if (ret)
			exit_stat = MSM_PM_STAT_IDLE_FAILED_POWER_COLLAPSE;
		else {
			exit_stat = MSM_PM_STAT_IDLE_POWER_COLLAPSE;
			msm_pm_sleep_limit = sleep_limit;
			bitmap_copy(msm_pm_clocks_no_tcxo_shutdown, clk_ids,
				NR_CLKS);
		}
#endif /* CONFIG_MSM_IDLE_STATS */
	} else if (allow[MSM_PM_SLEEP_MODE_APPS_SLEEP]) {
		uint32_t sleep_delay;

		sleep_delay = (uint32_t) msm_pm_convert_and_cap_time(
			timer_expiration, MSM_PM_SLEEP_TICK_LIMIT);
		if (sleep_delay == 0) /* 0 would mean infinite time */
			sleep_delay = 1;

		ret = msm_pm_apps_sleep(sleep_delay, sleep_limit);
		low_power = 0;

#ifdef CONFIG_MSM_IDLE_STATS
		if (ret)
			exit_stat = MSM_PM_STAT_IDLE_FAILED_SLEEP;
		else
			exit_stat = MSM_PM_STAT_IDLE_SLEEP;
#endif /* CONFIG_MSM_IDLE_STATS */
	} else if (allow[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT]) {
		ret = msm_pm_swfi(true);
		if (ret)
			while (!msm_irq_pending())
				udelay(1);
		low_power = 0;
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = ret ? MSM_PM_STAT_IDLE_SPIN : MSM_PM_STAT_IDLE_RAMP_DOWN_AND_WFI;
#endif /* CONFIG_MSM_IDLE_STATS */
	} else if (allow[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT]) {
		msm_pm_swfi(false);
		low_power = 0;
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_WFI;
#endif /* CONFIG_MSM_IDLE_STATS */
	} else {
		while (!msm_irq_pending())
			udelay(1);
		low_power = 0;
#ifdef CONFIG_MSM_IDLE_STATS
		exit_stat = MSM_PM_STAT_IDLE_SPIN;
#endif /* CONFIG_MSM_IDLE_STATS */
	}

arch_idle_exit:
	msm_timer_exit_idle(low_power);

#ifdef CONFIG_MSM_IDLE_STATS
	t2 = ktime_to_ns(ktime_get());
	msm_pm_add_stat(exit_stat, t2 - t1);
#endif /* CONFIG_MSM_IDLE_STATS */
}

/*
 * Suspend the Apps processor.
 *
 * Return value:
 *      -EAGAIN: modem reset occurred or early exit from suspend
 *      -EBUSY: modem not ready for our suspend
 *      -EINVAL: invalid sleep mode
 *      -EIO: could not ramp Apps processor clock
 *      -ETIMEDOUT: timed out waiting for modem's handshake
 *      0: success
 */
static int msm_pm_enter(suspend_state_t state)
{
	bool allow[MSM_PM_SLEEP_MODE_NR];
	uint32_t sleep_limit = SLEEP_LIMIT_NONE;
	int ret;
	int i;

#ifdef CONFIG_MSM_IDLE_STATS
	DECLARE_BITMAP(clk_ids, NR_CLKS);
	int64_t period = 0;
	int64_t time = 0;

	time = msm_timer_get_smem_clock_time(&period);
	ret = msm_clock_require_tcxo(clk_ids, NR_CLKS);
#elif defined(CONFIG_CLOCK_BASED_SLEEP_LIMIT)
	ret = msm_clock_require_tcxo(NULL, 0);
#endif /* CONFIG_MSM_IDLE_STATS */

#ifdef CONFIG_CLOCK_BASED_SLEEP_LIMIT
	if (ret)
		sleep_limit = SLEEP_LIMIT_NO_TCXO_SHUTDOWN;
#endif

	for (i = 0; i < ARRAY_SIZE(allow); i++)
		allow[i] = true;

	switch (msm_pm_sleep_mode) {
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		allow[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] =
			false;
		/* fall through */
	case MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT:
		allow[MSM_PM_SLEEP_MODE_APPS_SLEEP] = false;
		/* fall through */
	case MSM_PM_SLEEP_MODE_APPS_SLEEP:
		allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = false;
		allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = false;
		/* fall through */
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_SUSPEND:
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		break;
	default:
		printk(KERN_ERR "suspend sleep mode is invalid: %d\n",
			msm_pm_sleep_mode);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(allow); i++) {
		struct msm_pm_platform_data *mode = &msm_pm_modes[i];
		if (!mode->supported || !mode->suspend_enabled)
			allow[i] = false;
	}

	ret = 0;

	if (allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] ||
		allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]) {
#ifdef CONFIG_MSM_IDLE_STATS
		enum msm_pm_time_stats_id id;
		int64_t end_time;
#endif

#ifdef CONFIG_MSM_SLEEP_TIME_OVERRIDE
		if (msm_pm_sleep_time_override > 0) {
			int64_t ns;
			ns = NSEC_PER_SEC * (int64_t)msm_pm_sleep_time_override;
			msm_pm_set_max_sleep_time(ns);
			msm_pm_sleep_time_override = 0;
		}
#endif
		if (!allow[MSM_PM_SLEEP_MODE_POWER_COLLAPSE])
			sleep_limit = SLEEP_LIMIT_NO_TCXO_SHUTDOWN;

		ret = msm_pm_power_collapse(
			false, msm_pm_max_sleep_time, sleep_limit);

#ifdef CONFIG_MSM_IDLE_STATS
		if (ret)
			id = MSM_PM_STAT_FAILED_SUSPEND;
		else {
			id = MSM_PM_STAT_SUSPEND;
			msm_pm_sleep_limit = sleep_limit;
			bitmap_copy(msm_pm_clocks_no_tcxo_shutdown, clk_ids,
				NR_CLKS);
		}

		if (time != 0) {
			end_time = msm_timer_get_smem_clock_time(NULL);
			if (end_time != 0) {
				time = end_time - time;
				if (time < 0)
					time += period;
			} else
				time = 0;
		}

		msm_pm_add_stat(id, time);
#endif
	} else if (allow[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT]) {
		ret = msm_pm_swfi(true);
		if (ret)
			while (!msm_irq_pending())
				udelay(1);
	} else if (allow[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT]) {
		msm_pm_swfi(false);
	}

	return ret;
}

int msm_pm_prepare(void)
{
	// clear wakeup reason (if necessary).
	return fastpath_prepare();
}

int msm_pm_fastsleep(void)
{
	int wakeup_is_rtc;

	wakeup_is_rtc = msm_pm_smem_data->wakeup_reason &
					DEM_WAKEUP_REASON_TIMER;

	return fastpath_fastsleep(wakeup_is_rtc);
}

static struct platform_suspend_ops msm_pm_ops = {
	.prepare = msm_pm_prepare,
	.enter = msm_pm_enter,
	.valid = suspend_valid_only_mem,
	.fastsleep = msm_pm_fastsleep,
};

static void msm_pm_power_off(void)
{
	msm_proc_comm(PCOM_POWER_DOWN, 0, 0);
	for (;;)
		;
}

static void msm_pm_restart(char str)
{
	/* If there's a hard reset hook and the restart_reason
	 * is the default, prefer that to the (slower) proc_comm
	 * reset command.
	 */
#if 1
	if ((restart_reason == 0x776655AA) && msm_reset_hook)
		msm_reset_hook(str);
	else
		msm_proc_comm(PCOM_RESET_CHIP, &restart_reason, 0);
#endif
	for (;;)
		;
}

volatile static int panic_ref = 0;

static int msm_panic_call(struct notifier_block *this, unsigned long event, void *ptr)
{
	if (panic_ref)
		return NOTIFY_DONE;
	panic_ref++;

	restart_reason = RESTART_REASON_PANIC;
	return NOTIFY_DONE;
}

static struct notifier_block panic_notifier = {
	.notifier_call = msm_panic_call,
};

static int msm_reboot_call
	(struct notifier_block *this, unsigned long code, void *_cmd)
{
	if ((code == SYS_RESTART) && _cmd) {
		char *cmd = _cmd;
		if (!strcmp(cmd, "bootloader")) {
			restart_reason = 0x77665500;
		} else if (!strcmp(cmd, "recovery")) {
			restart_reason = 0x77665502;
		} else if (!strcmp(cmd, "eraseflash")) {
			restart_reason = 0x776655EF;
		}
		// Palm specific
		else if (!strcmp(cmd, "shutdown")) {
			restart_reason = RESTART_REASON_SHUTDOWN;
		} else if (!strcmp(cmd, "recover")) {
			restart_reason = RESTART_REASON_RECOVER;
		} else if (!strcmp(cmd, "panic")) {
			restart_reason = RESTART_REASON_PANIC;
		} else if (!strcmp(cmd, "dfu")) {
			restart_reason = RESTART_REASON_DFU;
		} else if (!strcmp(cmd, "reboot")) {
			restart_reason = RESTART_REASON_REBOOT;
		} else {
			restart_reason = RESTART_REASON_UNKNOWN;
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block msm_reboot_notifier = {
	.notifier_call = msm_reboot_call,
};


/******************************************************************************
 *
 *****************************************************************************/

/*
 * Initialize the power management subsystem.
 *
 * Return value:
 *      -ENODEV: initialization failed
 *      0: success
 */
static int __init msm_pm_init(void)
{
#ifdef CONFIG_MSM_IDLE_STATS
	struct proc_dir_entry *d_entry;
#endif

	pm_power_off = msm_pm_power_off;
	arm_pm_restart = msm_pm_restart;
	register_reboot_notifier(&msm_reboot_notifier);
	atomic_notifier_chain_register(&panic_notifier_list, &panic_notifier);

	msm_pm_smem_data = smem_alloc(SMEM_APPS_DEM_SLAVE_DATA,
		sizeof(*msm_pm_smem_data));
	if (msm_pm_smem_data == NULL) {
		printk(KERN_ERR "%s: failed to get smsm_data\n", __func__);
		return -ENODEV;
	}

#ifdef CONFIG_ARCH_QSD
	/* The bootloader is responsible for initializing many of Scorpion's
	 * coprocessor registers for things like cache timing. The state of
	 * these coprocessor registers is lost on reset, so part of the
	 * bootloader must be re-executed. Do not overwrite the reset vector
	 * or bootloader area.
	 */
	msm_pm_reset_vector = (uint32_t *) PAGE_OFFSET;
#else
	msm_pm_reset_vector = ioremap(0, PAGE_SIZE);
	if (msm_pm_reset_vector == NULL) {
		printk(KERN_ERR "%s: failed to map reset vector\n", __func__);
		return -ENODEV;
	}
#endif /* CONFIG_ARCH_QSD */

	BUG_ON(msm_pm_modes == NULL);

	atomic_set(&msm_pm_init_done, 1);
	suspend_set_ops(&msm_pm_ops);

#ifdef CONFIG_MSM_IDLE_STATS
	d_entry = create_proc_entry("msm_pm_stats",
			S_IRUGO | S_IWUSR | S_IWGRP, NULL);
	if (d_entry) {
		d_entry->read_proc = msm_pm_read_proc;
		d_entry->write_proc = msm_pm_write_proc;
		d_entry->data = NULL;
	}
#endif

#ifdef CONFIG_WAKE_SOURCES
	init_wakeup_sources(msm_pm_wakeup_sources,
			sizeof(msm_pm_wakeup_sources) / sizeof(struct wakeup_source));

	wakeup_source_unknown = get_wakeup_source("UNKNOWN");
	wakeup_source_smd = get_wakeup_source("SMD");
	wakeup_source_int = get_wakeup_source("INT");
	wakeup_source_gpio = get_wakeup_source("GPIO");
	wakeup_source_button = get_wakeup_source("BUTTON");
	wakeup_source_timer = get_wakeup_source("TIMER");
	wakeup_source_alarm = get_wakeup_source("ALARM");
	wakeup_source_other = get_wakeup_source("OTHER");
	wakeup_source_remote = get_wakeup_source("REMOTE");
#endif

	return 0;
}

late_initcall(msm_pm_init);
