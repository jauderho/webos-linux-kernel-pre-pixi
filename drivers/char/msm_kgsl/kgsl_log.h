/* Copyright (c) 2002,2008-2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef _GSL_LOG_H
#define _GSL_LOG_H

#include <linux/bug.h>
#include <linux/types.h>
#include <linux/msm_kgsl.h>
#include <linux/device.h>

extern unsigned int kgsl_drv_log;
extern unsigned int kgsl_cmd_log;
extern unsigned int kgsl_ctxt_log;
extern unsigned int kgsl_mem_log;

struct device *kgsl_driver_getdevnode(void);
int kgsl_debug_init(void);

#define KGSL_LOG_VDBG(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 7)  \
			dev_vdbg(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_DBG(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 7)  \
			dev_dbg(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_INFO(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 6)  \
			dev_info(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_WARN(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 4)  \
			dev_warn(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_ERR(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 3)  \
			dev_err(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_LOG_FATAL(lvl, fmt, args...) \
	do { \
		if ((lvl) >= 2) \
			dev_crit(kgsl_driver_getdevnode(), "|%s| " fmt, \
					__func__, ##args);\
	} while (0)

#define KGSL_DRV_VDBG(fmt, args...) KGSL_LOG_VDBG(kgsl_drv_log, fmt, ##args)
#define KGSL_DRV_DBG(fmt, args...)  KGSL_LOG_DBG(kgsl_drv_log, fmt, ##args)
#define KGSL_DRV_INFO(fmt, args...) KGSL_LOG_INFO(kgsl_drv_log, fmt, ##args)
#define KGSL_DRV_WARN(fmt, args...) KGSL_LOG_WARN(kgsl_drv_log, fmt, ##args)
#define KGSL_DRV_ERR(fmt, args...)  KGSL_LOG_ERR(kgsl_drv_log, fmt, ##args)
#define KGSL_DRV_FATAL(fmt, args...) KGSL_LOG_FATAL(kgsl_drv_log, fmt, ##args)

#define KGSL_CMD_VDBG(fmt, args...) KGSL_LOG_VDBG(kgsl_cmd_log, fmt, ##args)
#define KGSL_CMD_DBG(fmt, args...)  KGSL_LOG_DBG(kgsl_cmd_log, fmt, ##args)
#define KGSL_CMD_INFO(fmt, args...) KGSL_LOG_INFO(kgsl_cmd_log, fmt, ##args)
#define KGSL_CMD_WARN(fmt, args...) KGSL_LOG_WARN(kgsl_cmd_log, fmt, ##args)
#define KGSL_CMD_ERR(fmt, args...)  KGSL_LOG_ERR(kgsl_cmd_log, fmt, ##args)
#define KGSL_CMD_FATAL(fmt, args...) KGSL_LOG_FATAL(kgsl_cmd_log, fmt, ##args)

#define KGSL_CTXT_VDBG(fmt, args...) KGSL_LOG_VDBG(kgsl_ctxt_log, fmt, ##args)
#define KGSL_CTXT_DBG(fmt, args...)  KGSL_LOG_DBG(kgsl_ctxt_log, fmt, ##args)
#define KGSL_CTXT_INFO(fmt, args...) KGSL_LOG_INFO(kgsl_ctxt_log, fmt, ##args)
#define KGSL_CTXT_WARN(fmt, args...) KGSL_LOG_WARN(kgsl_ctxt_log, fmt, ##args)
#define KGSL_CTXT_ERR(fmt, args...)  KGSL_LOG_ERR(kgsl_ctxt_log, fmt, ##args)
#define KGSL_CTXT_FATAL(fmt, args...) KGSL_LOG_FATAL(kgsl_ctxt_log, fmt, ##args)

#define KGSL_MEM_VDBG(fmt, args...) KGSL_LOG_VDBG(kgsl_mem_log, fmt, ##args)
#define KGSL_MEM_DBG(fmt, args...)  KGSL_LOG_DBG(kgsl_mem_log, fmt, ##args)
#define KGSL_MEM_INFO(fmt, args...) KGSL_LOG_INFO(kgsl_mem_log, fmt, ##args)
#define KGSL_MEM_WARN(fmt, args...) KGSL_LOG_WARN(kgsl_mem_log, fmt, ##args)
#define KGSL_MEM_ERR(fmt, args...)  KGSL_LOG_ERR(kgsl_mem_log, fmt, ##args)
#define KGSL_MEM_FATAL(fmt, args...) KGSL_LOG_FATAL(kgsl_mem_log, fmt, ##args)

#endif /* _GSL_LOG_H */
