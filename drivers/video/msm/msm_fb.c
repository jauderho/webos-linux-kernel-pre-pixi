/* drivers/video/msm_fb/msm_fb.c
 *
 * Core MSM framebuffer driver.
 *
 * Copyright (C) 2007 Google Incorporated
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/delay.h>

#include <linux/freezer.h>
#include <linux/wait.h>
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif
#include <linux/msm_mdp.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/arch/msm_fb.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/msm_mdp.h>
#include <linux/file.h>

#include "mdp_hw.h"

#define MSMFB_DEBUG 0
#ifdef CONFIG_FB_MSM_LOGO
#define INIT_IMAGE_FILE "/logo.rle"
extern int load_565rle_image( char *filename );
#endif

#define PRINT_FPS 0
#define PRINT_BLIT_TIME 0

#define SLEEPING 0x4
#define UPDATING 0x3
#define FULL_UPDATE_DONE 0x2
#define WAKING 0x1
#define AWAKE 0x0

#define NONE 0
#define SUSPEND_RESUME 0x1
#define FPS 0x2
#define BLIT_TIME 0x4
#define VIDEO_IDLE_THRESHOLD_MS 32
#define DMA_IDLE_THRESHOLD_MS 500

#if MSMFB_DEBUG
#define	MDP_LOG_ENTER()		(printk(KERN_INFO"%s: called\n",\
					__PRETTY_FUNCTION__))
#define	MDP_LOG_EXIT()		(printk(KERN_INFO"%s: exit\n",\
					__PRETTY_FUNCTION__))
#define MDP_LOG_INFO(args...)	(printk(KERN_INFO args))					
#else
#define MDP_LOG_ENTER()
#define MDP_LOG_EXIT()
#define MDP_LOG_INFO(args...)
#endif // MSMFB_DEBUG			

/* define the custom FBIO_WAITFORVSYNC ioctl */
#define FBIO_WAITFORVSYNC	_IOW('F', 0x20, u_int32_t)		

#define DLOG(mask,fmt,args...) \
do { \
if (msmfb_debug_mask & mask) \
	printk(KERN_INFO "msmfb: "fmt, ##args); \
} while (0)

#define MSMVIDEO_DEBUG_MSGS 0
#if MSMVIDEO_DEBUG_MSGS
#define VIDEOLOG(fmt,args...) \
	do { printk(KERN_INFO "[%s:%s:%d] "fmt, __FILE__, __func__, __LINE__, \
		    ##args); } \
	while (0)
#else
#define VIDEOLOG(x...) do {} while (0)
#endif

static int msmfb_debug_mask;
module_param_named(msmfb_debug_mask, msmfb_debug_mask, int,
		   S_IRUGO | S_IWUSR | S_IWGRP);

struct msmfb_info {
	struct fb_info *fb_info;
	struct task_struct *task;
	struct mddi_panel_info *panel_info;
	unsigned yoffset;
	unsigned frame_requested;
	unsigned frame_done;
	int sleeping;
	unsigned update_frame;
	struct {
		int left;
		int top;
		int eright; /* exclusive */
		int ebottom; /* exclusive */
	} update_info;

#ifdef CONFIG_ANDROID_POWER
	android_early_suspend_t early_suspend;
	android_early_suspend_t slightly_earlier_suspend;
	android_suspend_lock_t idle_lock;
#endif
	spinlock_t update_lock;
	wait_queue_head_t frame_wq;
	char* black;
	struct work_struct resume_work;
	struct msmfb_callback dma_callback;
	struct msmfb_callback vsync_callback;
	struct msmfb_callback displayon_callback;
	struct msmfb_callback displayoff_callback;
	struct hrtimer fake_vsync;

	unsigned int autoupdate;
	wait_queue_head_t update_wq;
	struct mutex autoupdate_lock;
	struct mutex videoupdate_lock;
	

	unsigned int frameupdate;
	wait_queue_head_t wq;

	unsigned int pipeline_status;
	wait_queue_head_t pipeline_wq;

	struct mdp_blit_int_req last_video_req;
	uint32_t last_addr; 
	ktime_t last_update_time;
	ktime_t last_check_time;
	ktime_t last_dma_time;
	struct hrtimer video_idle_check;
	struct work_struct	push_video_work;

	int pipeline;
	uint32_t dma2_ibufformat;
};

#define FB_READY	0
#define FB_UPDATING	1

#define PIPELINE_LOCKED		0
#define PIPELINE_UNLOCKED	1

struct video_info {
	struct msmfb_info *info;
	struct msmfb_callback vsync_callback;
	struct msmfb_callback dma_callback;
	uint32_t addr;
	uint32_t dma2_ibufformat;
	struct msmfb_callback* user_callback;
};

static struct video_info *queued_video_info;

static uint32_t bytes_per_pixel[] = {
	[MDP_RGB_565] = 2,
	[MDP_RGB_888] = 3,
	[MDP_XRGB_8888] = 4,
	[MDP_ARGB_8888] = 4,
	[MDP_RGBA_8888] = 4,
	[MDP_Y_CBCR_H2V1] = 1,
	[MDP_Y_CBCR_H2V2] = 1,
	[MDP_Y_CRCB_H2V1] = 1,
	[MDP_Y_CRCB_H2V2] = 1,
	[MDP_YCRYCB_H2V1] = 2
};


struct fb_info *saved_context;

static void msmfb_force_update(void);


static int msmfb_open(struct fb_info *info, int user)
{
	return 0;
}

static int msmfb_release(struct fb_info *info, int user)
{
	return 0;
}

static void msmfb_start_video_dma(struct video_info *vinfo)
{
	uint32_t stride;
	unsigned long irq_flags;

	VIDEOLOG("msm_fb_start_video_dma: address=0x%x\n", vinfo->addr);

	if ((vinfo->info->pipeline == PIPELINE_GRAPHICS))
	{
		VIDEOLOG("msm_fb_start_video_dma:Already switched to graphics mode.\n");
		return;
	}

	spin_lock_irqsave(&vinfo->info->update_lock, irq_flags);
	vinfo->info->frame_requested++;
	spin_unlock_irqrestore(&vinfo->info->update_lock, irq_flags);

	switch(vinfo->dma2_ibufformat)
	{
		case DMA_IBUF_FORMAT_RGB888:
			stride = 0x3C0;
			break;
		case DMA_IBUF_FORMAT_ARGB8888:
			stride = 0x500;
			break;
		default:
			stride = 0x3C0;
			break;
	}

	mdp_dma_to_mddi(vinfo->addr, stride, 320, 400, 0, 0, &vinfo->dma_callback,
		vinfo->dma2_ibufformat);
}

static void msmfb_video_handle_dma_interrupt(struct msmfb_callback *callback)
{
	unsigned long irq_flags;
	struct msmfb_info *par; 

	struct video_info *vinfo = container_of(callback, struct video_info,
						dma_callback);

	VIDEOLOG("msmfb_video_handle_dma_interrupt+++\n");

	if(vinfo->user_callback)
		vinfo->user_callback->func(vinfo->user_callback);

	par = vinfo->info;

	spin_lock_irqsave(&par->update_lock, irq_flags);
	par->frame_done = par->frame_requested;
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
	wake_up(&par->frame_wq);

	par->pipeline_status = PIPELINE_UNLOCKED;
	wake_up(&par->pipeline_wq);

	kfree(vinfo);
}

/* Called from dma interrupt handler, must not sleep */
static void msmfb_handle_dma_interrupt(struct msmfb_callback *callback)
{
	unsigned long irq_flags;
	struct msmfb_info *par  = container_of(callback, struct msmfb_info,
					       dma_callback);
#if PRINT_FPS
	int64_t dt;
	ktime_t now;
	static int64_t frame_count;
	static ktime_t last_sec;
#endif

	spin_lock_irqsave(&par->update_lock, irq_flags);
	par->frame_done = par->frame_requested;
	if (par->sleeping == UPDATING && par->frame_done == par->update_frame) {
		DLOG(SUSPEND_RESUME, "full update completed\n");
		schedule_work(&par->resume_work);
	}
#if PRINT_FPS
	now = ktime_get();
	dt = ktime_to_ns(ktime_sub(now, last_sec));
	frame_count++;
	if (dt > NSEC_PER_SEC) {
		int64_t fps = frame_count * NSEC_PER_SEC * 100;
		frame_count = 0;
		last_sec = ktime_get();
		do_div(fps, dt);
		DLOG(FPS, "fps * 100: %llu\n", fps);
	}
#endif
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
	wake_up(&par->frame_wq);

	par->frameupdate = FB_READY;
	wake_up(&par->wq);
}

static int msmfb_start_dma(struct msmfb_info *par)
{
	uint32_t x, y, w, h;
	unsigned addr;
	unsigned long irq_flags;
	uint32_t yoffset;
	struct mddi_panel_info *pi = par->panel_info;

	spin_lock_irqsave(&par->update_lock, irq_flags);
	if (par->frame_done == par->frame_requested) {
		spin_unlock_irqrestore(&par->update_lock, irq_flags);
		return -1;
	}

	if (pi->force_full_update) {
		/* Force full update of the screen for clients that do not support partial */
		x = 0;
		y = 0;
		w = par->fb_info->var.xres;
		h = par->fb_info->var.yres;
		yoffset = par->yoffset;
	} else {
		// the partial update width and starting x position must be even.
		// if the x position of the update box is odd then we make it
		// even by expanding the box by 1 pixel (subtract 1).  the width is calculated
		// by taking the box width - starting position.  if that result is
		// odd we will add 1 to increase the box size by a pixel.
		x = par->update_info.left;
		if((x % 2) != 0) x-=1;
		y = par->update_info.top;
		w = par->update_info.eright - x;
		if((w % 2) != 0) w+=1;
		h = par->update_info.ebottom - y;
		yoffset = par->yoffset;
	}
	
	par->update_info.left = pi->width + 1;
	par->update_info.top = pi->height + 1;
	par->update_info.eright = 0;
	par->update_info.ebottom = 0;
	if (unlikely(w > pi->width || h > pi->height || w == 0 || h == 0)) {
		printk(KERN_INFO "invalid update: %d %d %d %d\n", x, y, w, h);
		par->frame_done = par->frame_requested;
		goto error;
	}
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
	
	addr = (par->fb_info->fix.line_length * (yoffset + y)) + (x * (pi->bits_per_pixel >> 3));

	MDP_LOG_INFO("addr:0x%x x:%d, y:%d, w:%d, h:%d\n", addr + par->fb_info->fix.smem_start, x, y, w, h);

	par->last_dma_time = ktime_get();

	mdp_dma_to_mddi(addr + par->fb_info->fix.smem_start,
		par->fb_info->fix.line_length, w, h, x, y, &par->dma_callback, 
		par->dma2_ibufformat);

	return 0;

error:
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
	/* some clients clear their vsync interrupt
	 * when the link activates */
	mddi_activate_link(pi->mddi);
	return 0;
}

/* Called from esync interrupt handler, must not sleep */
static void msmfb_video_handle_vsync_interrupt(struct msmfb_callback *callback)
{
	struct video_info *info  = container_of(callback, struct video_info,
					       vsync_callback);

	msmfb_start_video_dma(info);
}


/* Called from esync interrupt handler, must not sleep */
static void msmfb_handle_vsync_interrupt(struct msmfb_callback *callback)
{
	struct msmfb_info *par  = container_of(callback, struct msmfb_info,
					       vsync_callback);
#ifdef CONFIG_ANDROID_POWER
	android_unlock_suspend(&par->idle_lock);
#endif
	msmfb_start_dma(par);
}

static void msmfb_push_frame(struct work_struct *work)
{
	struct msmfb_info *par  = container_of(work, struct msmfb_info,
					       push_video_work);
	
	if(par)
	{
		if ((par->pipeline == PIPELINE_VIDEO)) 
		{
			msmfb_overlay(&par->last_video_req, par->last_addr, NULL);
		}
		else
		{
			msmfb_force_update();
		}
	}

}

static enum hrtimer_restart msmfb_video_idle_check(struct hrtimer *timer)
{
	struct msmfb_info *par  = container_of(timer, struct msmfb_info,
					       video_idle_check);

	/* Push frame with the last update time remains the same */
 	if(ktime_equal(par->last_check_time,par->last_update_time))
	{
		VIDEOLOG("msmfb_video_idle_check push video frame\n");
		schedule_work( &par->push_video_work );
	}
		
	return HRTIMER_NORESTART;
}


static enum hrtimer_restart msmfb_fake_vsync(struct hrtimer *timer)
{
	struct msmfb_info *par  = container_of(timer, struct msmfb_info,
					       fake_vsync);

	if ((par->pipeline == PIPELINE_VIDEO) &&
		(NULL != queued_video_info)) {
		msmfb_start_video_dma(queued_video_info);
		queued_video_info = NULL;
	}
	else
		msmfb_start_dma(par);
		
	return HRTIMER_NORESTART;
}

static void msmfb_pan_update(struct fb_info *info, uint32_t left, uint32_t top,
			 uint32_t eright, uint32_t ebottom, uint32_t yoffset,
			 int pan_display)
{
	struct msmfb_info *par = info->par;
	struct mddi_panel_info *pi = par->panel_info;
	unsigned long irq_flags;
	int sleeping;
	ktime_t time; 
#if PRINT_FPS
	ktime_t t1, t2;
	static uint64_t pans;
	static uint64_t dt;
	t1 = ktime_get();
#endif

	if(par->pipeline == PIPELINE_VIDEO)
	{
		uint64_t video_idle_time; 
		VIDEOLOG("In Video Mode: offset=%d\n", yoffset);
		par->yoffset = yoffset;
		par->frameupdate = FB_UPDATING;

		time = ktime_get();
		video_idle_time = ktime_to_ns(ktime_sub(time, par->last_update_time));
		do_div(video_idle_time, 1000000);

		/*
		 *Will force an update in the video pipeline based on the last
		 *video frame if the video pipeline is detected to be stalled.
		 */
		if(video_idle_time > VIDEO_IDLE_THRESHOLD_MS && par->last_addr > 0 )
		{
			VIDEOLOG("msmfb_pan_update push video frame\n");
			msmfb_overlay(&par->last_video_req, par->last_addr, NULL);
		}
		else
		{
 			if (!hrtimer_active(&par->video_idle_check)) 
			{
				par->last_check_time = par->last_update_time;
                        	hrtimer_start(&par->video_idle_check,
                                      ktime_set(0, NSEC_PER_SEC/30),
                                      HRTIMER_MODE_REL);
            }
		}

		return;
	}

	MDP_LOG_ENTER();
	MDP_LOG_INFO("left %d, top %d, eright %d, ebottom %d\n",
	       		left, top, eright, ebottom);
	MDP_LOG_INFO("f_req %d, f_done %d\n",
		     par->frame_requested, par->frame_done);
restart:
	spin_lock_irqsave(&par->update_lock, irq_flags);

	/* if we are sleeping, on a pan_display wait 10ms (to throttle back
	 * drawing otherwise return */
	if (par->sleeping == SLEEPING) {
		DLOG(SUSPEND_RESUME, "drawing while asleep\n");
		spin_unlock_irqrestore(&par->update_lock, irq_flags);
		if (pan_display)
			wait_event_interruptible_timeout(par->frame_wq,
				par->sleeping == AWAKE, HZ/10);
		return;
	}

	sleeping = par->sleeping;
	/* on a full update, if the last frame has not completed, wait for it */
	if (pan_display && par->frame_requested != par->frame_done) {
		spin_unlock_irqrestore(&par->update_lock, irq_flags);
		if (wait_event_interruptible_timeout(par->frame_wq,
		    par->frame_done == par->frame_requested, HZ) <= 0) {

 			uint64_t dma_time;
			ktime_t time;

			printk(KERN_WARNING "msmfb_pan_display timeout waiting "
					    "for frame start, %d %d\n",
					    par->frame_requested,
					    par->frame_done);

			time = ktime_get();
			dma_time = ktime_to_ns(ktime_sub(time, par->last_dma_time));
			do_div(dma_time, 1000000);

			if(dma_time < DMA_IDLE_THRESHOLD_MS)
				return;

			printk(KERN_WARNING "msmfb_pan_display: Start time of last DMA was over %dms ago\n", DMA_IDLE_THRESHOLD_MS); 
			par->frame_done = par->frame_requested;
			mdp_print_isr_status();
		}
		goto restart;
	}

#if PRINT_FPS
	t2 = ktime_get();
	if (pan_display) {
		uint64_t temp = ktime_to_ns(ktime_sub(t2, t1));
		do_div(temp, 1000);
		dt += temp;
		pans++;
		if (pans > 1000) {
			do_div(dt, pans);
			DLOG(FPS, "ave_wait_time: %lld\n", dt);
			dt = 0;
			pans = 0;
		}
	}
#endif

	par->frame_requested++;
	/* if necessary, update the y offset, if this is the
	 * first full update on resume, set the sleeping state */
	if (pan_display) {
		par->yoffset = yoffset;
		if (left == 0 && top == 0 && eright == info->var.xres &&
		    ebottom == info->var.yres) {
			if (sleeping == WAKING) {
				par->update_frame = par->frame_requested;
				DLOG(SUSPEND_RESUME, "full update starting\n");
				par->sleeping = UPDATING;
			}
		}
	}

	/* set the update request */
	if (left < par->update_info.left)
		par->update_info.left = left;
	if (top < par->update_info.top)
		par->update_info.top = top;
	if (eright > par->update_info.eright)
		par->update_info.eright = eright;
	if (ebottom > par->update_info.ebottom)
		par->update_info.ebottom = ebottom;

	par->frameupdate = FB_UPDATING;
	spin_unlock_irqrestore(&par->update_lock, irq_flags);

	/* if the panel is all the way on wait for vsync, otherwise sleep
	 * for 16 ms (long enough for the dma to panel) and then begin dma */
	if (pi->panel_ops->request_vsync && (sleeping == AWAKE)) {
#ifdef CONFIG_ANDROID_POWER
		android_lock_idle_auto_expire(&par->idle_lock, HZ/20);
#endif
		pi->panel_ops->request_vsync(pi, &par->vsync_callback);
	} else {
		if (!hrtimer_active(&par->fake_vsync)) {
			hrtimer_start(&par->fake_vsync,
				      ktime_set(0, NSEC_PER_SEC/60),
				      HRTIMER_MODE_REL);
		}
	}
}

static void msmfb_update(struct fb_info *info, uint32_t left, uint32_t top,
			 uint32_t eright, uint32_t ebottom)
{
	msmfb_pan_update(info, left, top, eright, ebottom, 0, 0);
}

static void msmfb_force_update(void)
{
	struct fb_info *fb = saved_context;
	struct msmfb_info *par = fb->par;
	
	if(fb && par)
	{
		//force video pipeline to burst a frame 
		if(par->pipeline == PIPELINE_VIDEO)
			par->last_update_time = ktime_set(0,0);

		msmfb_update(fb, 0, 0, fb->var.xres, fb->var.yres);
	}

	return;
}

static void msmfb_display_on(struct msmfb_callback *callback)
{
	struct msmfb_info *par  = container_of(callback, struct msmfb_info,
				       displayon_callback);

	printk("msmfb_display_on()\n");

	par->sleeping = AWAKE;
	msmfb_force_update();
}

static void msmfb_display_off(struct msmfb_callback *callback)
{
	long rc;
	struct msmfb_info *par  = container_of(callback, struct msmfb_info,
					       displayoff_callback);

	printk("msmfb_display_off()\n");

	par->sleeping = SLEEPING;

	rc = wait_event_interruptible_timeout(par->wq, par->frameupdate == FB_READY,
		msecs_to_jiffies(100));

	if(rc == 0)
		printk("msmfb_display_off: timed out waiting for dma completion\n");
}

static void power_on_panel(struct work_struct *work)
{
	struct msmfb_info *par = container_of(work, struct msmfb_info,
					      resume_work);
	unsigned long irq_flags;

	struct mddi_panel_info *pi = par->panel_info;
	DLOG(SUSPEND_RESUME, "turning on panel\n");
	pi->panel_ops->power(pi, 1);
	spin_lock_irqsave(&par->update_lock, irq_flags);
	par->sleeping = AWAKE;
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
}

#ifdef CONFIG_ANDROID_POWER
/* turn off the panel */
static void msmfb_slightly_earlier_suspend(android_early_suspend_t *h)
{
	struct msmfb_info *par = container_of(h, struct msmfb_info,
					      slightly_earlier_suspend);
	struct mddi_panel_info *pi = par->panel_info;
	unsigned int irq_flags;

	spin_lock_irqsave(&par->update_lock, irq_flags);
	par->sleeping = SLEEPING;
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
	wait_event_timeout(par->frame_wq,
			   par->frame_requested == par->frame_done, HZ/10);

	/* blank the screen */
	msmfb_update(par->fb_info, 0, 0, par->fb_info->var.xres,
		     par->fb_info->var.yres);
	mdp_dma_to_mddi(virt_to_phys(par->black), 0,
			par->fb_info->var.xres, par->fb_info->var.yres, 0, 0,
			NULL, par->dma2_ibufformat);
	mdp_dma_wait();
	/* turn off the backlight and the panel */
	pi->panel_ops->power(pi, 0);
}

/* userspace has stopped drawing */
static void msmfb_early_suspend(android_early_suspend_t *h)
{
}

/* turn on the panel */
static void msmfb_early_resume(android_early_suspend_t *h)
{
	struct msmfb_info *par = container_of(h, struct msmfb_info,
				 early_suspend);
	unsigned int irq_flags;

	spin_lock_irqsave(&par->update_lock, irq_flags);
	par->frame_requested = par->frame_done = par->update_frame = 0;
	par->sleeping = WAKING;
	DLOG(SUSPEND_RESUME, "ready, waiting for full update\n");
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
}

/* userspace has started drawing */
static void msmfb_slightly_later_resume(android_early_suspend_t *h)
{
}
#endif

static int msmfb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	if (var->xres != info->var.xres) return -EINVAL;
	if (var->yres != info->var.yres) return -EINVAL;
	if (var->xres_virtual != info->var.xres_virtual) return -EINVAL;
	if (var->yres_virtual != info->var.yres_virtual) return -EINVAL;
	if (var->xoffset != info->var.xoffset) return -EINVAL;
	if (var->bits_per_pixel != info->var.bits_per_pixel) return -EINVAL;
	if (var->grayscale != info->var.grayscale) return -EINVAL;
	return 0;
}

int msmfb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	/* "UPDT" */
	if (var->reserved[0] == 0x54445055) {
#if 0
		printk(KERN_INFO "pan rect %d %d %d %d\n",
		       var->reserved[1] & 0xffff,
		       var->reserved[1] >> 16, var->reserved[2] & 0xffff,
		       var->reserved[2] >> 16);
#endif
		msmfb_pan_update(info, var->reserved[1] & 0xffff,
				 var->reserved[1] >> 16,
				 var->reserved[2] & 0xffff,
				 var->reserved[2] >> 16, var->yoffset, 1);
	} else {
		msmfb_pan_update(info, 0, 0, info->var.xres, info->var.yres,
		var->yoffset, 1);
	}
	return 0;
}

static void msmfb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
	cfb_fillrect(p, rect);
	msmfb_update(p, rect->dx, rect->dy, rect->dx + rect->width,
		     rect->dy + rect->height);
}

static void msmfb_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
	cfb_copyarea(p, area);
	msmfb_update(p, area->dx, area->dy, area->dx + area->width,
		     area->dy + area->height);
}

static void msmfb_imageblit(struct fb_info *p, const struct fb_image *image)
{
	cfb_imageblit(p, image);
	msmfb_update(p, image->dx, image->dy, image->dx + image->width,
		     image->dy + image->height);
}

/*
 * Normally returns file pointer of the given fd. However:
 * Returns NULL if the file pointer of the given fd is not found
 * Returns -1 if the fd is the frame buffer's fd
 *
 */
static  struct file* msmfb_get_filp(int fd)
{
	int put_needed;
	struct file *f = NULL;

	if(!fd)
		return f;	

	f = fget_light(fd, &put_needed);
	
	if(!f)
	{
              printk(KERN_INFO "msmfb_get_filp failed, cannot retrieve filp from fd: %d\n", fd);
		
	}

	fput_light(f, put_needed);

 	if(MAJOR(f->f_dentry->d_inode->i_rdev) == FB_MAJOR)
		f = (struct file*)-1;



	return f;
}

static int msmfb_blit(struct fb_info *info, void __user *p)
{
	struct mdp_blit_req_list ereq_list;
 	struct mdp_blit_req ereq; 
	struct mdp_blit_int_req ireq;
	int i;
	int ret;

	if (copy_from_user(&ereq_list, p, sizeof(ereq_list)))
		return -EFAULT;

	for (i = 0; i < ereq_list.count; i++) 
	{
		struct mdp_blit_req_list *list =
			(struct mdp_blit_req_list *)p;

		if (copy_from_user(&ereq, &list->req[i], sizeof(ereq)))
			return -EFAULT;

		ireq.src.width = ereq.src.width;
		ireq.src.height = ereq.src.height;
		ireq.src.format = ereq.src.format;
		ireq.src.offset = ereq.src.offset;
		ireq.src.filp = msmfb_get_filp(ereq.src.memory_id);

		ireq.bg.width  = ereq.dst.width;
		ireq.bg.height = ereq.dst.height;
		ireq.bg.format = ereq.dst.format;
		ireq.bg.offset = ereq.dst.offset;
		ireq.bg.filp = msmfb_get_filp(ereq.bg.memory_id);

		ireq.dst.width  = ereq.dst.width;
		ireq.dst.height = ereq.dst.height;
		ireq.dst.format = ereq.dst.format;
		ireq.dst.offset = ereq.dst.offset;
		ireq.dst.filp = msmfb_get_filp(ereq.dst.memory_id);

		ireq.transp_mask = ereq.transp_mask;
		ireq.flags = ereq.flags;
		ireq.alpha  = ereq.alpha;

		ireq.dst_rect = ereq.dst_rect;
		ireq.src_rect = ereq.src_rect;

		ret = mdp_blit(info, &ireq);
		if (ret)
			return ret;
	}

	return 0;
}


int msmfb_switch_pipeline(int pipeline)
{
	int ret = 0;
	long rc;
	struct msmfb_info *par = saved_context->par;

	rc = wait_event_interruptible_timeout(par->pipeline_wq,
			par->pipeline_status == PIPELINE_UNLOCKED, msecs_to_jiffies(100));

	mutex_lock(&par->videoupdate_lock);

	if(rc == 0) 
	{
		printk("%s: timeout waiting for pipeline to free up\n", __func__);
	}

	if(par->pipeline == pipeline)
	{
		VIDEOLOG("Already setup for %s\n", par->pipeline == 1 ? "GRAPHICS" : "VIDEO");
	}
	else if(pipeline == PIPELINE_GRAPHICS)
	{
		VIDEOLOG("Switching FB to Graphics pipeline\n");
		par->pipeline = pipeline;
	}
	else if(pipeline == PIPELINE_VIDEO)
	{
		VIDEOLOG("Switching FB to Video pipeline\n");
		par->pipeline = pipeline;
	}
	else
	{
		VIDEOLOG("msmfb_switch_pipeline: %d is an invalid pipeline\n", pipeline);
		ret = -EINVAL;
	}

	mutex_unlock(&par->videoupdate_lock);
	
	return ret;
}
EXPORT_SYMBOL(msmfb_switch_pipeline);


void queue_video_frame(struct msmfb_info *info, uint32_t address, uint32_t format, struct msmfb_callback* user_callback)
{
	struct video_info *frame = kzalloc(sizeof(struct video_info), GFP_KERNEL);

	VIDEOLOG("queue_video_frame+++\n");

	frame->vsync_callback.func = msmfb_video_handle_vsync_interrupt;
	frame->dma_callback.func = msmfb_video_handle_dma_interrupt;
	frame->addr = address;
	frame->dma2_ibufformat = format;
	frame->user_callback = user_callback;
	frame->info = info;

	queued_video_info = frame;

	/* if the panel is all the way on wait for vsync, otherwise sleep
	 * for 16 ms (long enough for the dma to panel) and then begin dma */
	if (info->panel_info->panel_ops->request_vsync) {
		info->panel_info->panel_ops->request_vsync(info->panel_info, &frame->vsync_callback);
	} else {
		if (!hrtimer_active(&info->fake_vsync)) {
			hrtimer_start(&info->fake_vsync,
				      ktime_set(0, NSEC_PER_SEC/60),
				      HRTIMER_MODE_REL);
		}
	}
}


int msmfb_overlay(struct mdp_blit_int_req *req, uint32_t address, struct msmfb_callback* user_callback)
{
	struct fb_info *info = saved_context;
	struct msmfb_info *par = info->par;
	int ret = 0;

	mutex_lock(&par->videoupdate_lock);

	par->pipeline_status = PIPELINE_LOCKED;

	if(par->pipeline == PIPELINE_GRAPHICS)
	{
		printk("%s: switched to graphics, bail out\n", __func__);
		goto done;
	}

	//Only remember when the call is from actual video pipeline,
	//but not forced
	if(user_callback && address)
	{
		par->last_addr = address;
		par->last_video_req = *req;
		par->last_update_time = ktime_get();
	}

	VIDEOLOG("msmfb_overlay+++\n");

	//the yoffset is the number of lines, not the total size so we must calculate that
	req->bg.offset = req->bg.width * bytes_per_pixel[req->bg.format] * par->yoffset;
	
	// for the overlay we don't really need to support multiple requests since we
	// know the overlay can happen in a single pass.
	ret = mdp_blit(info, req);

	// after blit we don't need to prevent the graphics buffer from panning again.
	par->frameupdate = FB_READY;
	wake_up(&par->wq);

	VIDEOLOG("msmfb_overlay: done blitting\n");

	if(!info)
	{
		printk(KERN_ERR "fb_info handle is bad!\n");
		ret = -EINVAL;
		goto done;
	}

	if(!address) {
		VIDEOLOG("Address is NULL - Dont display this buffer\n");
		goto done;
	} 

	// mdp_blit won't return until the operation has completed so it's safe to start
	// the dma2 operation to display the overlayed frame now.
	queue_video_frame(par, address, DMA_IBUF_FORMAT_RGB888, user_callback);

	mutex_unlock(&par->videoupdate_lock);
	
	return ret;
done:
	mutex_unlock(&par->videoupdate_lock);
	par->pipeline_status = PIPELINE_UNLOCKED;
	wake_up(&par->pipeline_wq);
	
	return ret;
}
EXPORT_SYMBOL(msmfb_overlay);

static int msmfb_constant_update(void *data)
{
	struct fb_info *fb = (struct fb_info *)data;
	struct msmfb_info *par = (struct msmfb_info *)fb->par;
	

	VIDEOLOG("msmfb_constant_update+++\n");
	
	while (!kthread_should_stop()) 
	{

		wait_event_interruptible(par->update_wq, par->autoupdate == 1);

		// initiate DMA transfer of entire frame buffer
		msmfb_update(fb, 0, 0, fb->var.xres, fb->var.yres);

		// sleep for 33ms
		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(33));
	}

	printk("msmfb_constant_update---\n");

	return 0;
}

DECLARE_MUTEX(mdp_ppp_sem);

static int msmfb_ioctl(struct fb_info *p, unsigned int cmd, unsigned long arg)
{
	int userarg=0;
	struct msmfb_info *par = (struct msmfb_info *)p->par;
	int ret;
#if PRINT_BLIT_TIME
	ktime_t t1, t2;
#endif

	switch (cmd)
	{
		case MSMFB_AUTO_UPDATE:
			userarg = (int)arg;
			if(userarg == 1)
			{
				mutex_lock(&par->autoupdate_lock);
				par->autoupdate = 1;
				mutex_unlock(&par->autoupdate_lock);
				wake_up(&par->update_wq);
			}
			else	
			{
				mutex_lock(&par->autoupdate_lock);
				par->autoupdate = 0;
				mutex_unlock(&par->autoupdate_lock);
			}
			break;
		case FBIO_WAITFORVSYNC:
		{
			long rc;

			if(par->sleeping == SLEEPING) {
				DLOG(SUSPEND_RESUME, "SLEEPING: don't wait for vsync!\n");
				break;
			}

			rc = wait_event_interruptible_timeout(par->wq, par->frameupdate == FB_READY,
				msecs_to_jiffies(100));

			if(rc == 0) 
			{
				if(par->pipeline == PIPELINE_VIDEO)
					VIDEOLOG("msmfb_ioctl(FBIO_WAITFORVSYNC): timed out waiting in video pipeline for dma completion\n");
				else
					printk("msmfb_ioctl(FBIO_WAITFORVSYNC): timed out waiting for dma completion\n");
			}

			break;
		}
		case MSMFB_GRP_DISP:
			mdp_set_grp_disp(arg);
			break;
		case MSMFB_BLIT:
		{
#if PRINT_BLIT_TIME
			t1 = ktime_get();
#endif
			down(&mdp_ppp_sem);
			ret = msmfb_blit(p, (void __user *)arg);
			up(&mdp_ppp_sem);
			if (ret)
				return ret;
#if PRINT_BLIT_TIME
			t2 = ktime_get();
			DLOG(BLIT_TIME, "total %lld\n",
			       ktime_to_ns(t2) - ktime_to_ns(t1));
#endif
			break;
		}
		default:
			printk(KERN_INFO "msmfb unknown ioctl: %d\n", cmd);
			return -EINVAL;
	}
	return 0;
}

static struct fb_ops msmfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = msmfb_open,
	.fb_release = msmfb_release,
	.fb_check_var = msmfb_check_var,
	.fb_pan_display = msmfb_pan_display,
	.fb_fillrect = msmfb_fillrect,
	.fb_copyarea = msmfb_copyarea,
	.fb_imageblit = msmfb_imageblit,
	.fb_ioctl = msmfb_ioctl,
};

static unsigned PP[16];


#if MSMFB_DEBUG
static ssize_t debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t debug_read(struct file *file, char __user *buf, size_t count,
                          loff_t *ppos)
{
	const int debug_bufmax = 4096;
	static char buffer[4096];
	int n = 0;
	struct msmfb_info *par = (struct msmfb_info *)file->private_data;
	unsigned long irq_flags;

	spin_lock_irqsave(&par->update_lock, irq_flags);
	n = scnprintf(buffer, debug_bufmax, "yoffset %d\n", par->yoffset);
	n += scnprintf(buffer + n, debug_bufmax, "frame_requested %d\n", par->frame_requested);
	n += scnprintf(buffer + n, debug_bufmax, "frame_done %d\n", par->frame_done);
	n += scnprintf(buffer + n, debug_bufmax, "sleeping %d\n", par->sleeping);
	n += scnprintf(buffer + n, debug_bufmax, "update_frame %d\n", par->update_frame);
	spin_unlock_irqrestore(&par->update_lock, irq_flags);
	n++;
	buffer[n] = 0;
	return simple_read_from_buffer(buf, count, ppos, buffer, n);
}

static struct file_operations debug_fops = {
        .read = debug_read,
        .open = debug_open,
};
#endif

#ifdef CONFIG_PM 

static int msmfb_suspend(struct platform_device* pdev, pm_message_t state)
{
	return 0;
}

static int msmfb_resume(struct platform_device* pdev)
{
	msmfb_force_update();
	
	return 0;
}
#else
#define msmfb_suspend  NULL
#define msmfb_resume   NULL
#endif  /* CONFIG_PM */

static int msmfb_probe(struct platform_device *pdev)
{
	unsigned char *fbram;
	struct fb_info *info;
	struct msmfb_info *par;
	struct mddi_panel_info *pi = pdev->dev.platform_data;
	int r;

	printk(KERN_INFO "msmfb_probe() installing %d x %d panel\n",
	       pi->width, pi->height);

	fbram = ioremap(pi->fb_base, pi->fb_size);

	if (fbram == 0) {
		printk(KERN_ERR "cannot allocate fbram!\n");
		return -ENOMEM;
	}

	info = framebuffer_alloc(sizeof(struct msmfb_info), &pdev->dev);
	par = info->par;
	saved_context = info;
	par->fb_info = info;
	par->pipeline = PIPELINE_GRAPHICS;
	par->dma2_ibufformat = DMA_IBUF_FORMAT_ARGB8888;
	par->panel_info = pi;
	par->dma_callback.func = msmfb_handle_dma_interrupt;
	par->vsync_callback.func = msmfb_handle_vsync_interrupt;
	hrtimer_init(&par->fake_vsync, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
#ifdef CONFIG_ANDROID_POWER
	par->idle_lock.name = "msmfb_idle_lock";
	android_init_suspend_lock(&par->idle_lock);
#endif
	par->fake_vsync.function = msmfb_fake_vsync;
	hrtimer_init(&par->video_idle_check, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	par->video_idle_check.function = msmfb_video_idle_check;
	spin_lock_init(&par->update_lock);
	init_waitqueue_head(&par->frame_wq);
	init_waitqueue_head(&par->update_wq);
	init_waitqueue_head(&par->wq);
	init_waitqueue_head(&par->pipeline_wq);
	par->pipeline_status = PIPELINE_UNLOCKED;
	par->frameupdate = FB_READY;

	info->screen_base = fbram;
	strncpy(info->fix.id, "msmfb", 16);
	info->fix.smem_start = pi->fb_base;
	info->fix.smem_len = pi->fb_size;
	info->fix.ypanstep = 1;

	info->fbops = &msmfb_ops;
	info->flags = FBINFO_DEFAULT;

	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.line_length = pi->width * (pi->bits_per_pixel >> 3); 	// width * bytes_per_pixel

	info->var.xres = pi->width;
	info->var.yres = pi->height;
	info->var.xres_virtual = pi->width;
	// yres_virtual is height * number of available framebuffers
	info->var.yres_virtual = pi->height *  (pi->fb_size / (info->fix.line_length * pi->height));
	info->var.bits_per_pixel = pi->bits_per_pixel;
	info->var.accel_flags = 0;

	info->var.yoffset = 0;
	info->var.reserved[0] = 0x54445055;
	info->var.reserved[1] = 0;
	info->var.reserved[2] = (uint16_t)pi->width |
				((uint32_t)pi->height << 16);

	switch(pi->bits_per_pixel)
	{
		case 32:
			info->var.red.offset = 16;
			info->var.red.length = 8;
			info->var.red.msb_right = 0;
			info->var.green.offset = 8;
			info->var.green.length = 8;
			info->var.green.msb_right = 0;
			info->var.blue.offset = 0;
			info->var.blue.length = 8;
			info->var.blue.msb_right = 0;
			break;
		case 16:
			info->var.red.offset = 11;
			info->var.red.length = 5;
			info->var.red.msb_right = 0;
			info->var.green.offset = 5;
			info->var.green.length = 6;
			info->var.green.msb_right = 0;
			info->var.blue.offset = 0;
			info->var.blue.length = 5;
			info->var.blue.msb_right = 0;
			break;
		default:
			info->var.red.offset = 11;
			info->var.red.length = 5;
			info->var.red.msb_right = 0;
			info->var.green.offset = 5;
			info->var.green.length = 6;
			info->var.green.msb_right = 0;
			info->var.blue.offset = 0;
			info->var.blue.length = 5;
			info->var.blue.msb_right = 0;
			break;			
	}		

	r = fb_alloc_cmap(&info->cmap, 16, 0);
	info->pseudo_palette = PP;

	PP[0] = 0;
	for (r = 1; r < 16; r++)
		PP[r] = 0xffffffff;

#ifdef CONFIG_ANDROID_POWER
	INIT_WORK(&par->resume_work, power_on_panel);
#endif

	par->black = kmalloc(2*par->fb_info->var.xres, GFP_KERNEL);
	INIT_WORK(&par->resume_work, power_on_panel);
	memset(par->black, 0, 2*par->fb_info->var.xres);

	r = register_framebuffer(info);

	if (r)
		return r;

#ifdef CONFIG_FB_MSM_LOGO
        if (!load_565rle_image( INIT_IMAGE_FILE )) {
                /* Flip buffer */
                par->update_info.left = 0;
                par->update_info.top = 0;
                par->update_info.eright = info->var.xres;
                par->update_info.ebottom = info->var.yres;
                msmfb_pan_update( info, 0, 0, info->var.xres, info->var.yres,
                                    0, 1 );
        }
#endif

#ifdef CONFIG_ANDROID_POWER
	par->slightly_earlier_suspend.suspend = msmfb_slightly_earlier_suspend;
	par->slightly_earlier_suspend.resume = msmfb_slightly_later_resume;
	par->slightly_earlier_suspend.level =
		ANDROID_EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	android_register_early_suspend(&par->slightly_earlier_suspend);
	par->early_suspend.suspend = msmfb_early_suspend;
	par->early_suspend.resume = msmfb_early_resume;
	par->early_suspend.level = ANDROID_EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	android_register_early_suspend(&par->early_suspend);
#endif

#if MSMFB_DEBUG
	debugfs_create_file("msm_fb", S_IFREG | S_IRUGO, NULL,
			    (void *)info->par, &debug_fops);
#endif

	mutex_init(&par->autoupdate_lock);
	mutex_init(&par->videoupdate_lock);
	par->autoupdate = 0;

	par->last_addr = -1;

	// Start a thread that will keep updating the display 30 times per second
	par->task = kthread_run(msmfb_constant_update, info, "msm_fb");
	if (IS_ERR(par->task)) 
	{
		printk(KERN_INFO "msm_fb: failed to create update thread!\n");
	}

	// register display on callback
	if(pi->panel_ops->display_on)
	{
		par->displayon_callback.func = msmfb_display_on;
		pi->panel_ops->display_on(pi, &par->displayon_callback);
	}

	// register display off callback
	if(pi->panel_ops->display_off)
	{
		par->displayoff_callback.func = msmfb_display_off;
		pi->panel_ops->display_off(pi, &par->displayoff_callback);
	}

	INIT_WORK(&par->push_video_work, msmfb_push_frame);

	par->sleeping = AWAKE;

	return 0;
}

static int msmfb_remove(struct platform_device *pdev)
{
	return 0;
}



static struct platform_driver mddi_panel_driver = {
	.probe   = msmfb_probe,
	.remove  = msmfb_remove,
	.suspend	= msmfb_suspend,
	.resume = msmfb_resume,
	.driver  = { .name = "mddi_panel" },
};

extern int mdp_init(void);

static int __init msmfb_init(void)
{
	int ret;

	ret = mdp_init();
	if (ret)
		return ret;

	return platform_driver_register(&mddi_panel_driver);
}

module_init(msmfb_init);
