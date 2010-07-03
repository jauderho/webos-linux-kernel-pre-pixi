/*
 *  linux/drivers/media/video/msm/msm_v4l2_video.c - MSM V4L2 Video Out
 *
 *  Copyright (C) 2008 Palm Inc,
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author: Kevin McCray (kevin.mccray@palm.com)
 *
 */

#include <linux/init.h>

#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/platform_device.h>
#include <linux/android_pmem.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/msm_mdp.h>

#include <linux/time.h>

#include <media/v4l2-dev.h>
#include <media/videobuf-dma-sg.h>

#include <asm/arch/board.h>
#include <asm/arch/msm_fb.h>

#include <linux/msm_v4l2_video.h>

#define MSM_VIDEO -1
#define ROTATION_BUFFER	(vout->max_internal_bufs - 1)

#define MSMV4L2_DEBUG_MSGS 0
#if MSMV4L2_DEBUG_MSGS
#define DLOG(fmt,args...) \
	do { printk(KERN_INFO "[%s:%s:%d] "fmt, __FILE__, __func__, __LINE__, \
		    ##args); } \
	while (0)
#else
#define DLOG(x...) do {} while (0)
#endif

#define  MSMV4L2_DEBUG_PROFILING   0
#if MSMV4L2_DEBUG_PROFILING
static suseconds_t msmv4l2_stTime, msmv4l2_cbst;
#define MSMV4L2_PROFILE_START \
	{ \
		struct timeval msmv4l2_time; \
		do_gettimeofday(&msmv4l2_time);\
		msmv4l2_stTime = msmv4l2_time.tv_usec; \
	}

#define MSMV4L2_PROFILE_END(TEXT) \
	{ \
		unsigned long x; \
		struct timeval msmv4l2_time; \
		do_gettimeofday(&msmv4l2_time);\
		x = msmv4l2_time.tv_usec - msmv4l2_stTime; \
		printk(KERN_INFO "%s  %lu ms\n", TEXT, x/1000);\
		msmv4l2_stTime = msmv4l2_time.tv_usec; \
	}
#else
#define MSMV4L2_PROFILE_START
#define MSMV4L2_PROFILE_END(TEXT)
#endif

static struct msmv4l2_device 	*saved_vout;

int msmv4l2_pmem_allocate(char *name, int buffersize, struct pmem_device* buffer)
{
	int			ret;
	int 		pmemindex;
	struct file 	*filp 		= NULL;
	
	filp = filp_open(name, O_RDWR | O_LARGEFILE, 0);
	if(!filp){
		printk(KERN_ERR "msmv4l2: msmv4l2_pmem_allocate: filp_open failed!\n");
		goto end;
	}

	pmemindex = pmem_allocate(MINOR(filp->f_dentry->d_inode->i_rdev), buffersize);
	if(pmemindex < 0) {
		printk(KERN_ERR "msmv4l2: msmv4l2_pmem_allocate: pmem_allocate failed\n");
		ret = -ENOMEM;
		goto end;
	}

	// since i'm calling pmem_allocate directly i need to update the index myself
	// otherwise pmem won't remember this space was already allocated when we mmap
	update_pmem_index(filp, pmemindex);

	buffer->filp = filp;
	pmem_get_size(&buffer->region, filp);
	buffer->allocated = PMEM_ALLOCATED;

	return 0;

end:
	if(filp) filp_close(filp, NULL);
	return ret;
}


int msmv4l2_pmem_free(struct pmem_device* buffer)
{
	int ret = 0;

	buffer->allocated = PMEM_FREE;

	// Close the file
	filp_close(buffer->filp, NULL);
	buffer->filp = NULL;

	return ret;
}

static int
msmv4l2_free_overlay_buffers(struct msmv4l2_fh* fh)
{
	struct msmv4l2_device *vout = fh->vout;
	int i = vout->max_internal_bufs;

	DLOG("msmv4l2_free_overlay_buffers\n");

	while(i) {
		i--;
		if(PMEM_ALLOCATED == vout->ovlbufs[i].allocated) {
			DLOG("Address 0x%lx, size = 0x%lx\n",
					vout->ovlbufs[i].region.offset,
					vout->ovlbufs[i].region.len);
			msmv4l2_pmem_free(&vout->ovlbufs[i]);
		}
	}

	return 0;
}

static int
msmv4l2_allocate_overlay_buffers(struct msmv4l2_fh* fh)
{
	struct msmv4l2_device *vout = fh->vout;
	int i, ret;

	DLOG("msmv4l2_allocate_overlay_buffers\n");
	
	for(i = 0; i < vout->max_internal_bufs; i++)
	{	
		// open internal pmem devices for post processed buffer storage
		ret = msmv4l2_pmem_allocate("/dev/pmem_overlay",
										vout->screen_width * vout->screen_height * 3, // RGB888 format, 3bpp
										&vout->ovlbufs[i]);
		if (ret){
			ret = -ENOMEM;
			printk (KERN_ERR "msmv4l2: msmv4l2_allocate_overlay_buffers failed\n");
			goto end;
		}

		DLOG("Address 0x%lx, size = 0x%lx\n",
				vout->ovlbufs[i].region.offset,
				vout->ovlbufs[i].region.len);
	}

	return 0;

end:
	msmv4l2_free_overlay_buffers(fh);
	return ret;
}

static int
msmv4l2_startstreaming(struct msmv4l2_device *vout)
{
	spin_lock(&vout->qbufcount_lock);
	vout->qbuf_count = 0;
	spin_unlock(&vout->qbufcount_lock);
	
	vout->stop_q_flag = 0;
	vout->current_buffer = 0;

	// Clear intermediate rotation buffer prior to streaming
	// this ensures that when video doesn't cover the whole screen
	// it will have black background.
	memset(phys_to_virt(vout->ovlbufs[ROTATION_BUFFER].region.offset),
			0x0,
			vout->ovlbufs[ROTATION_BUFFER].region.len);


	msmfb_switch_pipeline(PIPELINE_VIDEO);
	vout->streaming = 1;

	return 0;
}

static int
msmv4l2_stopstreaming(struct msmv4l2_device *vout)
{
	if (!vout->streaming)
		return 0;

	vout->stop_q_flag = 1;

	DLOG("Waiting for qbuf operation to be finished. qbufcount = %d \n", vout->qbuf_count);

	wait_event_interruptible_timeout(vout->msmv4l2_wq, vout->qbuf_count != 0, msecs_to_jiffies(500));

	DLOG("All qbuf operations done. qbufcount = %d \n", vout->qbuf_count);

	msmfb_switch_pipeline(PIPELINE_GRAPHICS);
	vout->streaming = 0;

	return 0;
}

static int 
msmv4l2_vidioc_reqbufs(struct file *file, struct msmv4l2_fh *fh, struct v4l2_requestbuffers *req)
{
	DLOG("VIDIOC_REQBUFS\n");

	if (req->memory != V4L2_MEMORY_USERPTR)
		return -EINVAL;

	return 0;
}

void 
msmv4l2_callback(struct msmfb_callback* callback)
{
	struct msmv4l2_device *vout =
			container_of(callback, struct msmv4l2_device, user_callback);

	spin_lock(&vout->qbufcount_lock);
	vout->qbuf_count--;
	spin_unlock(&vout->qbufcount_lock);

	if(vout->stop_q_flag == 1)
		wake_up_interruptible(&vout->msmv4l2_wq);

	if(vout->qbuf_count < 0)
		DLOG("msmv4l2: msmv4l2_callback: count [%d] cannot be less than zero!\n", vout->qbuf_count);

	DLOG("msmv4l2_callback: Display updated , qbuf count = %d\n", vout->qbuf_count);
}

static int
msmv4l2_mapformat(uint32_t pixelformat)
{
	int mdp_format;

	switch(pixelformat) {
		case V4L2_PIX_FMT_RGB565:
			mdp_format = MDP_RGB_565;
			break;
		case V4L2_PIX_FMT_RGB32:
			mdp_format = MDP_ARGB_8888;
			break;
		case V4L2_PIX_FMT_RGB24:
			mdp_format = MDP_RGB_888;
			break;
		case V4L2_PIX_FMT_NV12:
			mdp_format = MDP_Y_CBCR_H2V2;
			break;
		case V4L2_PIX_FMT_NV21:
			mdp_format = MDP_Y_CRCB_H2V2;
			break;
		default:	
			mdp_format = MDP_Y_CBCR_H2V2;	
			break;
	}	

	return mdp_format;
}

static int
msmv4l2_get_buffer_pmem_offset(struct v4l2_buffer *buffer, unsigned long* offset)
{
	int rc = 0;

	struct vm_area_struct *vma;

	down_read(&current->mm->mmap_sem);

	if (!(vma = find_vma(current->mm, buffer->m.userptr))
		|| (vma->vm_start > buffer->m.userptr)
		|| (vma->vm_end < buffer->m.userptr)) {
		rc = -EINVAL;
		goto exit;
	}

	*offset = buffer->m.userptr - vma->vm_start;

	DLOG("userptr=0x%X vm_start=0x%X vm_end=0x%X -> offset=0x%X\n",
		buffer->m.userptr, vma->vm_start, vma->vm_end, *offset);

exit:
 	up_read(&current->mm->mmap_sem);

	return rc;
}

static int 
msmv4l2_overlay_stage_one(struct msmv4l2_device *vout, struct v4l2_buffer *buffer)
{
	int ret;
	unsigned long offset;

	MSMV4L2_PROFILE_START;
	memset (&vout->req, 0, sizeof(struct mdp_blit_int_req));

	vout->req.src.width = vout->pix.width;
	vout->req.src.height = vout->pix.height;

	vout->req.src_rect.x = vout->crop_rect.left;
	vout->req.src_rect.y = vout->crop_rect.top;
	vout->req.src_rect.w = vout->crop_rect.width;
	vout->req.src_rect.h = vout->crop_rect.height;

	vout->req.src.format = msmv4l2_mapformat(vout->pix.pixelformat);
	msmv4l2_get_buffer_pmem_offset(buffer, &offset);
	vout->req.src.offset = offset;

	int put_needed;	
	struct file *f = fget_light(buffer->reserved, &put_needed);
	vout->req.src.filp = f;
	fput_light(f, put_needed);

	vout->req.dst.width = vout->screen_width;
	vout->req.dst.height = vout->screen_height; 

	if ((vout->rotation == MDP_ROT_90) || (vout->rotation == MDP_ROT_270)) {
		vout->req.dst_rect.x = vout->win.w.top;
		vout->req.dst_rect.y = vout->win.w.left;
		vout->req.dst_rect.w = vout->win.w.height;
		vout->req.dst_rect.h = vout->win.w.width;
	}
	else {
		vout->req.dst_rect.x = vout->win.w.left;
		vout->req.dst_rect.y = vout->win.w.top;
		vout->req.dst_rect.w = vout->win.w.width;
		vout->req.dst_rect.h = vout->win.w.height;
	}

	vout->req.dst.format = MDP_RGB_888;
	//vout->req.dst.offset = 0;
	vout->req.dst.filp = vout->ovlbufs[ROTATION_BUFFER].filp;

	vout->req.alpha = MDP_ALPHA_NOP;
	vout->req.transp_mask = MDP_TRANSP_NOP;
	vout->req.flags = vout->rotation;

	DLOG("pass 1: SRC: width=%d, height=%d, filp=0x%x offset=0x%x\n", vout->req.src.width, vout->req.src.height, vout->req.src.filp, vout->req.src.offset);
	DLOG("pass 1: DST: width=%d, height=%d, filp=0x%x\n", vout->req.dst.width, vout->req.dst.height, vout->req.dst.filp);

	mutex_lock(&vout->frameupdate_lock);
	ret = msmfb_overlay(&vout->req, 0, &vout->user_callback);
	mutex_unlock(&vout->frameupdate_lock);
	
	MSMV4L2_PROFILE_END("Time taken for pass 1: ");

	return ret;
}

static int 
msmv4l2_vidioc_qbuf(struct file *file, struct msmv4l2_fh* fh, void *arg)
{
	struct msmv4l2_device *vout = fh->vout;
	struct v4l2_buffer *buffer = (struct v4l2_buffer *) arg;
	int ret;

	DLOG("VIDIOC_QBUF: buffer=0x%X, fd=%d\n", buffer->m.userptr, buffer->reserved);

	if(!vout->streaming) {
		printk(KERN_ERR "msmv4l2: Call VIDIOC_STREAMON first\n");
		return -EINVAL;
	}

	if ((buffer->type != V4L2_BUF_TYPE_VIDEO_OVERLAY) && (buffer->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)){
		printk(KERN_ERR "msmv4l2: VIDIOC_QBUF: invalid buffer type.\n");
		return -EINVAL;
	}

	if(vout->stop_q_flag == 1) {
		printk(KERN_ERR "msmv4l2: VIDIOC_QBUF: stop q flag is set.\n");
		return 0;
	}

	spin_lock(&vout->qbufcount_lock);
	vout->qbuf_count++;
	spin_unlock(&vout->qbufcount_lock);

	DLOG("Start: VIDIOC_QBUF: qbuf count=%d\n", vout->qbuf_count);

	mutex_lock(&vout->pipeline_lock);

	
	DLOG("VIDIOC_QBUF: buffer=0x%X, fd=%d\n", buffer->m.userptr, buffer->reserved);

	ret =  msmv4l2_overlay_stage_one( vout, buffer );
	if(ret < 0) {
		DLOG("msmv4l2: msmv4l2_overlay_stage_one failed.\n");
		goto end;
	}

	MSMV4L2_PROFILE_START;

	// Blit request
	memset (&vout->req, 0, sizeof(struct mdp_blit_int_req));
	
	vout->req.bg_rect.w 	= vout->req.bg.width  = vout->screen_width;
	vout->req.bg_rect.h 	= vout->req.bg.height = vout->screen_height;
	vout->req.bg.format 	= MDP_ARGB_8888;

	vout->req.bg.filp	= -1; 

	vout->req.src_rect.w 	= vout->screen_width;
	vout->req.src_rect.h 	= vout->screen_height;
	vout->req.src.width 	= vout->screen_width;
	vout->req.src.height 	= vout->screen_height;
	vout->req.src.format 	= MDP_RGB_888;
	vout->req.src.filp 		= vout->ovlbufs[ROTATION_BUFFER].filp;


	vout->req.dst_rect.w 	= vout->req.dst.width = vout->screen_width;
	vout->req.dst_rect.h 	= vout->req.dst.height = vout->screen_height;
	vout->req.dst.format 	= MDP_RGB_888;
	vout->req.dst.filp =  vout->ovlbufs[vout->current_buffer].filp;

	vout->req.alpha = MDP_ALPHA_NOP;
	vout->req.transp_mask = MDP_TRANSP_NOP;
	vout->req.flags = MDP_ROT_NOP;

	DLOG("pass 2: src.width=%d, src.height=%d,dst.width=%d, dst.height=%d, src filp=0x%x, dest filp=0x%x,\n bg.width=%d, bg.height=%d, bg filp=0x%x\n", vout->req.src.width, vout->req.src.height, vout->req.dst.width, vout->req.dst.height, vout->req.src.filp, vout->req.dst.filp, vout->req.bg.width, vout->req.bg.height, vout->req.bg.filp );

	ret = msmfb_overlay(&vout->req, vout->ovlbufs[vout->current_buffer].region.offset, &vout->user_callback);
	if(ret < 0) {
		DLOG("pass 2 msmfb_overlay failed\n");
	}
	
	vout->current_buffer = (vout->current_buffer + 1) % (vout->max_internal_bufs - 1)

	MSMV4L2_PROFILE_END("Time taken for pass 2: ");

end:
	mutex_unlock(&vout->pipeline_lock);
	if(ret < 0) {
		vout->qbuf_count--;
		if(vout->stop_q_flag == 1) wake_up_interruptible(&vout->msmv4l2_wq);
	}
	return ret;
}


static int
msmv4l2_do_ioctl (struct inode *inode, struct file *file,
		       unsigned int cmd, void *arg)
{
	struct msmv4l2_fh *fh = (struct msmv4l2_fh *)file->private_data;
	struct msmv4l2_device *vout = fh->vout;

	DLOG("msmv4l2_do_ioctl: cmd=0x%x\n", cmd);

	switch (cmd){
	case VIDIOC_REQBUFS:
		return msmv4l2_vidioc_reqbufs(file, fh, arg);

	case VIDIOC_QBUF:
		return msmv4l2_vidioc_qbuf(file, fh, arg);
	
	case VIDIOC_DQBUF:
		DLOG("VIDIOC_DQBUF\n");
		break;

	case VIDIOC_S_FMT: {
		struct v4l2_format *f = (struct v4l2_format *) arg;
		DLOG("VIDIOC_S_FMT\n");

		if(vout->streaming)
			return -EBUSY;

		switch(f->type){
			case V4L2_BUF_TYPE_VIDEO_OVERLAY: {
				memcpy(&vout->win, &f->fmt.win, sizeof(struct v4l2_window));
				DLOG("Overlay Window: left=%d, top=%d, width=%d, height=%d\n",
					vout->win.w.left, vout->win.w.top, vout->win.w.width,
					vout->win.w.height);
				break;
			}
			case V4L2_BUF_TYPE_VIDEO_OUTPUT: {				
				memcpy(&vout->pix, &f->fmt.pix, sizeof(struct v4l2_pix_format));
				DLOG("Pixel Data: format=%d, width=%d, height=%d\n",
					vout->pix.pixelformat, vout->pix.width, vout->pix.height);
		
				break;
			}
			default:
				return -EINVAL;
		}
		break;
	}
	case VIDIOC_G_FMT: {
		struct v4l2_format *f = (struct v4l2_format *) arg;
		DLOG("VIDIOC_G_FMT\n");

		switch (f->type){
			case V4L2_BUF_TYPE_VIDEO_OUTPUT: {
				struct v4l2_pix_format *pix = &f->fmt.pix;
				memset (pix, 0, sizeof (*pix));
				*pix = vout->pix;
				break;
			}

			case V4L2_BUF_TYPE_VIDEO_OVERLAY: {
				struct v4l2_window *win = &f->fmt.win;
				memset (win, 0, sizeof (*win));
				win->w = vout->win.w;
				break;
			}
			default:
				return -EINVAL;
		}
		break;
	}

	case VIDIOC_S_CROP: {
		struct v4l2_crop *crop = (struct v4l2_crop *) arg;
		DLOG("VIDIOC_S_CROP\n");

		switch(crop->type) {
			case V4L2_BUF_TYPE_VIDEO_OUTPUT:
				memcpy(&vout->crop_rect, &crop->c, sizeof(struct v4l2_rect));

				DLOG("New Crop Rect: left=%d, top=%d, width=%d, height=%d\n",
					vout->crop_rect.left, vout->crop_rect.top, vout->crop_rect.width, vout->crop_rect.height);
				break;
			default:
				return -EINVAL;
		}/* switch */
		break;
	}
	case VIDIOC_G_CROP: {
		struct v4l2_crop *crop = (struct v4l2_crop *) arg;
		DLOG("VIDIOC_G_CROP\n");
		
		switch (crop->type) {
			case V4L2_BUF_TYPE_VIDEO_OUTPUT:
				memcpy(&crop->c, &vout->crop_rect, sizeof(struct v4l2_rect));
				break;
			default:
				return -EINVAL;
		}/* switch */
		break;
	}
	case VIDIOC_STREAMON: {
		int 	*ptype 	= arg;
		DLOG("VIDIOC_STREAMON\n");

		if(vout->streaming) {
			printk(KERN_ERR "msmv4l2: VIDIOC_STREAMON: already streaming.\n");
			return -EBUSY;
		}

		if ((*ptype != V4L2_BUF_TYPE_VIDEO_OVERLAY) && (*ptype != V4L2_BUF_TYPE_VIDEO_OUTPUT)){
			printk(KERN_ERR "msmv4l2: VIDIOC_STREAMON: invalid buffer type.\n");
			return -EINVAL;
		}

		msmv4l2_startstreaming(vout);
		break;
	}

	case VIDIOC_STREAMOFF: {
		DLOG("VIDIOC_STREAMOFF\n");

		if(!vout->streaming) {
			printk(KERN_ERR "msmv4l2: VIDIOC_STREAMOFF: Invalid IOCTL request.\n");
			return -EINVAL;
		}

		msmv4l2_stopstreaming( vout );
		break;
	}

	case VIDIOC_S_MSM_ROTATION: {
		int *rotation = arg;
		//DLOG("VIDIOC_MSM_S_ROTATION=%d\n", *rotation);
		printk(KERN_INFO"VIDIOC_MSM_S_ROTATION=%d\n", *rotation);

		switch(*rotation) {
			case 0:
				vout->rotation = MDP_ROT_NOP;
				break;
			case 90:
				vout->rotation = MDP_ROT_90;
				break;
			case 180:
				vout->rotation = MDP_ROT_180;
				break;
			case 270:
				vout->rotation = MDP_ROT_270;
				break;
			default:
				printk(KERN_ERR "msmv4l2: VIDIOC_S_MSM_ROTATION: Bad rotation value\n");
				return -EINVAL;
		}/* switch */
		break;
	}

	case VIDIOC_G_MSM_ROTATION: {
		int *rotation = arg;
		DLOG("VIDIOC_MSM_G_ROTATION\n");	

		switch(vout->rotation) {
			case MDP_ROT_NOP:
				*rotation = 0;
				break;
			case MDP_ROT_90:
				*rotation = 90;
				break;
			case MDP_ROT_180:
				*rotation = 180;
				break;
			case MDP_ROT_270:
				*rotation = 270;
				break;
			default:
				printk(KERN_ERR "msmv4l2: VIDIOC_G_MSM_ROTATION: Bad rotation value\n");
				return -EINVAL;
		}/* switch */
		break;
	}

	case VIDIOC_S_MSM_FB_INFO: {
		DLOG("VIDIOC_S_MSM_FB_INFO deprecated\n");
	}
	break;
	case VIDIOC_G_MSM_FB_INFO: {
		int *fd = arg;
		DLOG("VIDIOC_G_MSM_FB_INFO deprecated\n");
		*fd = -1;
	}
	break;

	case VIDIOC_G_LAST_FRAME: {

		struct v4l2_last_frame *frame_info = (struct v4l2_last_frame*)arg;

		DLOG("VIDIOC_G_LAST_FRAME\n");

		if(!frame_info->buffer)
		{
			/*
			 * First call with NULL buffer would return
		 	 * the width, heigh and size.
			 * Caller should then allocate a buffer based on the
			 * returned size and call this IOCTL again to
			 * get newly allocated buffer filled
			 * Note: 3 bytes/pixel for RGB format
			 */
			frame_info->screen_width = vout->screen_width;
			frame_info->screen_height = vout->screen_height;
			frame_info->size = vout->screen_width*vout->screen_height*3;

			DLOG("VIDIOC_G_LAST_FRAME  w: %d, h: %d, size: %d\n",
					frame_info->screen_width,
					frame_info->screen_height,
					frame_info->size);
		}
		else
		{
			/*
			 * Second call with allocated buffer returns
			 * filled buffer of the last rotated frame
			 */

			unsigned long paddr, vaddr, len;

			
			get_pmem_addr(vout->ovlbufs[ROTATION_BUFFER].filp, &paddr, &vaddr, &len);

			DLOG("VIDIOC_G_LAST_FRAME p:%x v:%x l:%x\n", paddr, vaddr, len);
            

			mutex_lock(&vout->frameupdate_lock);
			if(copy_to_user((void __user *)frame_info->buffer,
							(void *)vaddr, frame_info->size))
			{
                                return -EFAULT;
			}
			mutex_unlock(&vout->frameupdate_lock);
		}
	}
	break;

	default:
		DLOG("Unrecognized IOCTL\n");
		return -ENOIOCTLCMD;

	}/* switch */

	return 0;
}

static int
msmv4l2_ioctl (struct inode *inode, struct file *file, unsigned int cmd,
		    unsigned long arg)
{
	return video_usercopy (inode, file, cmd, arg, msmv4l2_do_ioctl);
}

int
msmv4l2_release(struct inode *inode, struct file *file)
{
	struct msmv4l2_fh *fh = file->private_data;
	struct msmv4l2_device *vout = fh->vout;
	int ret;

	if(vout->streaming)
 		msmv4l2_stopstreaming( vout );

	ret = msmv4l2_free_overlay_buffers(fh);

	vout->ref_count--;

	kfree(fh);

	return ret;
}

int
msmv4l2_open (struct inode *inode, struct file *file)
{
	struct msmv4l2_device 	*vout = saved_vout;
	struct v4l2_pix_format	*pix;
	struct msmv4l2_fh *fh;

	DLOG("msmv4l2_open+++\n");

	if (vout->ref_count) {
		printk (KERN_ERR "msmv4l2_open: multiple open currently is not supported!\n");
		return -EBUSY;
	}

	// Increment reference count
	vout->ref_count++;

	/* allocate per-filehandle data */
	fh = kmalloc (sizeof(struct msmv4l2_fh), GFP_KERNEL);
	if (NULL == fh) {
		printk (KERN_ERR "msmv4l2_open: kmalloc failed\n");
		return -ENOMEM;
	}

	fh->vout = vout;
	fh->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;

	if (msmv4l2_allocate_overlay_buffers(fh) < 0) {
		printk (KERN_ERR "msmv4l2_open: msmv4l2_allocate_overlay_buffers failed\n");
		kfree(fh);
		return -ENOMEM;
	}

	file->private_data = fh;

	vout->streaming 	= 0;
	vout->rotation 		= MDP_ROT_NOP;
	vout->crop_rect.left 	= vout->crop_rect.top = 0;
	vout->crop_rect.width 	= vout->screen_width;
	vout->crop_rect.height 	= vout->screen_height;
	vout->user_callback.func = msmv4l2_callback;

	pix 			= &vout->pix;
	pix->width 		= vout->screen_width;
	pix->height 		= vout->screen_height;
	pix->pixelformat 	= V4L2_PIX_FMT_RGB32;// KM TODO: need the right YUV format
	pix->field 		= V4L2_FIELD_NONE;
	pix->bytesperline 	= pix->width * 4;// KM TODO: update this to match the pixel format
	pix->sizeimage 		= pix->bytesperline * pix->height;
	pix->priv 		= 0;
	pix->colorspace 	= V4L2_COLORSPACE_SRGB;

	vout->win.w.left 	= 0;
	vout->win.w.top 	= 0;
	vout->win.w.width 	= vout->screen_width;
	vout->win.w.height 	= vout->screen_height;

	mutex_init(&vout->pipeline_lock);
	mutex_init(&vout->frameupdate_lock);

	init_waitqueue_head(&vout->msmv4l2_wq);

	spin_lock_init(&vout->qbufcount_lock);

	return 0;
}

static int 
msmv4l2_probe(struct platform_device *pdev)
{	
	struct msmv4l2_device 	*vout 	= saved_vout;
	struct msm_v4l2_pd 	*pi 	= pdev->dev.platform_data;
	int ret;

	DLOG("msmv4l2_probe+++\n");

	vout->screen_width 		= pi->screen_width;
	vout->screen_height 	= pi->screen_height;
	vout->max_internal_bufs = pi->max_internal_bufs;
	vout->scaling_factor	= pi->scaling_factor;

	vout->ovlbufs = kmalloc(sizeof (struct pmem_device) * vout->max_internal_bufs, GFP_KERNEL);
	if (!vout->ovlbufs){
		printk (KERN_ERR "msmv4l2_probe: kmalloc failed\n");		
		ret = -ENOMEM;
		goto end;
	}

	memset(vout->ovlbufs, 0, sizeof (struct pmem_device) * vout->max_internal_bufs);

	return 0;

end:
	return ret;
}

static int 
msmv4l2_remove(struct platform_device *pdev)
{
	struct msmv4l2_device *vout = saved_vout;

	DLOG("msmv4l2_remove+++\n");

	if(vout->ovlbufs) kfree(vout->ovlbufs);

	return 0;
}

void msmv4l2_videodev_release(struct video_device *vfd)
{
	DLOG("msmv4l2_videodev_release+++\n");
	return;
}

static const struct file_operations msmv4l2_fops = {
	.owner		= THIS_MODULE,
	.open		= msmv4l2_open,
	.release	= msmv4l2_release,
	.ioctl		= msmv4l2_ioctl,
};

static struct video_device msmv4l2_vid_device = {
	.name		= "msmv4l2",
	.type2		= VID_TYPE_OVERLAY,
	.fops       = &msmv4l2_fops,
	.minor		= -1,
	.release	= msmv4l2_videodev_release,
};

static struct platform_driver msmv4l2_platform_driver = {
	.probe   = msmv4l2_probe,
	.remove  = msmv4l2_remove,
	.driver  = {
			 .name = "msmv4l2_pd",
		   },
};

static int __init
msmv4l2_init (void)
{
	int 			ret;
	struct msmv4l2_device 	*vout;

	DLOG("msmv4l2_init+++\n");

	vout = kmalloc (sizeof (struct msmv4l2_device), GFP_KERNEL);
	if (!vout){
		printk (KERN_ERR "msmv4l2: kmalloc failed\n");
		return -ENOMEM;
	}

	memset (vout, 0, sizeof (struct msmv4l2_device));
	saved_vout = vout;

	ret = platform_driver_register(&msmv4l2_platform_driver);
	if(ret < 0) {
		printk (KERN_ERR "msmv4l2: platform_driver_register failed\n");
		goto end;
	}

	//Register the device with videodev. Videodev will make IOCTL calls on application requests
	ret = video_register_device (&msmv4l2_vid_device, VFL_TYPE_GRABBER, MSM_VIDEO);
	if (ret < 0) {
		printk (KERN_ERR "msmv4l2: could not register Video for Linux device\n");
		goto end_unregister;
	}	
	
	return 0;

end_unregister:
	platform_driver_unregister(&msmv4l2_platform_driver);

end:
	kfree(vout);
	saved_vout = NULL;
	return ret;
}

static void
msmv4l2_exit (void)
{
	struct msmv4l2_device *vout = saved_vout;

	DLOG("mmsv4l2_exit+++\n");

	video_unregister_device(&msmv4l2_vid_device);

	platform_driver_unregister(&msmv4l2_platform_driver);

	if (vout) kfree(vout);
	saved_vout = NULL;
	return;
}

module_init (msmv4l2_init);
module_exit (msmv4l2_exit);

MODULE_AUTHOR ("Palm");
MODULE_DESCRIPTION ("MSM V4L2 Driver");
MODULE_LICENSE ("GPL");

