/****************************************************************************
 * drivers/video/video.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <sdk/config.h>

#include <sys/ioctl.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#include <arch/chip/cisif.h>
#include <arch/board/board.h>

#include <pthread.h>
#include <semaphore.h>
#include <mqueue.h>

#include <time.h>

#include "video/video.h"
#include "video/video_halif.h"
#include "nuttx/video/isx012.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Display time ON/OFF */
/* #define VIDEO_TIME_MEASURE */

/* At initialization, it automatically transits to ACTIVE_MODE */
/* #define VIDEO_INIT_ACTIVE */

#define video_printf(format, ...)   _info(format, ##__VA_ARGS__)

#define VIDEO_TRUE              (1)
#define VIDEO_FALSE             (0)

#define VIDEO_EOI_CORRECT_MAX_SIZE  (32)

#define VIDEO_INIT_REGNUM       (5)
#define VIDEO_AE_AUTO_REGNUM    (6)
#define VIDEO_AE_NOW_REGNUM     (6)
#define VIDEO_AWB_AUTO_REGNUM   (3)
#define VIDEO_AWB_NOW_REGNUM    (3)

#define VIDEO_ISX012_HALFREL_TIMEOUT      (2*1000*1000)   /* usec */
#define VIDEO_ISX012_HALFREL_WAITTIME     (20*1000)       /* usec */
#define VIDEO_ISX012_HALF_MOVE_STS_REG    (0x01B0)
#define VIDEO_ISX012_MOVE_AWB_F           (0x01)
#define VIDEO_ISX012_MOVE_AE_F            (0x02)
#define VIDEO_CISIF_TRANSEND_TIMEOUT      (1*1000)        /* msec */



#define VIDEO_BUF_STATUS_NOTRIGGER (0)
#define VIDEO_BUF_STATUS_TRIGGERED (1)

/* Debug option */
#ifdef CONFIG_DEBUG_VIDEO_ERROR
#  define videoerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define videoerr(x...) 
#endif

#ifdef CONFIG_DEBUG_VIDEO_WARN
#  define videowarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define videowarn(x...)
#endif

#ifdef CONFIG_DEBUG_VIDEO_INFO
#  define videoinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define videoinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct video_buf_inf_s
{
  struct v4l2_buffer         buf;    /* Buffer information */
  FAR struct video_buf_inf_s *next;  /* pointer to next buffer */
};

struct video_mng_per_buftype_s
{
  sem_t                      exclbuf;    /* for buffer */
  sem_t                      exclfmt;    /* for ioctl(FMT系) */
  uint8_t                    mode;       /* Buffer mode(FIFO or RING) */
  uint8_t                    bufnum;     /* number of enqueued buffer */
  uint8_t                    bufnum_max; /* count of VIDIOC_REQBUFS */
  uint8_t                    status;     /* VIDEO_BUF_STATUS_NOTRIGGER,
                                            VIDEO_BUF_STATUS_TRIGGERED */ 
  uint8_t                    interval;   /* capture interval(frame) */
  FAR sem_t                  *dqbuf;     /* dequeue requested */
  FAR struct video_buf_inf_s *dequeued;  /* pointer to dequeued buffer */
  FAR struct video_buf_inf_s *latest;    /* pointer to latest buffer */
  FAR struct video_buf_inf_s *input;     /* pointer to input buffer */

};

struct video_mng_s
{
  FAR char                       *devpath; /* parameter of video_register() */
  uint8_t                        open_num;     /* open回数を覚えておく */
  sem_t                          excldev;      /* for open, close */
  sem_t                          exclctrl;     /* for ioctl(CTRL系) */
  sem_t                          exclhalfpush; /* for ioctl(DO_HALFPUSH) */
  FAR struct pollfd              *poll;      /* poll(setup) information */
  struct video_mng_per_buftype_s video; /* VIDEO_CAPTURE resource */
  struct video_mng_per_buftype_s still; /* STILL_CAPTURE resource */
};

typedef struct video_mng_s video_mng_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods. */

static int video_open(FAR struct file *filep);
static int video_close(FAR struct file *filep);
static int video_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int video_poll(FAR struct file   *filep,
                      FAR struct pollfd *fds,
                      bool              setup);
static int video_poll_setup(FAR struct video_mng_s *priv,
                            FAR struct pollfd *fds);
static int video_poll_teardown(FAR struct video_mng_s *priv,
                               FAR struct pollfd *fds);

/* Common function */

static int video_lock(FAR sem_t *sem);
static int video_unlock(FAR sem_t *sem);
static FAR struct video_mng_per_buftype_s *check_buf_type
           (FAR struct video_mng_s *priv, uint8_t type);


/* internal function for each cmds of ioctl */

static int video_reqbufs(FAR struct video_mng_s *priv,
                         FAR struct v4l2_requestbuffers *reqbufs);
static int video_qbuf(FAR struct video_mng_s *priv,
                      FAR struct v4l2_buffer *buf);
static int video_dqbuf(FAR struct video_mng_s *priv,
                       FAR struct v4l2_buffer *buf);
static int video_enum_fmt(FAR struct v4l2_fmtdesc *fmt);
static int video_enum_framesizes(FAR struct v4l2_frmsizeenum *frmsize);
static int video_s_fmt(FAR struct video_mng_s *priv,
                       FAR struct v4l2_format *fmt);
static int video_enum_frameintervals(FAR struct v4l2_frmivalenum *frmival);
static int video_s_parm(FAR struct video_mng_s *priv,
                        FAR struct v4l2_streamparm *parm);
static int video_streamon(FAR struct video_mng_s *priv,
                          FAR enum v4l2_buf_type *type);
static int video_do_halfpush(FAR struct video_mng_s *priv, bool enable);
static int video_takepict_start(FAR struct video_mng_s *priv,
                                uint32_t interval);
static int video_takepict_stop(FAR struct video_mng_s *priv,
                               bool halfpush);
static int video_queryctrl(FAR struct v4l2_queryctrl *ctrl);
static int video_query_ext_ctrl(FAR struct v4l2_query_ext_ctrl *ctrl);
static int video_querymenu(FAR struct v4l2_querymenu *menu);
static int video_g_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl);
static int video_s_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl);
static int video_g_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls);
static int video_s_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_video_fops =
{
  video_open,               /* open */
  video_close,              /* close */
  0,                        /* read */
  0,                        /* write */
  0,                        /* seek */
  video_ioctl,              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  video_poll,               /* poll */
#endif
  0                         /* unlink */
};


static FAR const struct video_ops_s *g_video_ops;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int video_lock(FAR sem_t *sem)
{
  int ret;
  int l_errno;

  if (sem == NULL)
    {
      return -EINVAL;
    }

  while (1)
    {
      ret = sem_wait(sem);
      if (ret == ERROR)
        {
          l_errno = errno;
          if (l_errno == EINTR)
            {
              continue;
            }

          videoerr("sem_wait() failed:%d\n", l_errno);
        }

      break;
    }

  return ret;
}

static int video_unlock(FAR sem_t *sem)
{
  if (sem == NULL)
    {
      return -EINVAL;
    }

  sem_post(sem);

  return OK;
}

static int video_open(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  video_mng_t           *priv  = inode->i_private;
  int ret = OK;

  video_lock(&priv->excldev);
  if (priv->open_num == 0)
    {
      /* Only in first execution, open device */

      ret = g_video_ops->open(priv);
    }

  /* In second or later execution, ret is initial value(=OK) */
 
  if (ret == OK)
    {
      priv->open_num++;
    }
  video_unlock(&priv->excldev);

  return ret;
}

static int video_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  video_mng_t      *priv  = inode->i_private;
  int ret = ERROR;

  video_lock(&priv->excldev);
  if (priv->open_num == 0)
    {
      return -EBUSY;
    }

  priv->open_num--;
  if (priv->open_num == 0)
    {
      g_video_ops->close();
    }
  video_unlock(&priv->excldev);

  return ret;
}

static FAR struct video_mng_per_buftype_s *check_buf_type
(FAR struct video_mng_s *priv, uint8_t type)
{
  FAR struct video_mng_per_buftype_s *video_mng;

  switch (type)
    {
      case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        video_mng = &priv->video;
        break;

      case V4L2_BUF_TYPE_STILL_CAPTURE:
        video_mng = &priv->still;
        break;

      default:  /* Error case */
        video_mng = NULL;
        break;
    }

  return video_mng;
}


static int video_reqbufs(FAR struct video_mng_s *priv,
                         FAR struct v4l2_requestbuffers *reqbufs)
{
  FAR struct video_mng_per_buftype_s *video_mng;

  if (reqbufs == NULL)
    {
      return -EINVAL;
    }

  video_mng = check_buf_type(priv, reqbufs->type);

  if (video_mng == NULL)
    {
      return -EINVAL;
    }

  if (reqbufs->memory != V4L2_MEMORY_USERPTR)
    {
      return -ENOSYS;
    }

  /* Lock buffer control */

  video_lock(&video_mng->exclbuf);

  /* Save parameter */

  video_mng->mode       = reqbufs->mode;
  video_mng->bufnum_max = reqbufs->count;

  /* Unlock buffer control */

  video_unlock(&video_mng->exclbuf);

  return OK;
}

static int video_qbuf(FAR struct video_mng_s *priv,
                      FAR struct v4l2_buffer *buf)
{
  int ret = OK;                               /* return value */
  irqstate_t flags;

  /* pointer to video_mng for specified type */

  FAR struct video_mng_per_buftype_s *video_mng;

  FAR struct video_buf_inf_s *new;            /* saving area for new buffer */

  /* Parameter NULL check */

  if ((priv == NULL) || (buf == NULL))
    {
      return -EINVAL;
    }

  video_mng = check_buf_type(priv, buf->type);
  if (video_mng == NULL)
    {
      return -EINVAL;
    }

  if (buf->memory != V4L2_MEMORY_USERPTR)
    {
      return -ENOSYS;
    }

  /* Lock buffer control */

  video_lock(&video_mng->exclbuf);

  if (video_mng->bufnum >= video_mng->bufnum_max)
    {
      /* buffer number exceed VIDIOC_REQBUFS request */

      ret = -EINVAL;
      goto qbuf_locked;
    }

  /* allocate parameter saving area and save parameter */

  new = kmm_malloc(sizeof(struct video_buf_inf_s));

  if (new == NULL)
    {
      ret = -ENOMEM;
      goto qbuf_locked;
    }

  memcpy(&new->buf, buf, sizeof(struct v4l2_buffer));

  /* Initialize image data size */

  new->buf.bytesused = 0;

  flags = enter_critical_section();

  /* Set chain information */

  if (video_mng->bufnum == 0)
    {
      /* In first buffer case, next is set to own */

      if (video_mng->mode == V4L2_BUF_MODE_RING)
        {
          /* In ring buffer mode and first buffer */

          new->next = new;
        }
      else
        {
          new->next = NULL;
        }

      video_mng->input = new;

      if (video_mng->status == VIDEO_BUF_STATUS_TRIGGERED)
        {
          g_video_ops->start_getimage
            (buf->type,
             new->buf.m.userptr,
             new->buf.length,
             video_mng->interval);
        }
    }
  else
    {
      /* In not first cases, inherit from old latest information */

      new->next = video_mng->latest->next;
    }

  video_mng->bufnum++;

  /* Update buffer chain in critical section.
   *  - Add new buffer in next position of old latest buffer
   *  - In no dequeued buffer, set dequeued = new buffer
   */

  if (video_mng->latest != NULL)
    {
      video_mng->latest->next = new;
    }

  video_mng->latest = new;

  if (video_mng->dequeued == NULL)
    {
      video_mng->dequeued = new;
    }

  leave_critical_section(flags);

qbuf_locked:
  video_unlock(&video_mng->exclbuf);

  return ret;
}

static int video_dqbuf(FAR struct video_mng_s *priv,
                       FAR struct v4l2_buffer *buf)
{
  int ret = OK;                 /* return value */
  sem_t sem;                    /* Wait for getting image data */ 
  irqstate_t flags;

  /* pointer to video_mng for specified type */

  FAR struct video_mng_per_buftype_s *video_mng;

  /* old dequeued buffer address  */

  FAR struct video_buf_inf_s *old_dequeued; /* old dequeued buffer address  */

  /* Parameter NULL check */

  if (buf == NULL)
    {
      return -EINVAL;
    }

  video_mng = check_buf_type(priv, buf->type);
  if (video_mng == NULL)
    {
      return -EINVAL;
    }

  /* Lock buffer control */

  video_lock(&video_mng->exclbuf);
  flags = enter_critical_section();

  old_dequeued = video_mng->dequeued;

  if (old_dequeued == NULL)
    {
      /* no buffer */

      ret = -EINVAL;
      goto dqbuf_locked;
    }

  if (old_dequeued->buf.bytesused == 0)
    {
      /* If not yet receive image data from device, wait */

      sem_init(&sem, 0, 0);
      video_mng->dqbuf = &sem;
      leave_critical_section(flags);
      sem_wait(video_mng->dqbuf);
      flags = enter_critical_section();
      sem_destroy(&sem);
      video_mng->dqbuf = NULL;
    }

  memcpy(buf, &video_mng->dequeued->buf, sizeof(struct v4l2_buffer));

  /* Update buffer chain in critical section.
   *  - Delete dequeued buffer
   *  - next of latest buffer -> new dequeued buffer
   */

  video_mng->dequeued = old_dequeued->next;

  if (video_mng->latest->next != NULL)
    {
      video_mng->latest->next = video_mng->dequeued;
    }

  video_mng->bufnum--;

  if (video_mng->bufnum == 0)
    {
      g_video_ops->stop_getimage(buf->type, false);
      video_mng->dequeued = NULL;
      video_mng->latest   = NULL;
      video_mng->input    = NULL;
    }

  /* Free old dequeued buffer area */

  kmm_free(old_dequeued);

dqbuf_locked:
  /* Unlock buffer control */

  leave_critical_section(flags);
  video_unlock(&video_mng->exclbuf);

  return ret;
}

static int video_enum_fmt(FAR struct v4l2_fmtdesc *fmt)
{
  int ret;

  if ((g_video_ops == NULL) || (g_video_ops->get_range_of_fmt == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_ops->get_range_of_fmt(fmt);

  return ret;
}

static int video_enum_framesizes(FAR struct v4l2_frmsizeenum *frmsize)
{
  int ret;

  if ((g_video_ops == NULL) || (g_video_ops->get_range_of_framesize == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_ops->get_range_of_framesize(frmsize);

  return ret;
}

static int video_try_fmt(FAR struct v4l2_format *fmt)
{
  int ret;

  if ((g_video_ops == NULL) || (g_video_ops->try_format == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_ops->try_format(fmt);

  return ret;
}

static int video_s_fmt(FAR struct video_mng_s *priv,
                       FAR struct v4l2_format *fmt)
{
  int ret;

  if ((g_video_ops == NULL) || (g_video_ops->set_format == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_ops->set_format(fmt);

  return ret;
}

static int video_enum_frameintervals(FAR struct v4l2_frmivalenum *frmival)
{
  int ret;

  if ((g_video_ops == NULL) ||
      (g_video_ops->get_range_of_frameinterval == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_ops->get_range_of_frameinterval(frmival);

  return ret;
}

static int video_s_parm(FAR struct video_mng_s *priv,
                        FAR struct v4l2_streamparm *parm)
{
  int ret;

  if ((g_video_ops == NULL) ||
      (g_video_ops->set_frameinterval == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_ops->set_frameinterval(parm);

  return ret;
}

static int video_streamon(FAR struct video_mng_s *priv,
                          FAR enum v4l2_buf_type *type)
{
  int ret = OK;

  /* Parameter NULL check */

  if ((priv == NULL) || (type == NULL))
    {
      return -EINVAL;
    }

  if (*type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      if ((g_video_ops == NULL) || (g_video_ops->start_getimage == NULL))
        {
          return -EINVAL;
        }

      video_lock(&priv->video.exclbuf);

      if (priv->video.bufnum > 0)
        {
          ret = g_video_ops->start_getimage
                  (V4L2_BUF_TYPE_VIDEO_CAPTURE,
                   priv->video.input->buf.m.userptr,
                   priv->video.input->buf.length,
                   0);    /* no interval */
        }

      if (ret == OK)
        {
          /* Memory stream on request */

          priv->video.status = VIDEO_BUF_STATUS_TRIGGERED;
        }

      video_unlock(&priv->video.exclbuf);
    }

  return ret;
}

static int video_do_halfpush(FAR struct video_mng_s *priv, bool enable)
{
  int ret;

  if (priv == NULL)
    {
      return -EINVAL;
    }

  if ((g_video_ops == NULL) || (g_video_ops->do_halfpush == NULL))
    {
      return -EINVAL;
    }

  video_lock(&priv->exclhalfpush);
  ret = g_video_ops->do_halfpush(enable);
  video_unlock(&priv->exclhalfpush);

  return ret;
}

static int video_takepict_start(FAR struct video_mng_s *priv,
                                uint32_t               interval)
{
  int ret = OK;

  if (priv == NULL)
    {
      return -EINVAL;
    }

  if ((g_video_ops == NULL) || (g_video_ops->start_getimage == NULL))
    {
      return -EINVAL;
    } 

  video_lock(&priv->still.exclbuf);
  priv->still.interval = interval;
  if (priv->still.bufnum > 0)
    {
      ret = g_video_ops->start_getimage
              (V4L2_BUF_TYPE_STILL_CAPTURE,
               priv->still.input->buf.m.userptr,
               priv->still.input->buf.length,
               interval);
    }

  if (ret == OK)
    {
      /* Memory taking picture request */

      priv->still.status = VIDEO_BUF_STATUS_TRIGGERED;
    }
  video_unlock(&priv->still.exclbuf);

  return ret;
}

static int video_takepict_stop(FAR struct video_mng_s *priv, bool halfpush)
{
  int ret;

  if (priv == NULL)
    {
      return -EINVAL;
    }

  if ((g_video_ops == NULL) || (g_video_ops->stop_getimage == NULL))
    {
      return -EINVAL;
    }

  video_lock(&priv->still.exclbuf);
  ret = g_video_ops->stop_getimage(V4L2_BUF_TYPE_STILL_CAPTURE, halfpush);
  priv->still.status = VIDEO_BUF_STATUS_NOTRIGGER;
  video_unlock(&priv->still.exclbuf);

  return ret;
}

static int video_queryctrl(FAR struct v4l2_queryctrl *ctrl)
{
  int                        ret;
  struct v4l2_query_ext_ctrl ext_ctrl;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  /* Replace to VIDIOC_QUERY_EXT_CTRL format */

  ext_ctrl.ctrl_class = ctrl->ctrl_class;
  ext_ctrl.id         = ctrl->id;

  ret = video_query_ext_ctrl(&ext_ctrl);

  if (ret != OK)
    {
      return ret;
    }

  if ((ext_ctrl.type == V4L2_CTRL_TYPE_INTEGER64) ||
      (ext_ctrl.type == V4L2_CTRL_TYPE_U8) ||
      (ext_ctrl.type == V4L2_CTRL_TYPE_U16) ||
      (ext_ctrl.type == V4L2_CTRL_TYPE_U32))
    {
      /* Unsupported type in VIDIOC_QUERYCTRL */

      return -EINVAL;
    }

  /* Replace gotten value to VIDIOC_QUERYCTRL */

  ctrl->type          = ext_ctrl.type;
  ctrl->minimum       = ext_ctrl.minimum;
  ctrl->maximum       = ext_ctrl.maximum;
  ctrl->step          = ext_ctrl.step;
  ctrl->default_value = ext_ctrl.default_value;
  ctrl->flags         = ext_ctrl.flags;
  strncpy(ctrl->name, ext_ctrl.name, sizeof(ctrl->name));

  return OK;
}

static int video_query_ext_ctrl(FAR struct v4l2_query_ext_ctrl *ctrl)
{
  int ret;

  if ((g_video_ops == NULL) || (g_video_ops->get_range_of_ctrlvalue == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_ops->get_range_of_ctrlvalue(ctrl);

  return ret;
}

static int video_querymenu(FAR struct v4l2_querymenu *menu)
{
  int ret;

  if ((g_video_ops == NULL) || (g_video_ops->get_menu_of_ctrlvalue == NULL))
    {
      return -EINVAL;
    }

  ret = g_video_ops->get_menu_of_ctrlvalue(menu);

  return ret;
}

static int video_g_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl)
{
  int                      ret;
  struct v4l2_ext_controls ext_controls;
  struct v4l2_ext_control  control;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  /* Replace to VIDIOC_G_EXT_CTRLS format */

  control.id    = ctrl->id;

  ext_controls.ctrl_class = V4L2_CTRL_CLASS_USER;
  ext_controls.count      = 1;
  ext_controls.controls   = &control;

  /* Execute VIDIOC_G_EXT_CTRLS */

  ret = video_g_ext_ctrls(priv, &ext_controls);

  if (ret == OK)
    {
      /* Replace gotten value to VIDIOC_G_CTRL parameter */

      ctrl->value = control.value;
    }

  return ret;
}

static int video_s_ctrl(FAR struct video_mng_s *priv,
                        FAR struct v4l2_control *ctrl)
{
  int ret;
  struct v4l2_ext_controls ext_controls;
  struct v4l2_ext_control  control;

  if (ctrl == NULL)
    {
      return -EINVAL;
    }

  /* Replace to VIDIOC_S_EXT_CTRLS format */

  control.id    = ctrl->id;
  control.value = ctrl->value;

  ext_controls.ctrl_class = V4L2_CTRL_CLASS_USER;
  ext_controls.count      = 1;
  ext_controls.controls   = &control;

  /* Execute VIDIOC_S_EXT_CTRLS */

  ret = video_s_ext_ctrls(priv, &ext_controls);

  return ret;
}

static int video_g_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls)
{
  int ret = OK;
  int cnt;
  FAR struct v4l2_ext_control *control;

  if ((priv == NULL) || (ctrls == NULL))
    {
      return -EINVAL;
    }

  video_lock(&priv->exclctrl);

  for (cnt = 0, control = ctrls->controls;
       cnt < ctrls->count;
       cnt++, control++)
    {
      ret = g_video_ops->get_ctrlvalue(ctrls->ctrl_class, control);

      if (ret < 0)
        {
          /* Set cnt in that error occured */

          ctrls->error_idx = cnt;
          video_unlock(&priv->exclctrl);
          return ret;
        }
    }

  video_unlock(&priv->exclctrl);
  return ret;
}

static int video_s_ext_ctrls(FAR struct video_mng_s *priv,
                             FAR struct v4l2_ext_controls *ctrls)
{
  int ret;
  int cnt;
  FAR struct v4l2_ext_control *control;

  if ((priv == NULL) || (ctrls == NULL))
    {
      return -EINVAL;
    }

  video_lock(&priv->exclctrl);

  for (cnt = 0, control = ctrls->controls;
       cnt < ctrls->count;
       cnt++, control++)
    {
      ret = g_video_ops->set_ctrlvalue(ctrls->ctrl_class, control);

      if (ret < 0)
        {
          /* Set cnt in that error occured */

          ctrls->error_idx = cnt;
          video_unlock(&priv->exclctrl);
          return ret;
        }
    }

  ret = g_video_ops->refresh();
  if (ret < 0)
    {
      ctrls->error_idx = cnt;
    }

  video_unlock(&priv->exclctrl);
  return ret;
}


/****************************************************************************
 * Name: video_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 *
 ****************************************************************************/

static int video_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR video_mng_t  *priv  = inode->i_private;
  int ret = OK;


  switch (cmd)
    {
      case VIDIOC_REQBUFS:
        ret = video_reqbufs(priv, (FAR struct v4l2_requestbuffers *)arg);

        break;

      case VIDIOC_QBUF:
        ret = video_qbuf(priv, (FAR struct v4l2_buffer *)arg);

        break;

      case VIDIOC_DQBUF:
        ret = video_dqbuf(priv, (FAR struct v4l2_buffer *)arg);

        break;

      case VIDIOC_STREAMON:
        ret = video_streamon(priv, (FAR enum v4l2_buf_type *)arg);

        break;

      case VIDIOC_DO_HALFPUSH:
        ret = video_do_halfpush(priv, arg);

        break;

      case VIDIOC_TAKEPICT_START:
        ret = video_takepict_start(priv, arg);

        break;

      case VIDIOC_TAKEPICT_STOP:
        ret = video_takepict_stop(priv, arg);

        break;

      case VIDIOC_ENUM_FMT:
        ret = video_enum_fmt((FAR struct v4l2_fmtdesc *)arg);

        break;

      case VIDIOC_ENUM_FRAMESIZES:
        ret = video_enum_framesizes((FAR struct v4l2_frmsizeenum *)arg);

        break;

      case VIDIOC_TRY_FMT:
        ret = video_try_fmt((FAR struct v4l2_format *)arg);

        break;

      case VIDIOC_S_FMT:
        ret = video_s_fmt(priv, (FAR struct v4l2_format *)arg);

        break;

      case VIDIOC_ENUM_FRAMEINTERVALS:
        ret = video_enum_frameintervals((FAR struct v4l2_frmivalenum *)arg);

        break;

      case VIDIOC_S_PARM:
        ret = video_s_parm(priv, (FAR struct v4l2_streamparm *)arg);

        break;

      case VIDIOC_QUERYCTRL:
        ret = video_queryctrl((FAR struct v4l2_queryctrl *)arg);

        break;

      case VIDIOC_QUERY_EXT_CTRL:
        ret = video_query_ext_ctrl((FAR struct v4l2_query_ext_ctrl *)arg);

        break;

      case VIDIOC_QUERYMENU:
        ret = video_querymenu((FAR struct v4l2_querymenu *)arg);

      case VIDIOC_G_CTRL:
        ret = video_g_ctrl(priv, (FAR struct v4l2_control *)arg);

        break;

      case VIDIOC_S_CTRL:
        ret = video_s_ctrl(priv, (FAR struct v4l2_control *)arg);

        break;

      case VIDIOC_G_EXT_CTRLS:
        ret = video_g_ext_ctrls(priv, (FAR struct v4l2_ext_controls *)arg);

        break;

      case VIDIOC_S_EXT_CTRLS:
        ret = video_s_ext_ctrls(priv, (FAR struct v4l2_ext_controls *)arg);

        break;

      default:
        videoerr("Unrecognized cmd: %d\n", cmd);
        ret = - ENOTTY;
        break;
    }

  return ret;
}

static int video_poll_setup(FAR struct video_mng_s *priv,
                            FAR struct pollfd      *fds)
{
  irqstate_t flags;

  if ((fds->events & POLLIN) == 0)
    {
      return -EDEADLK;
    }

  flags = enter_critical_section();

  if ((priv->video.dequeued->buf.bytesused == 0) &&
      (priv->still.dequeued->buf.bytesused == 0))
    {
      priv->poll = fds;
    }
  else
    {
      sem_post(fds->sem);
    }

  leave_critical_section(flags);

  return OK;
}

static int video_poll_teardown(FAR struct video_mng_s *priv,
                               FAR struct pollfd      *fds)
{
  priv->poll = NULL;

  return OK;
}

static int video_poll(FAR struct file   *filep,
                      FAR struct pollfd *fds,
                      bool              setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR video_mng_t  *priv  = inode->i_private;

  if (setup)
    {
      return video_poll_setup(priv, fds);
    }
  else
    {
      return video_poll_teardown(priv, fds);
    }
  
  return OK;
}

static FAR void *video_register(FAR const char *devpath)
{
  FAR video_mng_t *priv;
  int ret;

  /* Initialize video device structure */

  priv = (FAR struct video_mng_s *)kmm_malloc(sizeof(struct video_mng_s));
  if (!priv)
    {
      videoerr("Failed to allocate instance\n");
      return priv;
    }

  memset(priv, 0, sizeof(struct video_mng_s));

  /* Save device path */

  priv->devpath = strdup(devpath);

  /* Initialize semaphore */

  sem_init(&priv->excldev,       0, 1);
  sem_init(&priv->exclctrl,      0, 1);
  sem_init(&priv->exclhalfpush,  0, 1);
  sem_init(&priv->video.exclbuf, 0, 1);
  sem_init(&priv->video.exclfmt, 0, 1);
  sem_init(&priv->still.exclbuf, 0, 1);
  sem_init(&priv->still.exclfmt, 0, 1);

  /* Register the character driver */

  ret = register_driver(priv->devpath, &g_video_fops, 0666, priv);
  if (ret < 0)
    {
      videoerr("Failed to register driver: %d\n", ret);
      kmm_free(priv->devpath);
      kmm_free(priv);
      return NULL;
    }

  return (FAR void *)priv;
}

static int video_unregister(void *priv)
{
  int ret = OK;

  if (priv)
    {
      sem_destroy(&((FAR video_mng_t *)priv)->still.exclfmt);
      sem_destroy(&((FAR video_mng_t *)priv)->still.exclbuf);
      sem_destroy(&((FAR video_mng_t *)priv)->video.exclfmt);
      sem_destroy(&((FAR video_mng_t *)priv)->video.exclbuf);
      sem_destroy(&((FAR video_mng_t *)priv)->exclhalfpush);
      sem_destroy(&((FAR video_mng_t *)priv)->exclctrl);
      sem_destroy(&((FAR video_mng_t *)priv)->excldev);
      unregister_driver((const char *)((FAR video_mng_t *)priv)->devpath);
      kmm_free(((FAR video_mng_t *)priv)->devpath);
      kmm_free(priv);
      return ret;
    }

  return -ENODEV;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
static FAR void *video_handler;

int video_initialize(FAR const char *devpath)
{
  g_video_ops = NULL;

#ifdef CONFIG_VIDEO_ISX012
  int ret;
  ret = board_isx012_initialize(devpath, IMAGER_I2C);
  if (ret != 0)
    {
      videoerr("ERROR: Failed to initialize ISX012 board. %d\n", errno);
      return ret;
    }

  g_video_ops = isx012_initialize();
#endif /* CONFIG_VIDEO_ISX012 */

  if (g_video_ops == NULL)
    {
      return ERROR;
    }

  video_handler = video_register(devpath);

  return OK;
}

int video_uninitialize(void)
{
  int ret;

  video_unregister(video_handler);

#ifdef CONFIG_VIDEO_ISX012
  isx012_uninitialize();

  ret = board_isx012_uninitialize();
  if (ret != 0)
    {
      videoerr("ERROR: Failed to uninitialize ISX012 board. %d\n", errno);
      return ret;
    }

#endif /* CONFIG_VIDEO_ISX012 */

  return OK;
}

int video_common_get_nextbuffer(uint32_t buf_type,
                                uint32_t datasize,
                                uint32_t data,
                                FAR uint32_t *nextbuf_size,
                                FAR uint32_t *nextbuf,
                                FAR void *priv)
{
  FAR struct v4l2_buffer             *next;
  FAR struct video_mng_per_buftype_s *video_mng;

  video_mng = check_buf_type((FAR struct video_mng_s *)priv, buf_type);
  if (video_mng == NULL)
    {
      return -EINVAL;
    }

  if (video_mng->input->buf.m.userptr != data)
    {
      /* 異常系 */
      /* dataのアドレス検索 and input 更新 */
    }

  /* Update bytesused */

  video_mng->input->buf.bytesused = datasize;

  /* Set the next buffer information */

  next = &video_mng->input->next->buf;
  *nextbuf = next->m.userptr;
  *nextbuf_size = next->length;

  /* If next address = NULL, stop until re-enqueued */

  if (*nextbuf == (uint32_t)NULL)
    {
      g_video_ops->stop_getimage(buf_type, false);
    }

  /* Update input link to  next buffer */

  video_mng->input = video_mng->input->next;

  /* Update dequeued if input oituki */

  if (video_mng->input == video_mng->dequeued)
    {
      video_mng->latest = video_mng->dequeued;
      video_mng->dequeued = video_mng->dequeued->next;
    }

  /* In polled case, unlock semaphore for poll */

  if (((FAR struct video_mng_s *)priv)->poll)
    {
      ((FAR struct video_mng_s *)priv)->poll->revents |= POLLIN;
      sem_post(((FAR struct video_mng_s *)priv)->poll->sem);
    }

  /* In receiving dequeue request, unlock semaphore for dequeue request */

  if (video_mng->dqbuf)
    {
      sem_post(video_mng->dqbuf);
    }

  return OK;
}
