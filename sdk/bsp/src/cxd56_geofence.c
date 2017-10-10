/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_geofence.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Takefumi Hayashi <Takefumi.Hayashi@sony.com>
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
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fixedmath.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>

#include <arch/chip/cxd56_geofence.h>
#include "cxd56_gnss_api.h"
#include "cxd56_cpu1signal.h"

#if defined(CONFIG_CXD56_GEOFENCE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_GEOFENCE_NPOLLWAITERS
#  define CONFIG_GEOFENCE_NPOLLWAITERS    4
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct geofence_dev_s
{
  sem_t          devsem;
  struct pollfd *fds[CONFIG_GEOFENCE_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int geofence_open(FAR struct file *filep);
static int geofence_close(FAR struct file *filep);
static ssize_t geofence_read(FAR struct file *filep, FAR char *buffer,
                              size_t len);
static int geofence_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
static int geofence_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_geofencefops = {
  geofence_open,  /* open */
  geofence_close, /* close */
  geofence_read,  /* read */
  0,              /* write */
  0,              /* seek */
  geofence_ioctl, /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  geofence_poll, /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*
 * ### CXD56_GEOFENCE_IOCTL_START ###
 * 
 * Start GEOFENCE Detect
 */
static int start(unsigned long arg)
{
  return GD_RegisterGeofence();
}

/*
 * ### CXD56_GEOFENCE_IOCTL_STOP ###
 * 
 * Start GEOFENCE Detect
 */
static int stop(unsigned long arg)
{
  return GD_ReleaseGeofence();
}

/*
 * ### CXD56_GEOFENCE_IOCTL_ADD ###
 * 
 * Add region
 */
static int add_region(unsigned long arg)
{
  int                                    ret;
  FAR struct cxd56_geofence_region_s *reg_data;
  
  if (!arg)
    {
      return -EINVAL;
    }
  reg_data = (FAR struct cxd56_geofence_region_s *)arg;
  
  ret = GD_GeoAddRegion(reg_data->id, reg_data->latitude, reg_data->longitude,
                                                          reg_data->radius);
  
  return ret;
}

/*
 * ### CXD56_GEOFENCE_IOCTL_MODIFY ###
 * 
 * Modify region
 */
static int modify_region(unsigned long arg)
{
  int                                    ret;
  FAR struct cxd56_geofence_region_s *reg_data;
  
  if (!arg)
    {
      return -EINVAL;
    }
  reg_data = (FAR struct cxd56_geofence_region_s *)arg;
 
  ret = GD_GeoModifyRegion(reg_data->id, reg_data->latitude, 
                           reg_data->longitude, reg_data->radius);
  
  return ret;
}

/*
 * ### CXD56_GEOFENCE_IOCTL_DELETE ###
 * 
 * delete region
 */
static int delete_region(unsigned long arg)
{
  int         ret;
  FAR uint8_t id;
  
  if (UINT8_MAX < arg)
    {
      return -EINVAL;
    }
  
  id = (uint8_t)arg;
  ret = GD_GeoDeleteRegione(id);
  
  return ret;
}

/*
 * ### CXD56_GEOFENCE_IOCTL_ALL_DELETE ###
 * 
 * All delete region
 */
static int delete_all_region(unsigned long arg)
{
  int ret;
  
  ret = GD_GeoDeleteAllRegion();
  
  return ret;
}


/*
 * ### CXD56_GEOFENCE_IOCTL_GET_REGION_DATA ###
 * 
 * Get used region ID
 */
static int get_region_data(unsigned long arg)
{
  int                                    ret;
  FAR struct cxd56_geofence_region_s *reg_data;
  
  if (!arg)
    {
      return -EINVAL;
    }
  reg_data = (FAR struct cxd56_geofence_region_s *)arg;
  
  ret = GD_GeoGetRegionData(reg_data->id, &reg_data->latitude, 
                            &reg_data->longitude, &reg_data->radius);
  
  return ret;
}

/*
 * ### CXD56_GEOFENCE_IOCTL_GET_USED_ID ###
 * 
 * Get used region ID
 */
static int get_used_id(unsigned long arg)
{
  if (!arg)
    {
      return -EINVAL;
    }
  
  *(uint32_t *)arg = GD_GeoGetUsedRegionId();
  
  return 0;
}

/*
 * ### CXD56_GEOFENCE_IOCTL_GET_ALL_STATUS ###
 * 
 * Get All transition status
 */
static int get_all_status(unsigned long arg)
{
  int ret;
  
  ret = GD_GeoSetAllRgionNotifyRequest();
  
  return ret;
}

/*
 * ### CXD56_GEOFENCE_IOCTL_SET_MODE ###
 * 
 * Set geofence operation mode
 */
static int set_mode(unsigned long arg)
{
  int                                   ret;
  FAR struct cxd56_geofence_mode_s *mode;
  
  if (!arg)
    {
      return -EINVAL;
    }
  mode = (FAR struct cxd56_geofence_mode_s *)arg;
  
  ret = GD_GeoSetOpMode(mode->deadzone, mode->dwell_detecttime);
  
  return ret;
}

/*
 *  GEOFENCE worker for poll
 */

static void geofence_sighandler(uint32_t data, void *userdata)
{
  FAR struct geofence_dev_s *priv = (FAR struct geofence_dev_s *)userdata;
  int                        i;
  int                        ret;

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      return;
    }
  
  for (i = 0; i < CONFIG_GEOFENCE_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          snvdbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
        }
    }

  sem_post(&priv->devsem);
}

/****************************************************************************
 * Name: geofence_initialize
 *
 * Description:
 *   initialize gnss device
 *
 ****************************************************************************/
static int geofence_initialize(struct geofence_dev_s* dev)
{
  int32_t ret = 0;

  return ret;
}

/****************************************************************************
 * Name: geofence_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int geofence_open(FAR struct file *filep)
{
  int32_t ret = 0;
  
  return ret;
}

/****************************************************************************
 * Name: geofence_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int geofence_close(FAR struct file *filep)
{
  int32_t ret = 0;

  return ret;
}

/****************************************************************************
 * Name: geofence_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t geofence_read(FAR struct file *filep, FAR char *buffer,
                              size_t len)
{
  int32_t ret = 0;

  if (!buffer)
    {
      ret = -EINVAL;
      goto _err;
    }
  if (len == 0)
    {
      return 0;
    }
  
  /* GD_ReadBuffer returns copied data size or negative error code */
  ret = GD_ReadBuffer(CXD56_CPU1_DEV_GEOFENCE, 0, buffer, len);
  if (ret > 0)
    {
      ret = 0; /* always return EOF if reading successfully */
    }

_err:
  return ret;
}



/****************************************************************************
 * Name: geofence_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 ****************************************************************************/
static int geofence_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  static int (*cmd_list[GEOFENCE_IOCTL_MAX])(unsigned long) = {
    NULL,                           /* CXD56_GEOFENCE_IOCTL_INVAL = 0 */
    start,                          /* CXD56_GEOFENCE_IOCTL_START */
    stop,                           /* CXD56_GEOFENCE_IOCTL_STOP */
    add_region,                     /* CXD56_GEOFENCE_IOCTL_ADD */
    modify_region,                  /* CXD56_GEOFENCE_IOCTL_MODIFY */
    delete_region,                  /* CXD56_GEOFENCE_IOCTL_DELETE */
    delete_all_region,              /* CXD56_GEOFENCE_IOCTL_ALL_DELETE */
    get_region_data,                /* CXD56_GEOFENCE_IOCTL_GET_REGION_DATA */
    get_used_id,                    /* CXD56_GEOFENCE_IOCTL_GET_USED_ID */
    get_all_status,                 /* CXD56_GEOFENCE_IOCTL_GET_ALL_STATUS */
    set_mode,                       /* CXD56_GEOFENCE_IOCTL_SET_MODE */
    /* max                             CXD56_GEOFENCE_IOCTL_MAX */
  };

  if (cmd <= GEOFENCE_IOCTL_INVAL || cmd >= GEOFENCE_IOCTL_MAX)
    {
      return -EINVAL;
    }

  return cmd_list[cmd](arg);
}


/****************************************************************************
 * Name: geofence_poll
 *
 * Description:
 *   Standard character driver poll method.
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int geofence_poll(FAR struct file *filep, FAR struct pollfd *fds,
                          bool setup)
{
  FAR struct inode *          inode;
  FAR struct geofence_dev_s  *priv;
  int                         ret = OK;
  int                         i;

  inode = filep->f_inode;
  priv  = (FAR struct geofence_dev_s *)inode->i_private;

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      for (i = 0; i < CONFIG_GEOFENCE_NPOLLWAITERS; i++)
        {
          /* Find an unused slot */

          if (priv->fds[i] == NULL)
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              GD_SetNotifyMask(CXD56_CPU1_DEV_GEOFENCE, FALSE);
              break;
            }
        }

      /* No space in priv fds array for poll handling */

      if (i >= CONFIG_GEOFENCE_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret       = -EBUSY;
          goto errout;
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  sem_post(&priv->devsem);
  return ret;
}
#endif

/****************************************************************************
 * Name: geofence_register
 *
 * Description:
 *   Register the GEOFENCE character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/geofence"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int geofence_register(FAR const char *devpath)
{
  FAR struct geofence_dev_s *priv;
  int                        ret;

  priv = (FAR struct geofence_dev_s *)kmm_malloc(
      sizeof(struct geofence_dev_s));
  if (!priv)
    {
      sndbg("Failed to allocate instance\n");
      return -ENOMEM;
    }
  memset(priv, 0, sizeof(struct geofence_dev_s));
  sem_init(&priv->devsem, 0, 1);

  ret = geofence_initialize(priv);
  if (ret < 0)
    {
      sndbg("Failed to initialize geofence device!\n");
      goto _err0;
    }
  
  ret = register_driver(devpath, &g_geofencefops, 0666, priv);
  if (ret < 0)
    {
      sndbg("Failed to register driver: %d\n", ret);
      goto _err0;
    }

  ret = cxd56_cpu1siginit(CXD56_CPU1_DEV_GEOFENCE, priv);
  if (ret < 0)
    {
      sndbg("Failed to initialize ICC for GPS CPU: %d\n", ret);
      goto _err2;
    }

  cxd56_cpu1sigregisterhandler(CXD56_CPU1_DEV_GEOFENCE, geofence_sighandler);

  snvdbg("GEOFENCE driver loaded successfully!\n");

  return ret;

_err2:
  unregister_driver(devpath);
_err0:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: cxd56_geofenceinitialize
 ****************************************************************************/

int cxd56_geofenceinitialize(FAR const char *devpath)
{
  int ret;

  snvdbg("Initializing GEOFENCE..\n");

  ret = geofence_register(devpath);
  if (ret < 0)
    {
      sndbg("Error registering GEOFENCE\n");
    }

  return ret;
}

#endif
