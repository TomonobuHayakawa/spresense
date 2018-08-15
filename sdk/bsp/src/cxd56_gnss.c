/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_gnss.c
 *
 *   Copyright (C) 2016,2017 Sony Corporation. All rights reserved.
 *   Author: Tomoyuki Takahashi <Tomoyuki.A.Takahashi@sony.com>
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

#include <nuttx/config.h>

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
#include <arch/chip/cxd56_gnss.h>
#include "cxd56_gnss_api.h"
#include "cxd56_cpu1signal.h"

#if defined(CONFIG_CXD56_GNSS)

/****************************************************************************
 * External Defined Functions
 ****************************************************************************/

extern int PM_LoadImage(int cpuid, const char* filename);
extern int PM_StartCpu(int cpuid, int wait);
extern int PM_SleepCpu(int cpuid, int mode);
extern int PM_GetCpuPowerStatus(int cpuid,int* status);
extern int PM_WakeUpCpu(int cpuid);
#ifndef CONFIG_CXD56_GNSS_DISABLE_LNA_CONTROL
extern int cxd56_gnssext_control_lna(bool en);
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CXD56_GNSS_NPOLLWAITERS
#  define CONFIG_CXD56_GNSS_NPOLLWAITERS      4
#endif

#ifndef CONFIG_CXD56_GNSS_NSIGNALRECEIVERS
#  define CONFIG_CXD56_GNSS_NSIGNALRECEIVERS  3
#endif

#ifndef CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE
#  define CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE 1024
#endif

#ifndef CONFIG_CXD56_GNSS_BACKUP_FILENAME
#  define CONFIG_CXD56_GNSS_BACKUP_FILENAME   "/mnt/spif/gnss_backup.bin"
#endif

#ifndef CONFIG_CXD56_GNSS_CEP_FILENAME
#  define CONFIG_CXD56_GNSS_CEP_FILENAME      "/mnt/spif/gnss_cep.bin"
#endif

#define CXD56_GNSS_GPS_CPUID                  1
#ifdef  CONFIG_CXD56_GNSS_FW_RTK
#  define CXD56_GNSS_FWNAME                   "gnssfwrtk"
#else
#  define CXD56_GNSS_FWNAME                   "gnssfw"
#endif
#ifndef PM_SLEEP_MODE_COLD
#  define PM_SLEEP_MODE_COLD                  2
#endif
#ifndef PM_SLEEP_MODE_HOT_ENABLE
#  define PM_SLEEP_MODE_HOT_ENABLE            7
#endif
#ifndef PM_SLEEP_MODE_HOT_DISABLE
#  define PM_SLEEP_MODE_HOT_DISABLE           8
#endif

/* Notify data of PUBLISH_TYPE_GNSS */

#define CXD56_GNSS_NOTIFY_TYPE_POSITION       0
#define CXD56_GNSS_NOTIFY_TYPE_BOOTCOMP       1
#define CXD56_GNSS_NOTIFY_TYPE_REQBKUPDAT     2
#define CXD56_GNSS_NOTIFY_TYPE_REQCEPOPEN     3
#define CXD56_GNSS_NOTIFY_TYPE_REQCEPCLOSE    4
#define CXD56_GNSS_NOTIFY_TYPE_REQCEPDAT      5
#define CXD56_GNSS_NOTIFY_TYPE_REQCEPBUFFREE  6

/* Common info shared with GNSS core */

#define SHARED_INFO_MAX_ARGC                  6

/* GNSS core CPU FIFO interface API */

#define CXD56_GNSS_CPUFIFOAPI_GD_START             0
#define CXD56_GNSS_CPUFIFOAPI_GD_STOP              1
#define CXD56_GNSS_CPUFIFOAPI_GD_CEPINITASSISTDATA 2

/* CPU FIFO API bitfield converter */

#define CXD56_GNSS_CPUFIFOAPI_SET_DATA(API, DATA) (((DATA) << 8) | (API))

/* GDSP File read/write arguments */

#define GNSS_ARGS_FILE_OFFSET		              0
#define GNSS_ARGS_FILE_BUF			              1
#define GNSS_ARGS_FILE_LENGTH		              2

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifndef CONFIG_DISABLE_SIGNAL
struct cxd56_gnss_sig_s
{
  uint8_t                           enable;
  int                               pid;
  FAR struct cxd56_gnss_signal_info info;
};
#endif

struct cxd56_gnss_shared_info_s
{
  int      retval;
  uint32_t argc;
  uint32_t argv[SHARED_INFO_MAX_ARGC];
};

struct cxd56_gnss_dev_s
{
  sem_t          devsem;
  sem_t          syncsem;
  uint8_t        num_open;
  uint8_t        notify_data;
  FILE *         cepfp;
  void *         cepbuf;
  struct pollfd *fds[CONFIG_CXD56_GNSS_NPOLLWAITERS];
#if !defined(CONFIG_DISABLE_SIGNAL) && \
  (CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0)
  struct cxd56_gnss_sig_s sigs[CONFIG_CXD56_GNSS_NSIGNALRECEIVERS];
#endif
  struct cxd56_gnss_shared_info_s shared_info;
  sem_t          ioctllock;
  sem_t          apiwait;
  int            apiret;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cxd56_gnss_open(FAR struct file *filep);
static int cxd56_gnss_close(FAR struct file *filep);
static ssize_t cxd56_gnss_read(FAR struct file *filep, FAR char *buffer,
                               size_t len);
static ssize_t cxd56_gnss_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static int cxd56_gnss_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int cxd56_gnss_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup);
static int8_t cxd56_gnss_select_notifytype(off_t fpos, uint32_t *offset);

static int cxd56_gnss_cpufifo_api(FAR struct file *filep,
                                  unsigned int api, unsigned int data);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This the vtable that supports the character driver interface */

static const struct file_operations g_gnssfops = {
  cxd56_gnss_open,  /* open */
  cxd56_gnss_close, /* close */
  cxd56_gnss_read,  /* read */
  cxd56_gnss_write, /* write */
  0,                /* seek */
  cxd56_gnss_ioctl, /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  cxd56_gnss_poll,  /* poll */
#endif
};

/* Common info shared with GNSS core */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IOCTL private functions */

/*
 * === CXD56_GNSS_IOCTL_START ===
 * Start a positioning
 * begining to search the satellites and measure the receiver position
 */
static int start(FAR struct file *filep, unsigned long arg)
{
  int     ret;
  uint8_t start_mode = (uint8_t)arg;

#ifndef CONFIG_CXD56_GNSS_DISABLE_LNA_CONTROL
  ret = cxd56_gnssext_control_lna(true);
  if (ret < 0)
    {
      return ret;
    }
#endif

  ret = cxd56_gnss_cpufifo_api(filep, CXD56_GNSS_CPUFIFOAPI_GD_START,
                               start_mode);
#ifndef CONFIG_CXD56_GNSS_DISABLE_LNA_CONTROL
  if (ret < 0)
    {
      cxd56_gnssext_control_lna(false);
    }
#endif

  return ret;
}

/*
 * === CXD56_GNSS_IOCTL_STOP ===
 * Stop a positioning
 */
static int stop(FAR struct file *filep, unsigned long arg)
{
  int     ret;

  ret = cxd56_gnss_cpufifo_api(filep, CXD56_GNSS_CPUFIFOAPI_GD_STOP, 0);
#ifndef CONFIG_CXD56_GNSS_DISABLE_LNA_CONTROL
  cxd56_gnssext_control_lna(false);
#endif

  return ret;
}

/*
 * === CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM ===
 * Select GNSSs to positioning
 * These are able to specified by GD_B_SAT_XXX defines.
 */
static int select_satellite_system(FAR struct file *filep, unsigned long arg)
{
  uint32_t system = (uint32_t)arg;

  return GD_SelectSatelliteSystem(system);
}

/*
 * === CXD56_GNSS_IOCTL_GET_SATELLITE_SYSTEM ===
 * Get current using GNSSs to positioning
 * A argument 'satellite' indicates current GNSSs by bit fields defined by GD_B_SAT_XXX.
 */
static int get_satellite_system(FAR struct file *filep, unsigned long arg)
{
  int ret;
  uint32_t system;

  if (!arg)
    {
    return -EINVAL;
    } 
  ret = GD_GetSatelliteSystem(&system);
  *(uint32_t *)arg = system;

  return ret;
}

/*
 * === CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ELLIPSOIDAL ===
 * Set the rough receiver position
 *   arg = { double lat, double lon, double height }
 */
static int
set_receiver_position_ellipsoidal(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_ellipsoidal_position *pos;

  if (!arg)
    {
      return -EINVAL;
    }
  pos = (FAR struct cxd56_gnss_ellipsoidal_position *)arg;
  return GD_SetReceiverPositionEllipsoidal(&pos->latitude, &pos->longitude,
                                           &pos->altitude);
}

/*
 * === CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ORTHOGONAL ===
 * Set the rough receiver position as orgothonal
 *   arg = { int32_t x, int32_t y, int32_t z }
 */
static int set_receiver_position_orthogonal(FAR struct file *filep,
                                            unsigned long    arg)
{
  FAR struct cxd56_gnss_orthogonal_position *pos;

  if (!arg)
    {
      return -EINVAL;
    }
  pos = (FAR struct cxd56_gnss_orthogonal_position *)arg;
  return GD_SetReceiverPositionOrthogonal(pos->x, pos->y, pos->z);
}

/*
 * === CXD56_GNSS_IOCTL_SET_OPE_MODE ===
 * Set the TCXO offset
 */
static int set_ope_mode(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_ope_mode_param *ope_mode;
  if (!arg)
    {
      return -EINVAL;
    }
  ope_mode = (FAR struct cxd56_gnss_ope_mode_param *)arg;

  return GD_SetOperationMode(ope_mode->mode, ope_mode->cycle);
}

/*
 * === CXD56_GNSS_IOCTL_GET_OPE_MODE ===
 * Set the TCXO offset
 */
static int get_ope_mode(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_ope_mode_param *ope_mode;
  if (!arg)
    {
      return -EINVAL;
    }
  ope_mode = (FAR struct cxd56_gnss_ope_mode_param *)arg;

  return GD_GetOperationMode(&ope_mode->mode, &ope_mode->cycle);
}

/*
 * === CXD56_GNSS_IOCTL_SET_TCXO_OFFSET ===
 * Set the TCXO offset
 */
static int set_tcxo_offset(FAR struct file *filep, unsigned long arg)
{
  int32_t offset = (int32_t)arg;

  return GD_SetTcxoOffset(offset);
}

/*
 * === CXD56_GNSS_IOCTL_GET_TCXO_OFFSET ===
 * Get the TCXO offset
 */
static int get_tcxo_offset(FAR struct file *filep, unsigned long arg)
{
  int     ret;
  int32_t offset;

  if (!arg)
    {
      return -EINVAL;
    }
  ret              = GD_GetTcxoOffset(&offset);
  *(uint32_t *)arg = offset;

  return ret;
}

/*
 * === CXD56_GNSS_IOCTL_SET_TIME ===
 * Set the estimated current time of the receiver.
 * 1st argument date & time are in UTC.
 */
static int set_time(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_datetime *date_time;

  if (!arg)
    {
      return -EINVAL;
    }
  date_time = (FAR struct cxd56_gnss_datetime *)arg;
  return GD_SetTime(&date_time->date, &date_time->time);
}

/*
 * === CXD56_GNSS_IOCTL_GET_ALMANAC ===
 * Get the almanac data
 */
static int get_almanac(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_orbital_param *param;
  uint32_t                            almanac_size;

  if (!arg)
    {
      return -EINVAL;
    }
  param = (FAR struct cxd56_gnss_orbital_param *)arg;
  return GD_GetAlmanac(param->type, param->data, &almanac_size);
}

/*
 * === CXD56_GNSS_IOCTL_SET_ALMANAC ===
 * Set the almanac data
 */
static int set_almanac(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_orbital_param *param;

  if (!arg)
    {
      return -EINVAL;
    }
  param = (FAR struct cxd56_gnss_orbital_param *)arg;
  return GD_SetAlmanac(param->type, param->data);
}

/*
 * === CXD56_GNSS_IOCTL_GET_EPHEMERIS ===
 * Get the Ephemeris data
 */
static int get_ephemeris(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_orbital_param *param;
  uint32_t                            ephemeris_size;

  if (!arg)
    {
      return -EINVAL;
    }
  param = (FAR struct cxd56_gnss_orbital_param *)arg;
  return GD_GetEphemeris(param->type, param->data, &ephemeris_size);
}

/*
 * === CXD56_GNSS_IOCTL_SET_EPHEMERIS ===
 * Set the Ephemeris data
 */
static int set_ephemeris(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_orbital_param *param;

  if (!arg)
    {
      return -EINVAL;
    }
  param = (FAR struct cxd56_gnss_orbital_param *)arg;
  return GD_SetEphemeris(param->type, param->data);
}

/*
 * === CXD56_GNSS_IOCTL_SAVE_BACKUP_DATA ===
 * Save the backup data to a Flash memory.
 */
static int save_backup_data(FAR struct file *filep, unsigned long arg)
{
  char *buf;
  FILE *fp;
  int n = 0;
  int32_t offset = 0;

  buf = (char *)malloc(CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE);
  if (buf == NULL)
    {
      return -ENOMEM;
    }

  fp = fopen(CONFIG_CXD56_GNSS_BACKUP_FILENAME, "wb");
  if (fp == NULL)
    {
      free(buf);
      return -ENOENT;
    }

  do
    {
      n = GD_ReadBuffer(CXD56_CPU1_DATA_TYPE_BACKUP, offset, buf,
                        CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE);
      if (n <= 0)
        {
          break;
        }
      n = fwrite(buf, 1, n, fp);
      offset += n;
    }
  while (n == CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE);

  free(buf);
  fclose(fp);

  return n < 0 ? n : 0;
}

/*
 * === CXD56_GNSS_IOCTL_ERASE_BACKUP_DATA ===
 * Erase the backup data on a Flash memory.
 */
static int erase_backup_data(FAR struct file *filep, unsigned long arg)
{
  return unlink(CONFIG_CXD56_GNSS_BACKUP_FILENAME);
}

/*
 * === CXD56_GNSS_IOCTL_OPEN_CEP_DATA ===
 *  open CEP data file 
 */
static int open_cep_data(FAR struct file *filep, unsigned long arg)
{
  return cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_CEPFILE, TRUE);
}

/*
 * === CXD56_GNSS_IOCTL_CLOSE_CEP_DATA ===
 *  close CEP data file
 */
static int close_cep_data(FAR struct file *filep, unsigned long arg)
{
  return cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_CEPFILE, FALSE);
}

/*
 * === CXD56_GNSS_IOCTL_CHECK_CEP_DATA ===
 * Check CEP data valid
 */
static int check_cep_data(FAR struct file *filep, unsigned long arg)
{
  return GD_CepCheckAssistData();
}

/*
 * === CXD56_GNSS_IOCTL_GET_CEP_AGE ===
 *  Get CEP valid term
 */
static int get_cep_age(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_cep_age *age;

  if (!arg)
    {
      return -EINVAL;
    }
  age = (FAR struct cxd56_gnss_cep_age *)arg;
  return GD_CepGetAgeData(&age->age, &age->cepi);
}

/*
 * === CXD56_GNSS_IOCTL_RESET_CEP_FLAG ===
 * Reset CEP data init flag & valid flag
 */
static int reset_cep_flag(FAR struct file *filep, unsigned long arg)
{
  return cxd56_gnss_cpufifo_api(filep,
                                CXD56_GNSS_CPUFIFOAPI_GD_CEPINITASSISTDATA, 0);
}

/*
 * === CXD56_GNSS_IOCTL_AGPS_SET_ACQUIST ===
 *  AGPS set acquist data
 */
static int set_acquist_data(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_agps_acquist *acquist;

  if (!arg)
    {
      return -EINVAL;
    }
  acquist = (FAR struct cxd56_gnss_agps_acquist *)arg;
  return GD_SetAcquist(acquist->data, acquist->size);
}

/*
 * === CXD56_GNSS_IOCTL_AGPS_SET_FRAMETIME ===
 *  AGPS set frame time
 */
static int set_frametime(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_agps_frametime *frametime;

  if (!arg)
    {
      return -EINVAL;
    }
  frametime = (FAR struct cxd56_gnss_agps_frametime *)arg;
  return GD_SetFrameTime(frametime->sec, frametime->frac);
}

/*
 * === CXD56_GNSS_IOCTL_AGPS_SET_TAU_GPS ===
 *  AGPS set TAU GPS
 */
static int set_tau_gps(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_agps_tau_gps *taugpstime;
  
  if (!arg)
    {
      return -EINVAL;
    }
  taugpstime = (FAR struct cxd56_gnss_agps_tau_gps *)arg;
  
  return GD_SetTauGps(&taugpstime->tauGps);
}

/*
 * === CXD56_GNSS_IOCTL_AGPS_SET_TIME_GPS ===
 *  Set high precision receiver time
 */
static int set_time_gps(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_agps_time_gps *time_gps;

  if (!arg)
    {
      return -EINVAL;
    }
  time_gps = (FAR struct cxd56_gnss_agps_time_gps *)arg;
  return GD_SetTimeGps(&time_gps->date, &time_gps->time);
}

/*
 * === CXD56_GNSS_IOCTL_AGPS_CLEAR_RECEIVER_INFO ===
 *  Clear info(s) for hot start
 */
static int clear_receiver_info(FAR struct file *filep, unsigned long arg)
{
  uint32_t clear_type = (double)arg;

  return GD_ClearReceiverInfo(clear_type);
}

/*
 * === CXD56_GNSS_IOCTL_AGPS_SET_TOW_ASSIST ===
 *  AGPS set acquist data
 */
static int set_tow_assist(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_agps_tow_assist *assist;

  if (!arg)
    {
      return -EINVAL;
    }
  assist = (FAR struct cxd56_gnss_agps_tow_assist *)arg;
  return GD_SetTowAssist(assist->data, assist->size);
}

/*
 * === CXD56_GNSS_IOCTL_AGPS_SET_UTC_MODEL ===
 *  AGPS set acquist data
 */
static int set_utc_model(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_agps_utc_model *model;

  if (!arg)
    {
      return -EINVAL;
    }
  model = (FAR struct cxd56_gnss_agps_utc_model *)arg;
  return GD_SetUtcModel(model->data, model->size);
}

/*
 * === CXD56_GNSS_IOCTL_SPECTRUM_CONTROL ===
 *  Enable or not to output spectrum data of GNSS signal
 */
static int control_spectrum(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_spectrum_control *control;

  if (!arg)
    {
      return -EINVAL;
    }
  control = (FAR struct cxd56_gnss_spectrum_control *)arg;
  return GD_SpectrumControl(control->time, control->enable, control->point1,
                            control->step1, control->point2, control->step2);
}

/*
 * === CXD56_GNSS_IOCTL_FACTORY_START_TEST ===
 *  Start GPS factory test
 */
static int start_test(FAR struct file *filep, unsigned long arg)
{
  int ret;
  FAR struct cxd56_gnss_test_info *info;

  /* check argument */

  if (!arg)
    {
      ret = -EINVAL;
    }
  else
    {
      /* set parameter */

      info = (FAR struct cxd56_gnss_test_info *)arg;
      GD_StartGpsTest(info->satellite, info->reserve1, info->reserve2, info->reserve3);

      /* start test */

      ret = GD_Start(CXD56_GNSS_STMOD_COLD);
    }

  return ret;
}

/*
 * === CXD56_GNSS_IOCTL_FACTORY_STOP_TEST ===
 *  Stop GPS factory test
 */
static int stop_test(FAR struct file *filep, unsigned long arg)
{
  int ret;

  /* term test */

  ret = GD_StopGpsTest();
  if(ret == OK)
    {
      /* stop test */

      ret = GD_Stop();
    }

  return ret;
}

/*
 * === CXD56_GNSS_IOCTL_FACTORY_GET_TEST_RESULT ===
 *  Get GPS factory test result
 */
static int get_test_result(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_test_result *result;

  if (!arg)
    {
      return -EINVAL;
    }
  result = (FAR struct cxd56_gnss_test_result *)arg;
  return GD_GetGpsTestResult(&result->cn, &result->doppler);
}

/*
 * === CXD56_GNSS_IOCTL_SIGNAL_SET ===
 *  Set signal information for synchronous reading data
 */
static int set_signal(FAR struct file *filep, unsigned long arg)
{
  int ret = 0;

#if !defined(CONFIG_DISABLE_SIGNAL) && \
  (CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0)
  FAR struct inode                     *inode;
  FAR struct cxd56_gnss_dev_s          *priv;
  FAR struct cxd56_gnss_signal_setting *setting;
  FAR struct cxd56_gnss_sig_s          *sig;
  FAR struct cxd56_gnss_sig_s          *checksig;
  int                                   pid;
  int                                   i;

  if (!arg)
    {
      return -EINVAL;
    }

  setting = (FAR struct cxd56_gnss_signal_setting *)arg;
  if (setting->gnsssig >= CXD56_CPU1_DATA_TYPE_MAX)
    {
      return -EPROTOTYPE;
    }

  inode = filep->f_inode;
  priv  = (FAR struct cxd56_gnss_dev_s *)inode->i_private;

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  sig = NULL;
  pid = getpid();
  for (i = 0; i < CONFIG_CXD56_GNSS_NSIGNALRECEIVERS; i++)
    {
      checksig = &priv->sigs[i];
      if (setting->enable)
        {
          if (sig == NULL && !checksig->enable)
            {
              sig = checksig;
            }
          else if (checksig->info.gnsssig == setting->gnsssig &&
                   checksig->pid == pid)
            {
              sig = checksig;
              break;
            }
        }
      else if (checksig->info.gnsssig == setting->gnsssig &&
               checksig->pid == pid)
        {
          checksig->enable = 0;
          goto _success;
        }
    }
  if (sig == NULL)
    {
      ret = -ENOENT;
      goto _err;
    }

  GD_SetNotifyMask(setting->gnsssig, FALSE); 

  sig->enable       = 1;
  sig->pid          = pid;
  sig->info.fd      = setting->fd;
  sig->info.gnsssig = setting->gnsssig;
  sig->info.signo   = setting->signo;
  sig->info.data    = setting->data;

_success:
_err:
  sem_post(&priv->devsem);
#endif /* if !defined(CONFIG_DISABLE_SIGNAL) && \
          (CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0) */
  return ret;
}

/*
 * === CXD56_GNSS_IOCTL_PVTLOG_START ===
 *  Start saving PVT logs.
 */
static int start_pvtlog(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_pvtlog_setting *setting;

  if (!arg)
    {
      return -EINVAL;
    }
  setting = (FAR struct cxd56_gnss_pvtlog_setting *)arg;
  return GD_RegisterPvtlog(setting->cycle, setting->threshold);
}

/*
 * === CXD56_GNSS_IOCTL_PVTLOG_STOP ===
 * Stop saving PVT logs.
 */
static int stop_pvtlog(FAR struct file *filep, unsigned long arg)
{
  return GD_ReleasePvtlog();
}

/*
 * === CXD56_GNSS_IOCTL_PVTLOG_STOP ===
 * Delete stored PVT logs.
 */
static int delete_pvtlog(FAR struct file *filep, unsigned long arg)
{
  return GD_PvtlogDeleteLog();
}

/*
 * === CXD56_GNSS_IOCTL_PVTLOG_GET_STATUS ===
 *  Get stored log status of PVTLOG.
 */
static int get_pvtlog_status(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_pvtlog_status *status;

  if (!arg)
    {
      return -EINVAL;
    }
  status = (FAR struct cxd56_gnss_pvtlog_status *)arg;
  return GD_PvtlogGetLogStatus(&status->status);
}

/*
 *  Synchronized with processes and CPUs
 *  CXD56_GNSS signal handler and utils
 */

/* utils for sync */

static int wait_gnss_notify(FAR sem_t *sem, time_t waitsec)
{
  int             ret;
  struct timespec timeout;

  ret = clock_gettime(CLOCK_REALTIME, &timeout);
  if (ret < 0)
    {
      return ret;
    }

  timeout.tv_sec += waitsec; /* <waitsec> seconds timeout for wait */

  return sem_timedwait(sem, &timeout);
}

static FAR char *read_cep_file(FAR FILE *fp, int32_t offset, size_t len, int *retval)
{
  char   *buf;
  size_t  n = 0;
  int     ret;

  if (fp == NULL)
    {
      ret = -ENOENT;
      goto _err0;
    }

  buf = (char *)malloc(len);
  if (buf == NULL)
    {
      ret = -ENOMEM;
      goto _err0;
    }

  ret = fseek(fp, offset, SEEK_SET);
  if (ret < 0)
    {
      goto _err1;
    }

  n = fread(buf, 1, len, fp);
  if (n <= 0)
    {
      ret = n < 0 ? n : ferror(fp) ? -errno : 0;
      clearerr(fp);
      goto _err1;
    }

  *retval = n;
  cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_CEP, (uint32_t)buf);

  return buf;

  /* send signal to CPU1 in error for just notify completion of read sequence */

_err1:
  free(buf);
_err0:
  *retval = ret;
  cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_CEP, 0);

  return NULL;
}

static void read_backup_file(int *retval)
{
  char *  buf;
  FILE *  fp;
  int32_t offset = 0;
  size_t  n;
  int     ret = 0;

  buf = (char *)malloc(CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE);
  if (buf == NULL)
    {
      ret = -ENOMEM;
      goto _err;
    }

  fp = fopen(CONFIG_CXD56_GNSS_BACKUP_FILENAME, "rb");
  if (fp == NULL)
    {
      free(buf);
      ret = -ENOENT;
      goto _err;
    }

  do
    {
      n = fread(buf, 1, CONFIG_CXD56_GNSS_BACKUP_BUFFER_SIZE, fp);
      if (n <= 0)
        {
          ret = n < 0 ? n : ferror(fp) ? -ENFILE : 0;
          break;
        }
      ret = GD_WriteBuffer(CXD56_CPU1_DATA_TYPE_BACKUP, offset, buf, n);
      if (ret < 0)
        {
          break;
        }
      offset += n;
    }
  while (n > 0);

  fclose(fp);
  free(buf);

  /* Notify the termination of backup sequence by write zero length data */

_err:
  *retval = ret;
  cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_BKUPFILE, 0);
}

/*
 * === CXD56_GNSS_IOCTL_RTK_START ===
 *  Start RTK data output
 */
static int start_rtk_output(FAR struct file *filep, unsigned long arg)
{
  FAR struct cxd56_gnss_rtk_setting *setting;
  
  if (!arg)
    {
      return -EINVAL;
    }
  setting = (FAR struct cxd56_gnss_rtk_setting *)arg;
  
  return GD_RtkStart(setting->interval, setting->gnss, setting->ephOut);
}

/*
 * === CXD56_GNSS_IOCTL_RTK_STOP ===
 *  Stop RTK data output
 */
static int stop_rtk_output(FAR struct file *filep, unsigned long arg)
{
  return GD_RtkStop();
}

/*
 * === CXD56_GNSS_IOCTL_RTK_SET_INTERVAL ===
 *  Set RTK data output interval
 */
static int set_rtk_interval(FAR struct file *filep, unsigned long arg)
{
  int interval = (int)arg;

  return GD_RtkSetOutputInterval(interval);
}

/*
 * === CXD56_GNSS_IOCTL_RTK_GET_INTERVAL ===
 *  Get RTK data output interval setting
 */
static int get_rtk_interval(FAR struct file *filep, unsigned long arg)
{
  int ret;
  int interval;

  if (!arg)
    {
      return -EINVAL;
    }
  ret              = GD_RtkGetOutputInterval(&interval);
  *(uint32_t *)arg = interval;

  return ret;
}

/*
 * === CXD56_GNSS_IOCTL_RTK_SELECT_SATELLITE_SYSTEM ===
 *  Select RTK satellite type
 */
static int select_rtk_satellite(FAR struct file *filep, unsigned long arg)
{
  uint32_t gnss  = (uint32_t)arg;

  return GD_RtkSetGnss(gnss);
}

/*
 * === CXD56_GNSS_IOCTL_RTK_GET_SATELLITE_SYSTEM ===
 *  Get RTK satellite type setting
 */
static int get_rtk_satellite(FAR struct file *filep, unsigned long arg)
{
  int       ret;
  uint32_t  gnss;

  if (!arg)
    {
      return -EINVAL;
    }
  ret              = GD_RtkGetGnss(&gnss);
  *(uint32_t *)arg = gnss;

  return ret;
}

/*
 * === CXD56_GNSS_IOCTL_RTK_SET_EPHEMERIS_ENABLER ===
 *  Set RTK ephemeris notify enable setting
 */
static int set_rtk_ephemeris_enable(FAR struct file *filep, unsigned long arg)
{
  int enable = (int)arg;

  return GD_RtkSetEphNotify(enable);
}

/*
 * === CXD56_GNSS_IOCTL_RTK_GET_EPHEMERIS_ENABLER ===
 *  Get RTK ephemeris notify enable setting
 */
static int get_rtk_ephemeris_enable(FAR struct file *filep, unsigned long arg)
{
  int ret;
  int enable;

  if (!arg)
    {
      return -EINVAL;
    }
  ret              = GD_RtkGetEphNotify(&enable);
  *(uint32_t *)arg = enable;

  return ret;
}

#if !defined(CONFIG_DISABLE_SIGNAL) && \
  (CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0)

/* Common signalqueue */

static void cxd56_gnss_common_signalhandler(uint32_t data, FAR void *userdata)
{
  FAR struct cxd56_gnss_dev_s *priv = (FAR struct cxd56_gnss_dev_s *)userdata;
  uint8_t                      sigtype = CXD56_CPU1_GET_DEV(data);
  int                          issetmask = 0;
  int                          i;
  int                          ret;

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      return;
    }

  for (i = 0; i < CONFIG_CXD56_GNSS_NSIGNALRECEIVERS; i++)
    {
      struct cxd56_gnss_sig_s *sig = &priv->sigs[i];
      if (sig->enable && sig->info.gnsssig == sigtype)
        {
#ifdef CONFIG_CAN_PASS_STRUCTS
          union sigval value;
          value.sival_ptr = &sig->info;
          (void)sigqueue(sig->pid, sig->info.signo, value);
#else
          (void)sigqueue(sig->pid, sig->info.signo, &sig->info);
#endif
          issetmask = 1;
        }
    }

    if (issetmask)
      {
        GD_SetNotifyMask(sigtype, FALSE); 
      }

  sem_post(&priv->devsem);
}

#endif /* if !defined(CONFIG_DISABLE_SIGNAL) && \
          (CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0) */

/* GNSS default handler for signal and poll */

static void cxd56_gnss_default_sighandler(uint32_t data, FAR void *userdata)
{
  FAR struct cxd56_gnss_dev_s *priv = (FAR struct cxd56_gnss_dev_s *)userdata;
  int                          i;
  int                          ret;
  int                          dtype = CXD56_CPU1_GET_DATA(data);

  switch (dtype)
    {
    case CXD56_GNSS_NOTIFY_TYPE_REQCEPDAT:
      priv->cepbuf = read_cep_file(
        priv->cepfp, priv->shared_info.argv[GNSS_ARGS_FILE_OFFSET],
        priv->shared_info.argv[GNSS_ARGS_FILE_LENGTH],
        &priv->shared_info.retval);
      return;

    case CXD56_GNSS_NOTIFY_TYPE_REQCEPBUFFREE:
      if (priv->cepbuf)
        {
          free(priv->cepbuf);
        }
      return;

    case CXD56_GNSS_NOTIFY_TYPE_BOOTCOMP:
      if (priv->num_open == 0)
        {

          /* Post to wait-semaphore in cxd56_gnss_open to notify completion
           * of GNSS core initialization in first device open.
           */

          priv->notify_data = dtype;
          sem_post(&priv->syncsem);
        }
      return;

    case CXD56_GNSS_NOTIFY_TYPE_REQBKUPDAT:
      read_backup_file(&priv->shared_info.retval);
      return;

    case CXD56_GNSS_NOTIFY_TYPE_REQCEPOPEN:
      if (priv->cepfp != NULL)
        {
          fclose(priv->cepfp);
        }
      priv->cepfp = fopen(CONFIG_CXD56_GNSS_CEP_FILENAME, "rb");
      return;

    case CXD56_GNSS_NOTIFY_TYPE_REQCEPCLOSE:
      if (priv->cepfp != NULL)
        {
          fclose(priv->cepfp);
          priv->cepfp = NULL;
        }
      return;

    default:
      break;
    }

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      return;
    }

  for (i = 0; i < CONFIG_CXD56_GNSS_NPOLLWAITERS; i++)
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

#if !defined(CONFIG_DISABLE_SIGNAL) && \
  (CONFIG_CXD56_GNSS_NSIGNALRECEIVERS != 0)
  cxd56_gnss_common_signalhandler(data, userdata);
#endif
}

/* GNSS CPU FIFO API handler */

static void cxd56_gnss_cpufifoapi_signalhandler(uint32_t data,
                                                FAR void *userdata)
{
  FAR struct cxd56_gnss_dev_s *priv = (FAR struct cxd56_gnss_dev_s *)userdata;

  priv->apiret = (int)CXD56_CPU1_GET_DATA(data);
  sem_post(&priv->apiwait);

  return;
}

/* GNSS core CPU Communication FIFO interface */

static int cxd56_gnss_cpufifo_api(FAR struct file *filep, unsigned int api,
                                 unsigned int data)
{
  FAR struct inode            *inode;
  FAR struct cxd56_gnss_dev_s *priv;
  unsigned int                type;
  int                         ret = OK;

  inode = filep->f_inode;
  priv  = (FAR struct cxd56_gnss_dev_s *)inode->i_private;

  type = CXD56_GNSS_CPUFIFOAPI_SET_DATA(api, data);
  cxd56_cpu1sigsend(CXD56_CPU1_DATA_TYPE_CPUFIFOAPI, type);

  sem_wait(&priv->apiwait);
  ret = priv->apiret;

  return ret;
}

/* decide notify type */
static int8_t cxd56_gnss_select_notifytype(off_t fpos, uint32_t *offset)
{
  int8_t type;
  
  if ((fpos >= CXD56_GNSS_READ_OFFSET_LAST_GNSS) && 
      (fpos < CXD56_GNSS_READ_OFFSET_AGPS))
    {
      type = CXD56_CPU1_DATA_TYPE_GNSS;
      *offset = fpos;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_AGPS)
    {
      type = CXD56_CPU1_DATA_TYPE_AGPS;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_RTK)
    {
      type = CXD56_CPU1_DATA_TYPE_RTK;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_GPSEPHEMERIS)
    {
      type = CXD56_CPU1_DATA_TYPE_GPSEPHEMERIS;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_GLNEPHEMERIS)
    {
      type = CXD56_CPU1_DATA_TYPE_GLNEPHEMERIS;
      *offset = 0;
    }
  else if ((fpos == CXD56_GNSS_READ_OFFSET_SPECTRUM) || 
           (fpos == CXD56_GNSS_READ_OFFSET_INFO) )
    {
      type = CXD56_CPU1_DATA_TYPE_SPECTRUM;
      *offset = 0;
    }
  else if (fpos == CXD56_GNSS_READ_OFFSET_PVTLOG)
    {
      type = CXD56_CPU1_DATA_TYPE_PVTLOG;
      *offset = 0;
    }
  else
    {
      type = -1;
    }
  
  return type;
}

/****************************************************************************
 * Name: cxd56_gnss_initialize
 *
 * Description:
 *   initialize gnss device
 *
 ****************************************************************************/
static int cxd56_gnss_initialize(struct cxd56_gnss_dev_s* dev)
{
  int32_t ret = 0;

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_open
 *
 * Description:
 *   Standard character driver open method.
 *
 ****************************************************************************/

static int cxd56_gnss_open(FAR struct file *filep)
{
  FAR struct inode *           inode;
  FAR struct cxd56_gnss_dev_s *priv;
  int                          ret = OK;

  inode = filep->f_inode;
  priv  = (FAR struct cxd56_gnss_dev_s *)inode->i_private;

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->num_open == 0)
    {
      ret = sem_init(&priv->syncsem, 0, 0);
      if (ret < 0)
        {
          goto _err0;
        }

      ret = PM_LoadImage(CXD56_GNSS_GPS_CPUID, CXD56_GNSS_FWNAME);
      if (ret < 0)
        {
          goto _err1;
        }
      ret = PM_StartCpu(CXD56_GNSS_GPS_CPUID, 1);
      if (ret < 0)
        {
          goto _err2;
        }

#ifndef CONFIG_CXD56_GNSS_HOT_SLEEP
      PM_SleepCpu(CXD56_GNSS_GPS_CPUID, PM_SLEEP_MODE_HOT_DISABLE);
#endif

      /* Wait the request from GNSS core to restore backup data,
      * or for completion of initialization of GNSS core here.
      * It is post the semaphore syncsem from cxd56_gnss_default_sighandler.
      */

      ret = wait_gnss_notify(&priv->syncsem, 5);
      if (ret < 0)
        {
          goto _err2;
        }

      ret = GD_WriteBuffer(CXD56_CPU1_DATA_TYPE_INFO, 0, &priv->shared_info,
                            sizeof(priv->shared_info));
      if (ret < 0)
        {
          goto _err2;
        }

      sem_destroy(&priv->syncsem);
    }

    priv->num_open++;
    goto _success;

_err2:
#ifndef CONFIG_CXD56_GNSS_HOT_SLEEP
  PM_SleepCpu(CXD56_GNSS_GPS_CPUID, PM_SLEEP_MODE_HOT_ENABLE);
#endif
  PM_SleepCpu(CXD56_GNSS_GPS_CPUID, PM_SLEEP_MODE_COLD);
_err1:
  sem_destroy(&priv->syncsem);
_err0:
_success:
  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_close
 *
 * Description:
 *   Standard character driver close method.
 *
 ****************************************************************************/

static int cxd56_gnss_close(FAR struct file *filep)
{
  FAR struct inode *          inode;
  FAR struct cxd56_gnss_dev_s *priv;
  int                         ret = OK;

  inode = filep->f_inode;
  priv  = (FAR struct cxd56_gnss_dev_s *)inode->i_private;

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  priv->num_open--;
  if (priv->num_open == 0)
    {
#ifndef CONFIG_CXD56_GNSS_HOT_SLEEP
      PM_SleepCpu(CXD56_GNSS_GPS_CPUID, PM_SLEEP_MODE_HOT_ENABLE);
#endif

      ret = PM_SleepCpu(CXD56_GNSS_GPS_CPUID, PM_SLEEP_MODE_COLD);
      if (ret < 0)
        {
          goto errout;
        }
    }

errout:
  sem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_read
 *
 * Description:
 *   Standard character driver read method.
 *
 ****************************************************************************/

static ssize_t cxd56_gnss_read(FAR struct file *filep, FAR char *buffer,
                               size_t len)
{
  int32_t   ret = 0;
  uint32_t  offset = 0;
  int8_t   type;
  
  if (!buffer)
    {
      ret = -EINVAL;
      goto _err;
    }
  if (len == 0)
    {
      goto _success;
    }
  
  /* setect data type */

  type = cxd56_gnss_select_notifytype(filep->f_pos, &offset);
  if (type < 0)
    {
      ret = -ESPIPE;
      goto _err;
    }
  
  if (type == CXD56_CPU1_DATA_TYPE_GNSS)
    {
      /* Trim len if read would go beyond end of device */
      if ((offset + len) > sizeof(GD_GNSS_POSITION_DATA))
        {
          len = sizeof(GD_GNSS_POSITION_DATA) - offset;
        }
    }
  else if (type == CXD56_CPU1_DATA_TYPE_AGPS)
    {
      if ((offset + len) > sizeof(GD_SUPL_MEASUREMENT_DATA))
        {
          len = sizeof(GD_SUPL_MEASUREMENT_DATA) - offset;
        }
    }

  /* GD_ReadBuffer returns copied data size or negative error code */
  
  ret = GD_ReadBuffer(type, offset, buffer, len);

_err:
_success:
  filep->f_pos = 0;
  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_write
 *
 * Description:
 *   Standard character driver write method.
 ****************************************************************************/
static ssize_t cxd56_gnss_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen)
{
  return -ENOENT;
}

/****************************************************************************
 * Name: cxd56_gnss_ioctl
 *
 * Description:
 *   Standard character driver ioctl method.
 ****************************************************************************/
static int cxd56_gnss_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  static int (*cmd_list[CXD56_GNSS_IOCTL_MAX])(FAR struct file *filep, unsigned long arg) = {
    NULL,                               /* CXD56_GNSS_IOCTL_INVAL = 0 */
    start,                              /* CXD56_GNSS_IOCTL_START */
    stop,                               /* CXD56_GNSS_IOCTL_STOP */
    select_satellite_system,            /* CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM */
    get_satellite_system,               /* CXD56_GNSS_IOCTL_GET_SATELLITE_SYSTEM */
    set_receiver_position_ellipsoidal,  /* CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ELLIPSOIDAL */
    set_receiver_position_orthogonal,   /* CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ORTHOGONAL */
    set_ope_mode,                       /* CXD56_GNSS_IOCTL_SET_OPE_MODE */
    get_ope_mode,                       /* CXD56_GNSS_IOCTL_GET_OPE_MODE */
    set_tcxo_offset,                    /* CXD56_GNSS_IOCTL_SET_TCXO_OFFSET */
    get_tcxo_offset,                    /* CXD56_GNSS_IOCTL_GET_TCXO_OFFSET */
    set_time,                           /* CXD56_GNSS_IOCTL_SET_TIME */
    get_almanac,                        /* CXD56_GNSS_IOCTL_GET_ALMANAC */
    set_almanac,                        /* CXD56_GNSS_IOCTL_SET_ALMANAC */
    get_ephemeris,                      /* CXD56_GNSS_IOCTL_GET_EPHEMERIS */
    set_ephemeris,                      /* CXD56_GNSS_IOCTL_SET_EPHEMERIS */
    save_backup_data,                   /* CXD56_GNSS_IOCTL_SAVE_BACKUP_DATA */
    erase_backup_data,                  /* CXD56_GNSS_IOCTL_ERASE_BACKUP_DATA */
    open_cep_data,                      /* CXD56_GNSS_IOCTL_OPEN_CEP_DATA */
    close_cep_data,                     /* CXD56_GNSS_IOCTL_CLOSE_CEP_DATA */
    check_cep_data,                     /* CXD56_GNSS_IOCTL_CHECK_CEP_DATA */
    get_cep_age,                        /* CXD56_GNSS_IOCTL_GET_CEP_AGE */
    reset_cep_flag,                     /* CXD56_GNSS_IOCTL_RESET_CEP_FLAG */
    start_rtk_output,                   /* CXD56_GNSS_IOCTL_RTK_START */
    stop_rtk_output,                    /* CXD56_GNSS_IOCTL_RTK_STOP */
    set_rtk_interval,                   /* CXD56_GNSS_IOCTL_RTK_SET_INTERVAL */
    get_rtk_interval,                   /* CXD56_GNSS_IOCTL_RTK_GET_INTERVAL */
    select_rtk_satellite,               /* CXD56_GNSS_IOCTL_RTK_SELECT_SATELLITE_SYSTEM */
    get_rtk_satellite,                  /* CXD56_GNSS_IOCTL_RTK_GET_SATELLITE_SYSTEM */
    set_rtk_ephemeris_enable,           /* CXD56_GNSS_IOCTL_RTK_SET_EPHEMERIS_ENABLER */
    get_rtk_ephemeris_enable,           /* CXD56_GNSS_IOCTL_RTK_GET_EPHEMERIS_ENABLER */
    set_acquist_data,                   /* CXD56_GNSS_IOCTL_AGPS_SET_ACQUIST */
    set_frametime,                      /* CXD56_GNSS_IOCTL_AGPS_SET_FRAMETIME */
    set_tau_gps,                        /* CXD56_GNSS_IOCTL_AGPS_SET_TAU_GPS */
    set_time_gps,                       /* CXD56_GNSS_IOCTL_AGPS_SET_TIME_GPS */
    clear_receiver_info,                /* CXD56_GNSS_IOCTL_AGPS_CLEAR_RECEIVER_INFO */
    set_tow_assist,                     /* CXD56_GNSS_IOCTL_AGPS_SET_TOW_ASSIST */
    set_utc_model,                      /* CXD56_GNSS_IOCTL_AGPS_SET_UTC_MODEL */
    control_spectrum,                   /* CXD56_GNSS_IOCTL_SPECTRUM_CONTROL */
    start_test,                         /* CXD56_GNSS_IOCTL_FACTORY_START_TEST */
    stop_test,                          /* CXD56_GNSS_IOCTL_FACTORY_STOP_TEST */
    get_test_result,                    /* CXD56_GNSS_IOCTL_FACTORY_GET_TEST_RESULT */
    set_signal,                         /* CXD56_GNSS_IOCTL_SIGNAL_SET */
    start_pvtlog,                       /* CXD56_GNSS_IOCTL_PVTLOG_START */
    stop_pvtlog,                        /* CXD56_GNSS_IOCTL_PVTLOG_STOP */
    delete_pvtlog,                      /* CXD56_GNSS_IOCTL_PVTLOG_DELETE_LOG */
    get_pvtlog_status,                  /* CXD56_GNSS_IOCTL_PVTLOG_GET_STATUS */
    /* max                                 CXD56_GNSS_IOCTL_MAX */
  };
  FAR struct inode *          inode;
  FAR struct cxd56_gnss_dev_s *priv;
  int ret;

  inode = filep->f_inode;
  priv  = (FAR struct cxd56_gnss_dev_s *)inode->i_private;

  if (cmd <= CXD56_GNSS_IOCTL_INVAL || cmd >= CXD56_GNSS_IOCTL_MAX)
    {
      return -EINVAL;
    }

  ret = sem_wait(&priv->ioctllock);
  if (ret < 0)
    {
      return ret;
    }

  ret = cmd_list[cmd](filep, arg);

  sem_post(&priv->ioctllock);

  return ret;
}

/****************************************************************************
 * Name: cxd56_gnss_poll
 *
 * Description:
 *   Standard character driver poll method.
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int cxd56_gnss_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup)
{
  FAR struct inode *          inode;
  FAR struct cxd56_gnss_dev_s *priv;
  int                         ret = OK;
  int                         i;

  inode = filep->f_inode;
  priv  = (FAR struct cxd56_gnss_dev_s *)inode->i_private;

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

      for (i = 0; i < CONFIG_CXD56_GNSS_NPOLLWAITERS; i++)
        {
          /* Find an unused slot */

          if (priv->fds[i] == NULL)
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              GD_SetNotifyMask(CXD56_CPU1_DEV_GNSS, FALSE);
              break;
            }
        }

      /* No space in priv fds array for poll handling */

      if (i >= CONFIG_CXD56_GNSS_NPOLLWAITERS)
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
 * Name: cxd56_gnss_register
 *
 * Description:
 *   Register the GNSS character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/gps"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int cxd56_gnss_register(FAR const char *devpath)
{
  FAR struct cxd56_gnss_dev_s *priv;
  int                          i;
  int                          ret;
  static struct
  {
    uint8_t                sigtype;
    cxd56_cpu1sighandler_t handler;
  } devsig_table[] = {
    {
      CXD56_CPU1_DATA_TYPE_GNSS,
      cxd56_gnss_default_sighandler
    },
    {
      CXD56_CPU1_DATA_TYPE_AGPS,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_RTK,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_GPSEPHEMERIS,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_GLNEPHEMERIS,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_SPECTRUM,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_PVTLOG,
      cxd56_gnss_common_signalhandler
    },
    {
      CXD56_CPU1_DATA_TYPE_CPUFIFOAPI,
      cxd56_gnss_cpufifoapi_signalhandler
    }
  };

  priv = (FAR struct cxd56_gnss_dev_s *)kmm_malloc(
    sizeof(struct cxd56_gnss_dev_s));
  if (!priv)
    {
      sndbg("Failed to allocate instance\n");
      return -ENOMEM;
    }

  memset(priv, 0, sizeof(struct cxd56_gnss_dev_s));

  ret = sem_init(&priv->devsem, 0, 1);
  if (ret < 0)
    {
      sndbg("Failed to initialize gnss devsem!\n");
      goto _err0;
    }

  ret = sem_init(&priv->apiwait, 0, 0);
  if (ret < 0)
    {
      sndbg("Failed to initialize gnss apiwait!\n");
      goto _err0;
    }

  ret = sem_init(&priv->ioctllock, 0, 1);
  if (ret < 0)
    {
      sndbg("Failed to initialize gnss ioctllock!\n");
      goto _err0;
    }

  ret = cxd56_gnss_initialize(priv);
  if (ret < 0)
    {
      sndbg("Failed to initialize gnss device!\n");
      goto _err0;
    }

  ret = register_driver(devpath, &g_gnssfops, 0666, priv);
  if (ret < 0)
    {
      sndbg("Failed to register driver: %d\n", ret);
      goto _err0;
    }

  for (i = 0; i < sizeof(devsig_table) / sizeof(devsig_table[0]); i++)
    {
      ret = cxd56_cpu1siginit(devsig_table[i].sigtype, priv);
      if (ret < 0)
        {
          sndbg("Failed to initialize ICC for GPS CPU: %d,%d\n", ret,
                devsig_table[i].sigtype);
          goto _err2;
        }
      cxd56_cpu1sigregisterhandler(devsig_table[i].sigtype,
                                   devsig_table[i].handler);
    }

  snvdbg("GNSS driver loaded successfully!\n");

  return ret;

_err2:
  unregister_driver(devpath);
_err0:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: cxd56_gnssinitialize
 ****************************************************************************/

int cxd56_gnssinitialize(FAR const char *devpath)
{
  int ret;

  snvdbg("Initializing GNSS..\n");

  ret = cxd56_gnss_register(devpath);
  if (ret < 0)
    {
      sndbg("Error registering GNSS\n");
    }

  return ret;
}

#endif
