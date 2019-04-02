/****************************************************************************
 * transport_mode/accel_sensor.cxx
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
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <nuttx/sensors/bmi160.h>

#include "accel_sensor.h"
#include "sensing/logical_sensor/transport_mode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* For physical sensor. */

#define TRAM_ACCEL_DEVNAME "/dev/accel0"

#ifndef CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO 15
#endif
#ifndef CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_WM_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_WM_SIGNO 14
#endif
#ifndef CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO 13
#endif

/* Gravity acceleration measurement range fixed at 2g. */

#define TRAM_ACCEL_RANGE 2

/* For error */

#define err(format, ...)    fprintf(stderr, format, ##__VA_ARGS__)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR void *accel_sensor_entry(pthread_addr_t arg)
{
  static int s_entory_function_result = PHYSICAL_SENSOR_ERR_CODE_OK;
  FAR physical_sensor_t *sensor =
    reinterpret_cast<FAR physical_sensor_t *>(arg);

  /* Create instanse of AccelSensorClass. */

  AccelSensorClass *instance = new AccelSensorClass(sensor);

  /* Set sensor signal number. */

  instance->add_signal(CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_WM_SIGNO);
  instance->add_signal(CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO);
  instance->add_signal(CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO);

  /* Start accel sensor process. */

  instance->run();

  /* Delete sensor signal number. */

  instance->delete_signal(CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_WM_SIGNO);
  instance->delete_signal(CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO);
  instance->delete_signal(CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO);

  /* Free instance of AccelSensorClass. */

  free(instance);

  return (void *)&s_entory_function_result;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR physical_sensor_t *AccelSensorCreate(pysical_event_handler_t handler)
{
  return PhysicalSensorCreate(handler,
                              (void *)accel_sensor_entry,
                              "accel_sensor");
}

/*--------------------------------------------------------------------------*/
int AccelSensorOpen(FAR physical_sensor_t *sensor,
                    FAR struct ScuSettings *settings)
{
  return PhysicalSensorOpen(sensor, reinterpret_cast<void *>(settings));
}

/*--------------------------------------------------------------------------*/
int AccelSensorStart(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorStart(sensor);
}

/*--------------------------------------------------------------------------*/
int AccelSensorStop(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorStop(sensor);
}

/*--------------------------------------------------------------------------*/
int AccelSensorClose(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorClose(sensor);
}

/*--------------------------------------------------------------------------*/
int AccelSensorDestroy(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorDestroy(sensor);
}

/****************************************************************************
 * AccelSensorClass
 ****************************************************************************/

int AccelSensorClass::open_sensor()
{
  /* Create oneshot timer */

  int ret = create_timer(&m_timer_id);
  if (ret != 0)
    {
      err("Create timer failed. error = %d\n", ret);
      ASSERT(0);
    }

  m_fd = open(TRAM_ACCEL_DEVNAME, O_RDONLY);
  if (m_fd <= 0)
    {
      return -1;
    }
  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::close_sensor()
{
  delete_timer(m_timer_id);
  return close(m_fd);
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::start_sensor()
{
  return ioctl(m_fd, SCUIOC_START, 0);
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::stop_sensor()
{
  return ioctl(m_fd, SCUIOC_STOP, 0);
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::setup_sensor(FAR void *param)
{
  uint32_t scu_sampling_rate;
  uint32_t power_mode;
  uint32_t output_data_rate;

  FAR struct ScuSettings *settings =
    reinterpret_cast<FAR struct ScuSettings *>(param);
  scu_sampling_rate =
    GET_SCU_ACCEL_SAMPLING_FREQUENCY(settings->samplingrate);

  switch (scu_sampling_rate)
    {
      case ACCEL_SAMPLING_FREQUENCY_MS:
        power_mode       = BMI160_PM_LOWPOWER;
        output_data_rate = BMI160_ACCEL_ODR_25HZ;
        break;

      case ACCEL_SAMPLING_FREQUENCY_CMD:
        power_mode       = BMI160_PM_NORMAL;
        output_data_rate = BMI160_ACCEL_ODR_100HZ;
        break;

      default:
        return -1;
    }

  /* Set power mode. */

  int ret = ioctl(m_fd, SNIOC_SETACCPM, power_mode);
  if (ret < 0)
    {
      err("Accel set power mode error %d\n", ret);
      return ret;
    }

  /* Set output data rate. */

  ret = ioctl(m_fd, SNIOC_SETACCODR, output_data_rate);
  if (ret < 0)
    {
      err("Accel set output data rate error %d\n", ret);
      return ret;
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::setup_scu(FAR void *param)
{
  FAR struct ScuSettings *settings =
  reinterpret_cast<FAR struct ScuSettings *>(param);

  /* Free FIFO. */

  int ret = ioctl(m_fd, SCUIOC_FREEFIFO, 0);
  if (ret < 0)
    {
      err("Accel free FIFO error %d\n", ret);
      return ret;
    }

  /* Set FIFO size. */

  ret = ioctl(m_fd,
              SCUIOC_SETFIFO,
              sizeof(struct accel_t) * settings->fifosize);
  if (ret < 0)
    {
      err("Accel set FIFO size error %d\n", ret);
      return ret;
    }

  /* Set sampling rate */

  ret = ioctl(m_fd, SCUIOC_SETSAMPLE, settings->samplingrate);
  if (ret < 0)
    {
      err("Accel set sequencer sampling rate error %d\n", ret);
      return ret;
    }

  /* Set elements */

  if (settings->elements)
    {
      ret = ioctl(m_fd, SCUIOC_SETELEMENTS, settings->elements);
      if (ret < 0)
        {
          err("Accel set elements error %d\n", ret);
          return ret;
        }
    }

  /* Set MathFunction filter */

  if (settings->mf)
    {
      ret = ioctl(m_fd,
                  SCUIOC_SETFILTER,
                  static_cast<unsigned long>((uintptr_t)settings->mf));
      if (ret < 0)
        {
          err("Accel set MathFunction filter error %d\n", ret);
          return ret;
        }
    }

  /* Set event */

  if (settings->ev)
    {
      settings->ev->signo = CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO;
      settings->ev->arg   = &m_ev_arg;

      ret = ioctl(m_fd,
                  SCUIOC_SETNOTIFY,
                  static_cast<unsigned long>((uintptr_t)settings->ev));
      if (ret < 0)
        {
          err("Accel set event error %d\n", ret);
          return ret;
        }
    }

  /* Set water mark */

  if (settings->wm)
    {
      settings->wm->signo = CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_WM_SIGNO;
      settings->wm->ts    = &m_wm_ts;

      ret = ioctl(m_fd,
                  SCUIOC_SETWATERMARK,
                  static_cast<unsigned long>((uintptr_t)settings->wm));
      if (ret < 0)
        {
          err("Accel set water mark error %d\n", ret);
          return ret;
        }
    }

  /* Reset status flags */

  m_is_first_rise_ev = true;
  m_is_rise = false;

  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::receive_signal(int sig_no, FAR siginfo_t *sig_info)
{
  int ret = -1;

  switch (sig_no)
    {
      case CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO:
        {
          ret = receive_timer_ev();
        }
        break;
 
      case CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_WM_SIGNO:
        {
          ret = receive_scu_wm_ev();
        }
        break;

      case CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_EV_SIGNO:
        {
          ret = receive_scu_math_function_event(sig_info);
        }
        break;

      default:
        break;
    }
  return ret;
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::receive_timer_ev()
{
  int ret = 0;

  /* Still rise, then send rise event */

  if (m_is_rise)
    {
      MemMgrLite::MemHandle dummy;
      uint32_t timestamp = get_timestamp();
      ret = m_handler(ACCEL_EV_MF, timestamp, dummy);
    }
  return ret;
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::receive_scu_wm_ev()
{
  MemMgrLite::MemHandle mh_dst;
  MemMgrLite::MemHandle mh_src;
  FAR char *p_src;
  FAR char *p_dst;

  /* Get segment of memory handle. */

  if (ERR_OK != mh_src.allocSeg(
                  ACCEL_DATA_BUF_POOL,
                  (sizeof(struct accel_t) * ACCEL_WATERMARK_NUM)))
    {
      /* Fatal error occured. */

      err("Fail to allocate segment of memory handle.\n");
      ASSERT(0);
    }
  p_src = reinterpret_cast<char *>(mh_src.getPa());

  if (ERR_OK != mh_dst.allocSeg(
                  ACCEL_DATA_BUF_POOL,
                  (sizeof(accel_float_t) * ACCEL_WATERMARK_NUM)))
    {
      /* Fatal error occured. */

      err("Fail to allocate segment of memory handle.\n");
      ASSERT(0);
    }
  p_dst = reinterpret_cast<char *>(mh_dst.getPa());

  /* Read accelerometer data from driver. */

  read(m_fd, p_src, sizeof(struct accel_t) * ACCEL_WATERMARK_NUM);

  this->convert_data(reinterpret_cast<FAR struct accel_t *>(p_src),
                     reinterpret_cast<FAR accel_float_t *>(p_dst),
                     ACCEL_WATERMARK_NUM);

  /* Notify accelerometer data to sensor manager. */

  this->notify_data(mh_dst);

  return 0;
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::receive_scu_math_function_event(FAR siginfo_t *sig_info)
{
  int ret = 0;

  /* Get info of math-function event. */

  FAR struct scuev_arg_s *scuev =
    (FAR struct scuev_arg_s *)sig_info->si_value.sival_ptr;

  if (scuev->type == SCU_EV_RISE)
    {
      m_is_rise = true;

      if (m_is_first_rise_ev)
        {
          /* Rise event occurs just after sensor starts
           * because of the filter setting.
           * So check later if it is moving or not.
           */

          ret = start_timer(m_timer_id, 1000);
          if (ret != 0)
            {
              err("Start timer failed.\n");
              ASSERT(0);
            }

          m_is_first_rise_ev = false;
        }
      else
        {
          MemMgrLite::MemHandle dummy;
          uint32_t timestamp = get_timestamp();
          m_handler(ACCEL_EV_MF, timestamp, dummy);
        }
    }
  else
    {
      m_is_rise = false;
    }

  return ret;
}


/*--------------------------------------------------------------------------*/
void AccelSensorClass::convert_data(FAR accel_t *p_src,
                                    FAR accel_float_t *p_dst,
                                    int sample_num)
{
  /* Convert the range of data obtained from the sensor (-32768 to 32767)
   * to the range of gravitational acceleration
   * (-TRAM_ACCEL_RANGE to TRAM_ACCEL_RANGE).
   */

  for (int i = 0; i < sample_num; i++, p_src++, p_dst++)
    {
      p_dst->x = (float)p_src->x * TRAM_ACCEL_RANGE / 32768;
      p_dst->y = (float)p_src->y * TRAM_ACCEL_RANGE / 32768;
      p_dst->z = (float)p_src->z * TRAM_ACCEL_RANGE / 32768;
    }
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::notify_data(MemMgrLite::MemHandle &mh_dst)
{
  uint32_t timestamp = get_timestamp();

  return m_handler(ACCEL_EV_WM, timestamp, mh_dst);
};

/*--------------------------------------------------------------------------*/
int AccelSensorClass::create_timer(FAR timer_t *timerid)
{
  struct sigevent notify;

  notify.sigev_notify          = SIGEV_SIGNAL;
  notify.sigev_signo           = CONFIG_EXAMPLES_SENSOR_TRAM_ACCEL_TM_SIGNO;
  notify.sigev_value.sival_int = 0;

  return timer_create(CLOCK_REALTIME, &notify, timerid);
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::start_timer(timer_t timerid, uint32_t milliseconds)
{
  struct itimerspec timer;

  timer.it_value.tv_sec     = milliseconds / 1000;
  timer.it_value.tv_nsec    = milliseconds % 1000 * 1000 * 1000;
  timer.it_interval.tv_sec  = 0;
  timer.it_interval.tv_nsec = 0;

  return timer_settime(timerid, 0, &timer, NULL);
}

/*--------------------------------------------------------------------------*/
int AccelSensorClass::delete_timer(timer_t timerid)
{
  return timer_delete(timerid);
}
