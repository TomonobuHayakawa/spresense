/****************************************************************************
 * transport_mode/pressure_sensor.cxx
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

#include "pressure_sensor.h"
#include "sensing/logical_sensor/barometer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TRAM_PRESSURE_DEVNAME "/dev/press0"

#ifndef CONFIG_EXAMPLES_SENSOR_TRAM_PRESS_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_TRAM_PRESS_SIGNO 11
#endif

/* Error message */

#define err(format, ...)        fprintf(stderr, format, ##__VA_ARGS__)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR void *press_sensor_entry(pthread_addr_t arg)
{
  static int s_entory_function_result = PHYSICAL_SENSOR_ERR_CODE_OK;
  FAR physical_sensor_t *sensor =
    reinterpret_cast<FAR physical_sensor_t *>(arg);

  /* Create instanse of PressSensorClass. */

  PressSensorClass *instance = new PressSensorClass(sensor);

  /* Set sensor signal number. */

  instance->add_signal(CONFIG_EXAMPLES_SENSOR_TRAM_PRESS_SIGNO);

  /* Start pressure sensor process. */

  instance->run();

  /* Delete sensor signal number. */

  instance->delete_signal(CONFIG_EXAMPLES_SENSOR_TRAM_PRESS_SIGNO);

  /* Free instance of PressSensorClass. */

  free(instance);

  return (void *)&s_entory_function_result;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR physical_sensor_t *PressSensorCreate(pysical_event_handler_t handler)
{
  return PhysicalSensorCreate(handler,
                              (void *)press_sensor_entry,
                              "press_sensor");
}

/*--------------------------------------------------------------------------*/
int PressSensorOpen(FAR physical_sensor_t *sensor,
                    FAR struct bmp280_press_adj_s *press_adj)
{
  return PhysicalSensorOpen(sensor, reinterpret_cast<void *>(press_adj));
}

/*--------------------------------------------------------------------------*/
int PressSensorStart(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorStart(sensor);
}

/*--------------------------------------------------------------------------*/
int PressSensorStop(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorStop(sensor);
}

/*--------------------------------------------------------------------------*/
int PressSensorClose(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorClose(sensor);
}

/*--------------------------------------------------------------------------*/
int PressSensorDestroy(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorDestroy(sensor);
}

/****************************************************************************
 * PressSensorClass
 ****************************************************************************/

int PressSensorClass::open_sensor()
{
  m_fd = open(TRAM_PRESSURE_DEVNAME, O_RDONLY);
  if (m_fd <= 0)
    {
      return -1;
    }
  return 0;
}

/*--------------------------------------------------------------------------*/
int PressSensorClass::close_sensor()
{
  return close(m_fd);
}

/*--------------------------------------------------------------------------*/
int PressSensorClass::start_sensor()
{
  return ioctl(m_fd, SCUIOC_START, 0);
}

/*--------------------------------------------------------------------------*/
int PressSensorClass::stop_sensor()
{
  return ioctl(m_fd, SCUIOC_STOP, 0);
}

/*--------------------------------------------------------------------------*/
int PressSensorClass::setup_sensor(FAR void *param)
{
  struct bmp280_press_adj_s *press_adj =
    reinterpret_cast<FAR struct bmp280_press_adj_s *>(param);

  /* Set standby duration */

  int ret = ioctl(m_fd, SNIOC_SETSTB, BMP280_STANDBY_63_MS);
  if (ret < 0)
    {
      err("Press set standby duration error %d\n", ret);
      return ret;
    }

  /* Get adjust value */

  ret = ioctl(m_fd, SNIOC_GETADJ, (unsigned long)(uintptr_t)press_adj);
  if (ret < 0)
    {
      err("Press get adjust value error %d\n", ret);
      return ret;
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
int PressSensorClass::setup_scu(FAR void *param)
{
  /* Free FIFO. */

  int ret = ioctl(m_fd, SCUIOC_FREEFIFO, 0);
  if (ret < 0)
    {
      err("Press free FIFO error %d\n", ret);
      return ret;
    }

  /* Set FIFO size to 3 bytes * 8 Hz = 24 */

  ret = ioctl(m_fd,
              SCUIOC_SETFIFO,
              sizeof(struct bmp280_meas_s) * PRESSURE_WATERMARK_NUM);
  if (ret < 0)
    {
      err("Press set FIFO size error %d\n", ret);
      return ret;
    }

  /* Set sequencer sampling rate 8 Hz
   * (if config CXD56_SCU_PREDIV = 64)
   * 32768 / 64 / (2 ^ 6) = 8
   */

  ret = ioctl(m_fd, SCUIOC_SETSAMPLE, 6);
  if (ret < 0)
    {
      err("Press set sequencer sampling rate error %d\n", ret);
      return ret;
    }

  /* Set water mark */

  struct scufifo_wm_s wm;
  wm.signo     = CONFIG_EXAMPLES_SENSOR_TRAM_PRESS_SIGNO;
  wm.ts        = &m_wm_ts;
  wm.watermark = PRESSURE_WATERMARK_NUM;

  ret = ioctl(m_fd,
              SCUIOC_SETWATERMARK,
              static_cast<unsigned long>((uintptr_t)&wm));
  if (ret < 0)
    {
      err("Press set water mark error %d\n", ret);
      return ret;
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
int PressSensorClass::receive_signal(int sig_no, FAR siginfo_t *sig_info)
{
  int ret = -1;

  switch (sig_no)
    {
      case CONFIG_EXAMPLES_SENSOR_TRAM_MAG_SIGNO:
        {
          ret = receive_scu_wm_ev();
        }
        break;

      default:
        break;
    }
  return ret;
}

/*--------------------------------------------------------------------------*/
int PressSensorClass::receive_scu_wm_ev()
{
  MemMgrLite::MemHandle mh_dst;
  MemMgrLite::MemHandle mh_src;
  FAR char *p_src;
  FAR char *p_dst;

  /* Get segment of memory handle. */

  if (ERR_OK != mh_src.allocSeg(
                  MAG_DATA_BUF_POOL,
                  (sizeof(struct bmp280_meas_s) * PRESSURE_WATERMARK_NUM)))
    {
      /* Fatal error occured. */

      err("Fail to allocate segment of memory handle.\n");
      ASSERT(0);
    }
  p_src = reinterpret_cast<char *>(mh_src.getPa());

  if (ERR_OK != mh_dst.allocSeg(
                  MAG_DATA_BUF_POOL,
                  (sizeof(uint32_t) * PRESSURE_WATERMARK_NUM)))
    {
      /* Fatal error occured. */

      err("Fail to allocate segment of memory handle.\n");
      ASSERT(0);
    }
  p_dst = reinterpret_cast<char *>(mh_dst.getPa());

  /* Read mangetometer data from driver. */

  read(m_fd, p_src, sizeof(struct bmp280_meas_s) * PRESSURE_WATERMARK_NUM);

  this->convert_data(reinterpret_cast<FAR struct bmp280_meas_s *>(p_src),
                     reinterpret_cast<FAR int32_t *>(p_dst),
                     PRESSURE_WATERMARK_NUM);

  /* Notify mangetometer data to sensor manager. */

  this->notify_data(mh_dst);

  return 0;
}

/*--------------------------------------------------------------------------*/
void PressSensorClass::convert_data(FAR struct bmp280_meas_s *p_src,
                                  FAR int32_t *p_dst,
                                  int sample_num)
{
  for (int i = 0; i < sample_num; i++, p_src++, p_dst++)
    {
      /* Get the raw pressure data from registers. Unit is hPa. */

      *p_dst = (int32_t)((((uint32_t)(p_src->msb)) << 12)
        | ((uint32_t)p_src->lsb << 4)
        | ((uint32_t)p_src->xlsb >> 4));
    }
}

/*--------------------------------------------------------------------------*/
int PressSensorClass::notify_data(MemMgrLite::MemHandle &mh_dst)
{
  if (m_handler != NULL)
    {
      uint32_t timestamp = get_timestamp();
      return m_handler(0, timestamp, mh_dst);
    }

  return 0;
};
