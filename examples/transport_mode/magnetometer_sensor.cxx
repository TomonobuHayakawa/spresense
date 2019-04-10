/****************************************************************************
 * transport_mode/magnetometer_sensor.cxx
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
#include <nuttx/sensors/ak09912.h>

#include "magnetometer_sensor.h"
#include "sensing/logical_sensor/transport_mode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* For physical sensor. */

#define TRAM_MAG_DEVNAME "/dev/mag0"

#ifndef CONFIG_EXAMPLES_SENSOR_TRAM_MAG_SIGNO
#  define CONFIG_EXAMPLES_SENSOR_TRAM_MAG_SIGNO 12
#endif

/* Error message */

#define err(format, ...)        fprintf(stderr, format, ##__VA_ARGS__)

/* Device fixed */

#define AK09912_SENSITIVITY      128
#define AK9912_DECIMAL_MAX       32752.0F
#define AK9912_PHISICAL_MAX      4912.0F

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR void *mag_sensor_entry(pthread_addr_t arg)
{
  static int s_entory_function_result = PHYSICAL_SENSOR_ERR_CODE_OK;
  FAR physical_sensor_t *sensor =
    reinterpret_cast<FAR physical_sensor_t *>(arg);

  /* Create instanse of MagSensorClass. */

  MagSensorClass *instance = new MagSensorClass(sensor);

  /* Set sensor signal number. */

  instance->add_signal(CONFIG_EXAMPLES_SENSOR_TRAM_MAG_SIGNO);

  /* Start mangetometer sensor process. */

  instance->run();

  /* Delete sensor signal number. */

  instance->delete_signal(CONFIG_EXAMPLES_SENSOR_TRAM_MAG_SIGNO);

  /* Free instance of MagSensorClass. */

  free(instance);

  return (void *)&s_entory_function_result;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR physical_sensor_t *MagSensorCreate(pysical_event_handler_t handler)
{
  return PhysicalSensorCreate(handler,
                              (void *)mag_sensor_entry,
                              "mag_sensor");
}

/*--------------------------------------------------------------------------*/
int MagSensorOpen(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorOpen(sensor, NULL);
}

/*--------------------------------------------------------------------------*/
int MagSensorStart(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorStart(sensor);
}

/*--------------------------------------------------------------------------*/
int MagSensorStop(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorStop(sensor);
}

/*--------------------------------------------------------------------------*/
int MagSensorClose(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorClose(sensor);
}

/*--------------------------------------------------------------------------*/
int MagSensorDestroy(FAR physical_sensor_t *sensor)
{
  return PhysicalSensorDestroy(sensor);
}

/****************************************************************************
 * MagSensorClass
 ****************************************************************************/

int MagSensorClass::open_sensor()
{
  m_fd = open(TRAM_MAG_DEVNAME, O_RDONLY);
  if (m_fd <= 0)
    {
      return -1;
    }
  return 0;
}

/*--------------------------------------------------------------------------*/
int MagSensorClass::close_sensor()
{
  return close(m_fd);
}

/*--------------------------------------------------------------------------*/
int MagSensorClass::start_sensor()
{
  return ioctl(m_fd, SCUIOC_START, 0);
}

/*--------------------------------------------------------------------------*/
int MagSensorClass::stop_sensor()
{
  return ioctl(m_fd, SCUIOC_STOP, 0);
}

/*--------------------------------------------------------------------------*/
int MagSensorClass::setup_sensor(FAR void *param)
{
  struct ak09912_sensadj_s sensadj;

  /* Get adjust value */

  int ret = ioctl(m_fd, SNIOC_GETADJ, (unsigned long)(uintptr_t)&sensadj);
  if (ret < 0)
    {
      err("Mag get adjust value error %d\n", ret);
      return ret;
    }

  m_adj[0] = sensadj.x + AK09912_SENSITIVITY;
  m_adj[1] = sensadj.y + AK09912_SENSITIVITY;
  m_adj[2] = sensadj.z + AK09912_SENSITIVITY;

  return 0;
}

/*--------------------------------------------------------------------------*/
int MagSensorClass::setup_scu(FAR void *param)
{
  /* Free FIFO. */

  int ret = ioctl(m_fd, SCUIOC_FREEFIFO, 0);
  if (ret < 0)
    {
      err("Mag free FIFO error %d\n", ret);
      return ret;
    }

  /* Set FIFO size to 6 bytes * 8 Hz = 48 */

  ret = ioctl(m_fd,
              SCUIOC_SETFIFO,
              sizeof(struct mag_data_s) * MAG_WATERMARK_NUM);
  if (ret < 0)
    {
      err("Mag set FIFO size error %d\n", ret);
      return ret;
    }

  /* Set sequencer sampling rate 8 Hz
   * (if config CXD56_SCU_PREDIV = 64)
   * 32768 / 64 / (2 ^ 6) = 8
   */

  ret = ioctl(m_fd, SCUIOC_SETSAMPLE, 6);
  if (ret < 0)
    {
      err("Mag set sequencer sampling rate error %d\n", ret);
      return ret;
    }

  /* Set water mark */

  struct scufifo_wm_s wm;
  wm.signo     = CONFIG_EXAMPLES_SENSOR_TRAM_MAG_SIGNO;
  wm.ts        = &m_wm_ts;
  wm.watermark = MAG_WATERMARK_NUM;

  ret = ioctl(m_fd,
              SCUIOC_SETWATERMARK,
              static_cast<unsigned long>((uintptr_t)&wm));
  if (ret < 0)
    {
      err("Mag set water mark error %d\n", ret);
      return ret;
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
int MagSensorClass::receive_signal(int sig_no, FAR siginfo_t *sig_info)
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
int MagSensorClass::receive_scu_wm_ev()
{
  MemMgrLite::MemHandle mh_dst;
  MemMgrLite::MemHandle mh_src;
  FAR char *p_src;
  FAR char *p_dst;

  /* Get segment of memory handle. */

  if (ERR_OK != mh_src.allocSeg(
                  MAG_DATA_BUF_POOL,
                  (sizeof(struct mag_data_s) * MAG_WATERMARK_NUM)))
    {
      /* Fatal error occured. */

      err("Fail to allocate segment of memory handle.\n");
      ASSERT(0);
    }
  p_src = reinterpret_cast<char *>(mh_src.getPa());

  if (ERR_OK != mh_dst.allocSeg(
                  MAG_DATA_BUF_POOL,
                  (sizeof(mag_float_t) * MAG_WATERMARK_NUM)))
    {
      /* Fatal error occured. */

      err("Fail to allocate segment of memory handle.\n");
      ASSERT(0);
    }
  p_dst = reinterpret_cast<char *>(mh_dst.getPa());

  /* Read mangetometer data from driver. */

  read(m_fd, p_src, sizeof(struct mag_data_s) * MAG_WATERMARK_NUM);

  this->convert_data(reinterpret_cast<FAR struct mag_data_s *>(p_src),
                     reinterpret_cast<FAR mag_float_t *>(p_dst),
                     MAG_WATERMARK_NUM);

  /* Notify mangetometer data to sensor manager. */

  this->notify_data(mh_dst);

  return 0;
}

/*--------------------------------------------------------------------------*/
void MagSensorClass::convert_data(FAR struct mag_data_s *p_src,
                                  FAR mag_float_t *p_dst,
                                  int sample_num)
{
  /* Ratio of the value and the magnetic obtained from the sensor. */

  float coef = AK9912_PHISICAL_MAX / AK9912_DECIMAL_MAX;

  for (int i = 0; i < sample_num; i++, p_src++, p_dst++)
    {
      p_dst->x = (float)(((p_src->x * (int32_t)m_adj[0]) >> 8) * coef);
      p_dst->y = (float)(((p_src->y * (int32_t)m_adj[1]) >> 8) * coef);
      p_dst->z = (float)(((p_src->z * (int32_t)m_adj[2]) >> 8) * coef);
    }
}

/*--------------------------------------------------------------------------*/
int MagSensorClass::notify_data(MemMgrLite::MemHandle &mh_dst)
{
  if (m_handler != NULL)
    {
      uint32_t timestamp = get_timestamp();
      return m_handler(0, timestamp, mh_dst);
    }

  return 0;
};
