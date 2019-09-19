/****************************************************************************
 * transport_mode/magnetometer_sensor.h
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

#ifndef __TRAM_MAGNETOMETER_SENSOR_H
#define __TRAM_MAGNETOMETER_SENSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>
#include <arch/chip/cxd56_scu.h>

#include "physical_sensor.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

FAR physical_sensor_t *MagSensorCreate(pysical_event_handler_t handler);
int MagSensorOpen(FAR physical_sensor_t *sensor);
int MagSensorStart(FAR physical_sensor_t *sensor);
int MagSensorStop(FAR physical_sensor_t *sensor);
int MagSensorClose(FAR physical_sensor_t *sensor);
int MagSensorDestroy(FAR physical_sensor_t *sensor);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

/****************************************************************************
 * Class
 ****************************************************************************/

#ifdef __cplusplus

class MagSensorClass : public PhysicalSensorClass
{
public:

  MagSensorClass(FAR physical_sensor_t *sensor) :
    PhysicalSensorClass(sensor)
    {
      create();
    };

  ~MagSensorClass(){};

private:

  struct mag_float_s
    {
      float x;  /* Northward component, X. */
      float y;  /* Northward component, X. */
      float z;  /* Northward component, X. */
    };
  typedef struct mag_float_s mag_float_t;

  /* Override method */

  int open_sensor();
  int close_sensor();
  int start_sensor();
  int stop_sensor();

  int setup_sensor(FAR void *param);
  int setup_scu(FAR void *param);
  int receive_signal(int sig_no, FAR siginfo_t *sig_info);

  /* Local method */

  int receive_scu_wm_ev();
  void convert_data(FAR struct mag_data_s *p_src,
                    FAR mag_float_t *p_dst,
                    int sample_num);
  int notify_data(MemMgrLite::MemHandle &mh_dst);

  /* Inline method */

  uint32_t get_timestamp()
    {
      return 1000 * m_wm_ts.sec + ((1000 * m_wm_ts.tick) >> 15);
    }

  int m_fd;
  struct scutimestamp_s m_wm_ts;
  int16_t m_adj[3];
};

#endif /* __cplusplus */
#endif /* __TRAM_MAGNETOMETER_SENSOR_H */