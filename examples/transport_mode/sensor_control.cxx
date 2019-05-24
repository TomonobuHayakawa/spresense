/****************************************************************************
 * transport_mode/sensor_control.cxx
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

#include "accel_sensor.h"
#include "magnetometer_sensor.h"
#include "pressure_sensor.h"
#include "temperature_sensor.h"
#include "sensor_control.h"
#include "sensing/sensor_api.h"
#include "sensing/logical_sensor/barometer.h"
#include "sensing/logical_sensor/transport_mode.h"
#include "memutils/memory_manager/MemHandle.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Error message */

#define err(format, ...)        fprintf(stderr, format, ##__VA_ARGS__)

/* Value check macros */

#define CHECK_FUNC_RET(func)                                            \
  do {                                                                  \
    if ((func) < 0) {                                                   \
      err("return error, %s, %d\n", __FUNCTION__, __LINE__);            \
      return false;                                                     \
    }                                                                   \
  } while(0)

#define CHECK_NULL_RET(expr)                                            \
  do {                                                                  \
    if (expr == NULL) {                                                 \
      err("check failed. %s, %d\n", __FUNCTION__, __LINE__);            \
      return false;                                                     \
    }                                                                   \
  } while(0)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Sensor instances */

static FAR TramClass         *s_tram_ins     = NULL;
static FAR BarometerClass    *s_bar_ins      = NULL;
static FAR physical_sensor_t *p_accel_sensor = NULL;
static FAR physical_sensor_t *p_mag_sensor   = NULL;
static FAR physical_sensor_t *p_press_sensor = NULL;
static FAR physical_sensor_t *p_temp_sensor  = NULL;

#ifdef CONFIG_EXAMPLES_SENSOR_TRAM_DETAILED_INFO
static  MemMgrLite::MemHandle s_likelihood;
#endif

static mqd_t s_ev_msg_id = (mqd_t)-1;
static int s_pw_status = 0;
static int s_active_status = 0;

/****************************************************************************
 * Callback Function
 ****************************************************************************/

static int accel_read_callback(uint32_t ev_type,
                               uint32_t timestamp,
                               MemMgrLite::MemHandle &mh)
{
  if (ACCEL_EV_WM == ev_type)
    {
      sensor_command_data_mh_t packet;
      packet.header.size = 0;
      packet.header.code = SendData;
      packet.self        = accelID;
      packet.time        = timestamp;
      packet.fs          = ACCEL_SAMPLING_FREQUENCY_CMD;
      packet.size        = ACCEL_WATERMARK_NUM;
      packet.mh          = mh;

      SS_SendSensorDataMH(&packet);
    }
  else if (ACCEL_EV_MF == ev_type)
    {
      sensor_command_data_t packet;
      packet.header.size = 0;
      packet.header.code = SendData;
      packet.self        = accelID;
      packet.time        = 0;
      packet.fs          = 0;
      packet.size        = 0;
      packet.is_ptr      = false;
      packet.data        = 1;

      SS_SendSensorData(&packet);
    }
  return 0;
}

/*--------------------------------------------------------------------------*/
static int mag_read_callback(uint32_t ev_type,
                             uint32_t timestamp,
                             MemMgrLite::MemHandle &mh)
{
  sensor_command_data_mh_t packet;
  packet.header.size = 0;
  packet.header.code = SendData;
  packet.self        = magID;
  packet.time        = timestamp;
  packet.fs          = MAG_SAMPLING_FREQUENCY;
  packet.size        = MAG_WATERMARK_NUM;
  packet.mh          = mh;

  SS_SendSensorDataMH(&packet);

  return 0;
}

/*--------------------------------------------------------------------------*/
static int press_read_callback(uint32_t ev_type,
                               uint32_t timestamp,
                               MemMgrLite::MemHandle &mh)
{
  sensor_command_data_mh_t packet;
  packet.header.size = 0;
  packet.header.code = SendData;
  packet.self        = pressureID;
  packet.time        = timestamp;
  packet.fs          = PRESSURE_SAMPLING_FREQUENCY;
  packet.size        = PRESSURE_WATERMARK_NUM;
  packet.mh          = mh;

  SS_SendSensorDataMH(&packet);

  return 0;
}

/*--------------------------------------------------------------------------*/
static int temp_read_callback(uint32_t ev_type,
                               uint32_t timestamp,
                               MemMgrLite::MemHandle &mh)
{
  sensor_command_data_mh_t packet;
  packet.header.size = 0;
  packet.header.code = SendData;
  packet.self        = tempID;
  packet.time        = timestamp;
  packet.fs          = TEMPERATURE_SAMPLING_FREQUENCY;
  packet.size        = TEMPERATURE_WATERMARK_NUM;
  packet.mh          = mh;

  SS_SendSensorDataMH(&packet);

  return 0;
}

/*--------------------------------------------------------------------------*/
static bool accel_power_ctrl_callback(bool is_on)
{
  uint8_t ev;

  if (is_on)
    {
      ev = TRAM_POWER_EV_ACCEL_ON;
    }
  else
    {
      ev = TRAM_POWER_EV_ACCEL_OFF;
    }

  mq_send(s_ev_msg_id, (char *)&ev, sizeof(uint8_t), 0);

  return true;
}

/*--------------------------------------------------------------------------*/
static int accel_power_ctrl(bool is_on)
{
  if (is_on)
    {
      FAR ScuSettings *settings = TramGetAccelScuSettings(s_tram_ins);
      CHECK_NULL_RET(settings);

      CHECK_FUNC_RET(AccelSensorOpen(p_accel_sensor, settings));
      CHECK_FUNC_RET(AccelSensorStart(p_accel_sensor));
    }
  else
    {
      CHECK_FUNC_RET(AccelSensorStop(p_accel_sensor));
      CHECK_FUNC_RET(AccelSensorClose(p_accel_sensor));
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
static bool magne_power_ctrl_callback(bool is_on)
{
  uint8_t ev;

  if (is_on)
    {
      ev = TRAM_POWER_EV_MAG_ON;
    }
  else
    {
      ev = TRAM_POWER_EV_MAG_OFF;
    }

  mq_send(s_ev_msg_id, (char *)&ev, sizeof(uint8_t), 0);

  return true;
}

/*--------------------------------------------------------------------------*/
static int magne_power_ctrl(bool is_on)
{
  if (is_on)
    {
      CHECK_FUNC_RET(MagSensorOpen(p_mag_sensor));
      CHECK_FUNC_RET(MagSensorStart(p_mag_sensor));
    }
  else
    {
      CHECK_FUNC_RET(MagSensorStop(p_mag_sensor));
      CHECK_FUNC_RET(MagSensorClose(p_mag_sensor));
    }

  return 0;
}
/*--------------------------------------------------------------------------*/
static bool pressure_power_ctrl_callback(bool is_on)
{
  uint8_t ev;

  if (is_on)
    {
      ev = TRAM_POWER_EV_PRESS_ON;
    }
  else
    {
      ev = TRAM_POWER_EV_PRESS_OFF;
    }

  mq_send(s_ev_msg_id, (char *)&ev, sizeof(uint8_t), 0);

  return true;
}

/*--------------------------------------------------------------------------*/
static int pressure_power_ctrl(bool is_on)
{
  if (is_on)
    {
      struct bmp280_press_adj_s press_adj;
      CHECK_FUNC_RET(PressSensorOpen(p_press_sensor, &press_adj));
      s_bar_ins->setAdjustParam(&press_adj);
      CHECK_FUNC_RET(PressSensorStart(p_press_sensor));
    }
  else
    {
      CHECK_FUNC_RET(PressSensorStop(p_press_sensor));
      CHECK_FUNC_RET(PressSensorClose(p_press_sensor));
    }

  return 0;
}
/*--------------------------------------------------------------------------*/
static bool temperature_power_ctrl_callback(bool is_on)
{
  uint8_t ev;

  if (is_on)
    {
      ev = TRAM_POWER_EV_TEMP_ON;
    }
  else
    {
      ev = TRAM_POWER_EV_TEMP_OFF;
    }

  mq_send(s_ev_msg_id, (char *)&ev, sizeof(uint8_t), 0);

  return true;
}

/*--------------------------------------------------------------------------*/
static int temperature_power_ctrl(bool is_on)
{
  if (is_on)
    {
      struct bmp280_temp_adj_s temp_adj;
      CHECK_FUNC_RET(TempSensorOpen(p_temp_sensor, &temp_adj));
      s_bar_ins->setAdjustParam(&temp_adj);
      CHECK_FUNC_RET(TempSensorStart(p_temp_sensor));
    }
  else
    {
      CHECK_FUNC_RET(TempSensorStop(p_temp_sensor));
      CHECK_FUNC_RET(TempSensorClose(p_temp_sensor));
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
static bool bar_receive_data(sensor_command_data_mh_t& data)
{
  CHECK_FUNC_RET(BarometerWrite(s_bar_ins, &data));

  return true;
}

/*--------------------------------------------------------------------------*/
static bool baro_power_ctrl_callback(bool is_data)
{
  if (is_data)
    {
      CHECK_FUNC_RET(BarometerStart(s_bar_ins));
    }
  else
    {
      CHECK_FUNC_RET(BarometerStop(s_bar_ins));
    }

  return true;
}

/*--------------------------------------------------------------------------*/
static bool tram_receive_event(sensor_command_data_t& data)
{
  /* Ignore event from mathfunc... */

  return true;
}

/*--------------------------------------------------------------------------*/
static bool tram_receive_data(sensor_command_data_mh_t& data)
{
  CHECK_FUNC_RET(TramWrite(s_tram_ins, &data));

  return true;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int create_physical_sensor(void)
{
  int ret = 0;

  /* Accelerator */

  p_accel_sensor = AccelSensorCreate(accel_read_callback);
  if (p_accel_sensor == NULL)
    {
      err("AccelSensorCreate() failure. %d\n", ret);
      return -1;
    }
  s_active_status |= 1 << accelID;

  /* Magnetmeter */

  p_mag_sensor = MagSensorCreate(mag_read_callback);
  if (p_mag_sensor == NULL)
    {
      err("MagSensorCreate() failure. %d\n", ret);
      return -1;
    }
  s_active_status |= 1 << magID;
  
  /* Pressure */

  p_press_sensor = PressSensorCreate(press_read_callback);
  if (p_mag_sensor == NULL)
    {
      err("PressSensorCreate() failure. %d\n", ret);
      return -1;
    }
  s_active_status |= 1 << pressureID;

  /* Temperature */

  p_temp_sensor = TempSensorCreate(temp_read_callback);
  if (p_temp_sensor == NULL)
    {
      err("TempSensorCreate() failure. %d\n", ret);
      return -1;
    }
  s_active_status |= 1 << tempID;

  return 0;
}

/*--------------------------------------------------------------------------*/
static void destroy_physical_sensor(void)
{
  uint8_t ev = TRAM_APP_EV_PHYSEN_DESTROY;
  mq_send(s_ev_msg_id, (char *)&ev, sizeof(uint8_t), 0);

  while(s_active_status != 0)
    {
      sleep(1);
    }
}

/*--------------------------------------------------------------------------*/
static int init_logical_sensor(uint8_t cmd_pool_id)
{
  int ret;

  /* Tram */

  s_tram_ins = TramCreate((MemMgrLite::PoolId)cmd_pool_id);

#ifdef CONFIG_EXAMPLES_SENSOR_TRAM_DETAILED_INFO

  /* Make buffer for likelihood */

  if (s_likelihood.allocSeg(LIKELIHOOD_BUF_POOL,
                            sizeof(float) * TRAM_NUMBER_OF_MODES)
        != ERR_OK)
    {
      err("allocSeg() failure.\n");
      return -1;
    }

  ret = TramOpen(s_tram_ins, (float*)s_likelihood.getPa());
#else
  ret = TramOpen(s_tram_ins);
#endif

  if (ret < 0)
    {
      err("TramOpen() failure. %d\n", ret);
      return -1;
    }

  /* Barometer */

  s_bar_ins = BarometerCreate();
  ret = BarometerOpen(s_bar_ins);
  if (ret < 0)
    {
      err("BarometerOpen() failure. %d\n", ret);
      return -1;
    }

  return 0;
}

/*--------------------------------------------------------------------------*/
static void finish_logical_sensor(void)
{
  TramClose(s_tram_ins);
  BarometerClose(s_bar_ins);

#ifdef CONFIG_EXAMPLES_SENSOR_TRAMLITE_DETAILED_INFO
  s_likelihood.freeSeg();
#endif
}

/*--------------------------------------------------------------------------*/
static void regist_physical_sensor(void)
{
  sensor_command_register_t reg;

  reg.header.size       = 0;
  reg.header.code       = ResisterClient;
  reg.self              = accelID;
  reg.subscriptions     = 0;
  reg.callback          = NULL;
  reg.callback_mh       = NULL;
  reg.callback_pw       = accel_power_ctrl_callback;
  SS_SendSensorResister(&reg);

  reg.header.size       = 0;
  reg.header.code       = ResisterClient;
  reg.self              = magID;
  reg.subscriptions     = 0;
  reg.callback          = NULL;
  reg.callback_mh       = NULL;
  reg.callback_pw       = magne_power_ctrl_callback;
  SS_SendSensorResister(&reg);

  reg.header.size       = 0;
  reg.header.code       = ResisterClient;
  reg.self              = pressureID;
  reg.subscriptions     = 0;
  reg.callback          = NULL;
  reg.callback_mh       = NULL;
  reg.callback_pw       = pressure_power_ctrl_callback;
  SS_SendSensorResister(&reg);

  reg.header.size       = 0;
  reg.header.code       = ResisterClient;
  reg.self              = tempID;
  reg.subscriptions     = 0;
  reg.callback          = NULL;
  reg.callback_mh       = NULL;
  reg.callback_pw       = temperature_power_ctrl_callback;
  SS_SendSensorResister(&reg);
}

/*--------------------------------------------------------------------------*/
static void release_physical_sensor(void)
{
  sensor_command_release_t rel;

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = accelID;
  SS_SendSensorRelease(&rel);

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = magID;
  SS_SendSensorRelease(&rel);

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = pressureID;
  SS_SendSensorRelease(&rel);

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = tempID;
  SS_SendSensorRelease(&rel);
}

/*--------------------------------------------------------------------------*/
static void regist_logical_sensor(void)
{
  sensor_command_register_t reg;

  reg.header.size       = 0;
  reg.header.code       = ResisterClient;
  reg.self              = barometerID;
  reg.subscriptions     = (0x01 << pressureID) | (0x01 << tempID);
  reg.callback          = NULL;
  reg.callback_mh       = bar_receive_data;
  reg.callback_pw       = baro_power_ctrl_callback;
  SS_SendSensorResister(&reg);

  reg.header.size       = 0;
  reg.header.code       = ResisterClient;
  reg.self              = tramID;
  reg.subscriptions     = (0x01 << accelID) |
                          (0x01 << magID)   |
                          (0x01 << barometerID);
  reg.callback          = tram_receive_event;
  reg.callback_mh       = tram_receive_data;
  reg.callback_pw       = NULL;
  SS_SendSensorResister(&reg);
}

/*--------------------------------------------------------------------------*/
static void release_logical_sensor(void)
{
  sensor_command_release_t rel;

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = tramID;
  SS_SendSensorRelease(&rel);

  rel.header.size = 0;
  rel.header.code = ReleaseClient;
  rel.self        = barometerID;
  SS_SendSensorRelease(&rel);
}

/****************************************************************************
 * External Interface
 ****************************************************************************/
int TramOpenSensors(uint8_t cmd_pool_id, mqd_t msg_id)
{
  /* Clear power status of physical sensor */

  s_pw_status = 0;
  s_active_status = 0;

  /* Set message id for event occure */

  s_ev_msg_id = msg_id;

  /* Initialize physical sensor */

  create_physical_sensor();
  regist_physical_sensor();

  /* Initialize logical sensor */

  init_logical_sensor(cmd_pool_id);
  regist_logical_sensor();

  return 0;
}

/*--------------------------------------------------------------------------*/
int TramCloseSensors(void)
{
  /* Finalize logical sensors */

  release_logical_sensor();
  finish_logical_sensor();

  /* Finalize physical sensors */

  release_physical_sensor();
  destroy_physical_sensor();

  return 0;
}

/*--------------------------------------------------------------------------*/
int TramStartSensors(void)
{
  int ret = TramStart(s_tram_ins);
  if (ret != SS_ECODE_OK)
    {
      return ret;
    }

  /* Wait physical sensor start. */

  while(s_pw_status == 0)
    {
      sleep(1);
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
int TramStopSensors(void)
{
  int ret = TramStop(s_tram_ins);
  if (ret != SS_ECODE_OK)
    {
      return ret;
    }

  /* Wait physical sensor start. */

  while(s_pw_status != 0)
    {
      sleep(1);
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
int TramSendMathFuncEvent(void)
{
  return TramHandleEvent(s_tram_ins, MathFuncEvent);
}

/*--------------------------------------------------------------------------*/
int TramChangeScuSettings(void)
{
  FAR struct ScuSettings* settings = TramGetAccelScuSettings(s_tram_ins);
  if (settings == NULL)
    {
      err("TramGetAccelScuSettings() failure.\n");
      return -1;
    }

  /* Restart accelerometer sensor */

  CHECK_FUNC_RET(AccelSensorStop(p_accel_sensor));
  CHECK_FUNC_RET(AccelSensorClose(p_accel_sensor));
  CHECK_FUNC_RET(AccelSensorOpen(p_accel_sensor, settings));
  CHECK_FUNC_RET(AccelSensorStart(p_accel_sensor));

  return 0;
}

/*--------------------------------------------------------------------------*/
int TramDestroyPhysicalSensors(void)
{
  int ret;

  ret = AccelSensorDestroy(p_accel_sensor);
  if (ret != PHYSICAL_SENSOR_ERR_CODE_OK)
    {
      err("AccelSensorDestroy() failure. %d\n", ret);
      return -1;
    }
  s_active_status &= ~(1 << accelID);

  ret = MagSensorDestroy(p_mag_sensor);
  if (ret != PHYSICAL_SENSOR_ERR_CODE_OK)
    {
      err("MagSensorDestroy() failure. %d\n", ret);
      return -1;
    }
  s_active_status &= ~(1 << magID);

  ret = PressSensorDestroy(p_press_sensor);
  if (ret != PHYSICAL_SENSOR_ERR_CODE_OK)
    {
      err("PressSensorDestroy() failure. %d\n", ret);
      return -1;
    }
  s_active_status &= ~(1 << pressureID);

  ret = TempSensorDestroy(p_temp_sensor);
  if (ret != PHYSICAL_SENSOR_ERR_CODE_OK)
    {
      err("TempSensorDestroy() failure. %d\n", ret);
      return -1;
    }
  s_active_status &= ~(1 << tempID);

  return 0;
}

/*--------------------------------------------------------------------------*/
int TramPowerControl(int power_ev)
{
  int ret = 0;

  switch (power_ev)
    {
      case TRAM_POWER_EV_ACCEL_ON:
        {
          ret = accel_power_ctrl(true);
          if(ret == 0)
            {
              s_pw_status |= 1 << accelID;
            }
        }
        break;

      case TRAM_POWER_EV_ACCEL_OFF:
        {
          ret = accel_power_ctrl(false);
          if(ret == 0)
            {
              s_pw_status &= ~(1 << accelID);
            }
        }
        break;

      case TRAM_POWER_EV_MAG_ON:
        {
          ret = magne_power_ctrl(true);
          if(ret == 0)
            {
              s_pw_status |= 1 << magID;
            }
        }
        break;

      case TRAM_POWER_EV_MAG_OFF:
        {
          ret = magne_power_ctrl(false);
          if(ret == 0)
            {
              s_pw_status &= ~(1 << magID);
            }
        }
        break;

      case TRAM_POWER_EV_PRESS_ON:
        {
          ret = pressure_power_ctrl(true);
          if(ret == 0)
            {
              s_pw_status |= 1 << pressureID;
            }
        }
        break;

      case TRAM_POWER_EV_PRESS_OFF:
        {
          ret = pressure_power_ctrl(false);
          if(ret == 0)
            {
              s_pw_status &= ~(1 << pressureID);
            }
        }
        break;

      case TRAM_POWER_EV_TEMP_ON:
        {
          ret = temperature_power_ctrl(true);
          if(ret == 0)
            {
              s_pw_status |= 1 << tempID;
            }
        }
        break;

      case TRAM_POWER_EV_TEMP_OFF:
        {
          ret = temperature_power_ctrl(false);
          if(ret == 0)
            {
              s_pw_status &= ~(1 << tempID);
            }
        }
        break;

      default:
        ret = -1;
        break;
    }

  return ret;
}

/*--------------------------------------------------------------------------*/
#ifdef CONFIG_EXAMPLES_SENSOR_TRAM_DETAILED_INFO
FAR float *TramGetLikelihood(void)
{
  return (FAR float *)s_likelihood.getVa();
}
#endif
