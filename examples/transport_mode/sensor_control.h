/****************************************************************************
 * transport_mode/sensor_control.h
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

#ifndef __TRAM_SENSOR_CONTROL_H
#define __TRAM_SENSOR_CONTROL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Event type. */

#define TRAM_LOGSEN_EV     0x00
#define TRAM_PHYSEN_EV     0x10
#define TRAM_POWER_EV      0x20
#define TRAM_APP_EV        0x30

#define TRAM_APP_FINISH_PHYSICAL_SENSOR (TRAM_APP_EV | 0x01)

#define TRAM_LOGSEN_EV_SCU_CHANGE (TRAM_LOGSEN_EV | 0x01)

#define TRAM_PHYSEN_EV_ACCEL_MF   (TRAM_PHYSEN_EV | 0x01)

#define TRAM_POWER_EV_ACCEL_ON    (TRAM_POWER_EV | 0x0)
#define TRAM_POWER_EV_ACCEL_OFF   (TRAM_POWER_EV | 0x1)
#define TRAM_POWER_EV_MAG_ON      (TRAM_POWER_EV | 0x2)
#define TRAM_POWER_EV_MAG_OFF     (TRAM_POWER_EV | 0x3)
#define TRAM_POWER_EV_PRESS_ON    (TRAM_POWER_EV | 0x4)
#define TRAM_POWER_EV_PRESS_OFF   (TRAM_POWER_EV | 0x5)
#define TRAM_POWER_EV_TEMP_ON     (TRAM_POWER_EV | 0x6)
#define TRAM_POWER_EV_TEMP_OFF    (TRAM_POWER_EV | 0x7)

#define TRAM_APP_EV_PHYSEN_DESTROY (TRAM_APP_EV | 0x01)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int TramOpenSensors(uint8_t cmd_pool_id, mqd_t msg_id);
int TramCloseSensors(void);
int TramStartSensors(void);
int TramStopSensors(void);
int TramSendMathFuncEvent(void);
int TramChangeScuSettings(void);
int TramDestroyPhysicalSensors(void);
int TramPowerControl(int power_ev);

#ifdef CONFIG_EXAMPLES_SENSOR_TRAM_DETAILED_INFO
FAR float *TramGetLikelihood(void);
#endif

#endif /* __TRAM_SENSOR_CONTROL_H */

