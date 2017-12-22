/****************************************************************************
 * include/sensing/sensor_ecode.h
 *
 *   Copyright (C) 2017 Sony Corporation
 *   Author: Tomonobu Hayakawa <Tomonobu.Hayakawa@sony.com>
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

#ifndef __INCLUDE_SENSING_SENSOR_ECODE_H
#define __INCLUDE_SENSING_SENSOR_ECODE_H

/* Error codes from sensorutils */

#define SENSOR_OK 0
#define SENSOR_DSP_LOAD_ERROR 1
#define SENSOR_DSP_BOOT_ERROR 2
#define SENSOR_DSP_VERSION_ERROR 3
#define SENSOR_DSP_INIT_ERROR 4
#define SENSOR_DSP_EXEC_ERROR 5
#define SENSOR_DSP_UNLOAD_ERROR 5
#define SENSOR_MEMHANDLE_ALLOC_ERROR 6
#define SENSOR_MEMHANDLE_FREE_ERROR 7
#define SENSOR_QUEUE_PUSH_ERROR 8
#define SENSOR_QUEUE_POP_ERROR 9
#define SENSOR_QUEUE_MISSING_ERROR 10
#define SENSOR_REQUIRED_SENSOR_NOT_ACTIVE 11
#define SENSOR_REQUIRED_SENSOR_STILL_ACTIVE 12
#define SENSOR_NOTIFICATION_DST_UNDEFINED 13
#define SENSOR_TASK_CREATE_ERROR 14
#define SENSOR_PARAM_ERROR 15
#define SENSOR_STATE_ERROR 16

#endif /* __INCLUDE_SENSING_SENSOR_ECODE_H */

