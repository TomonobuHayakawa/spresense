/******************************************************************************
 *
 *  Copyright (C) 2012 Sony Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/
#include <stdlib.h>
#include <stdint.h>

#ifndef _LOG_H_
#define _LOG_H_

#ifdef __cplusplus
 extern "C" {
#endif

     int __android_log_print(int prio, const char *tag,  const char *fmt, ...);
     int property_get(const char *key, char *value, const char *default_value);
     int set_log_level( int n );

     extern int g_log_time_info;
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" unsigned char appl_trace_level;
#endif

#endif
/*  end of log.h */
