/******************************************************************************
 *
 *  Copyright (C) 2013 Sony Corporation
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
#ifndef _CXD224X_DEBUG_H_
#define _CXD224X_DEBUG_H_

#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

void cxd224x_debug(uint32_t indent, const char *fmt,...);
void cxd224x_dump1(uint32_t indent, const char *str, const uint8_t *p, int32_t len);
void cxd224x_dump2(uint32_t indent, const char *str, const uint8_t *p, int32_t len);

void cxd224x_init_extra_trace_level();
int cxd224x_get_extra_trace_level();
#define EXTRA_TRACE_LEVEL_PACKET_PARSE_EN (1<<0)
#define EXTRA_TRACE_LEVEL_THREAD_LOG_EN   (1<<1)

#if 1
#define CXD224X_TRACE               do{cxd224x_debug(5, "[TRACE] %5d:%-40s", __LINE__, __FUNCTION__);}while(0)
#else
#define CXD224X_TRACE               do{;}while(0)
#endif

#if 1
#define CXD224X_PRINT(fmt,...)      do{cxd224x_debug(0, fmt, ## __VA_ARGS__);}while(0)
#else
#define CXD224X_PRINT(fmt,...)      do{;}while(0)
#endif

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // _CXD224X_DEBUG_H_

