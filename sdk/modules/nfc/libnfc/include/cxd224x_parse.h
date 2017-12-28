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
#ifndef _CXD224X_PARSE_H_
#define _CXD224X_PARSE_H_

#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define CXD224X_DATA_PACKET_NONE        0
#define CXD224X_DATA_PACKET_HCI         1
#define CXD224X_DATA_PACKET_LLCP        2
#define CXD224X_DATA_PACKET_T1          3
#define CXD224X_DATA_PACKET_T2          4
#define CXD224X_DATA_PACKET_T3          5
#define CXD224X_DATA_PACKET_T4          6
#define CXD224X_DATA_PACKET_ISO15693    7

void cxd224x_init();
void cxd224x_parse_set_dh_data_type(int type);
void cxd224x_parse(const char *str, const uint8_t *p, int32_t len, int32_t detail_flag);

typedef struct
{
    int rf_intf_activated_ntf_only;
    int rf_intf_activated_ntf_cnt;
    int parse_dtype;
    int time_info;
    int status_color;
    int data_write_flag;
    int is_brcm;
} tCXD224X_PARSE;

/* Global NFC data */
extern tCXD224X_PARSE  g_cxd224x;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // _CXD224X_PARSE_H_

