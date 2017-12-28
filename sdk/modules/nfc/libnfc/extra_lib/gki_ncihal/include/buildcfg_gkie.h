/******************************************************************************
 *
 *  Copyright (C) 1999-2012 Broadcom Corporation
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
#ifndef __BUILDCFG_H
#define __BUILDCFG_H
#include <string.h>
#ifndef SPZ2_IMPL
#include <memory.h>
#endif
#include <stdio.h>
#include "data_types.h"

#ifndef NFC_CONTORLLER_ID
#define NFC_CONTORLLER_ID       (1)
#endif

#define BTE_APPL_MAX_USERIAL_DEV_NAME           (256)

#ifdef BT_TRACE_VERBOSE
#undef BT_TRACE_VERBOSE
#endif
#define BT_TRACE_VERBOSE      TRUE

#define TRACE_TASK_INCLUDED   TRUE

#define GKI_NUM_FIXED_BUF_POOLS 6

/* nci: GKI_PUBLIC_POOL */
#define GKI_BUF0_SIZE           268
//TagRead, CE
//#define GKI_BUF0_MAX            40
#define GKI_BUF0_MAX            10
#define NCI_BUF_POOL_ID         GKI_POOL_ID_0

#define GKI_BUF1_MAX            0

/* nci: */
#define GKI_BUF2_SIZE           660
//TagRead, CE
//#define GKI_BUF2_MAX            50
#define GKI_BUF2_MAX            10

/* nci: */
#define GKI_BUF3_MAX            30

/* POOL_ID_4: HAL GKI_PUBLIC_POOL */
#define GKI_BUF4_SIZE           64
#define GKI_BUF4_MAX            10

/* POOL_ID_5: HAL NFC_HAL/USERIAL */
#define GKI_BUF5_SIZE           288
#define GKI_BUF5_MAX            10
#define NFC_HAL_NCI_POOL_ID     GKI_POOL_ID_5
#define USERIAL_POOL_ID         GKI_POOL_ID_5
#define USERIAL_POOL_BUF_SIZE   GKI_BUF5_SIZE

#define GKI_BUF6_MAX            0
#define GKI_BUF7_MAX            0
#define GKI_BUF8_MAX            0

#define USERIAL_HAL_TASK            5
#define NFC_HAL_TASK                6

#ifdef	__cplusplus
extern "C" {
#endif

extern UINT8 *scru_dump_hex (UINT8 *p, char *p_title, UINT32 len, UINT32 trace_layer, UINT32 trace_type);
extern void ScrLog(UINT32 trace_set_mask, const char *fmt_str, ...);
extern void DispNci (UINT8 *p, UINT16 len, BOOLEAN is_recv);

#ifndef _TIMEB
#define _TIMEB
struct _timeb
{
    long    time;
    short   millitm;
    short   timezone;
    short   dstflag;
};
void    _ftime (struct _timeb*);

#endif

#ifdef	__cplusplus
};
#endif
#endif
