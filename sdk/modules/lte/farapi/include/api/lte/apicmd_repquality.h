/****************************************************************************
 * modules/lte/farapi/include/api/lte/apicmd_repquality.h
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

#ifndef __MODULES_LTE_FARAPI_INCLUDE_API_LTE_APICMD_REPQUALITY_H
#define __MODULES_LTE_FARAPI_INCLUDE_API_LTE_APICMD_REPQUALITY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "apicmd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_SET_REPQUALITY_DISABLE      (0)
#define APICMD_SET_REPQUALITY_ENABLE       (1)
#define APICMD_SET_REPQUALITY_INTERVAL_MIN (1)
#define APICMD_SET_REPQUALITY_INTERVAL_MAX (4233600)

#define APICMD_SET_REPQUALITY_RES_OK      (0)
#define APICMD_SET_REPQUALITY_RES_ERR     (1)

#define APICMD_REP_QUALITY_DISABLE  (0)
#define APICMD_REP_QUALITY_ENABLE   (1)
#define APICMD_REP_QUALITY_RSRP_MIN (-140)
#define APICMD_REP_QUALITY_RSRP_MAX (0)
#define APICMD_REP_QUALITY_RSRQ_MIN (-60)
#define APICMD_REP_QUALITY_RSRQ_MAX (0)
#define APICMD_REP_QUALITY_SINR_MIN (-128)
#define APICMD_REP_QUALITY_SINR_MAX (40)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure discribes the data structure of the API command */

PACK_STRUCT_BEGIN
struct apicmd_cmddat_setrepquality_s
{
  PACK_STRUCT_FIELD(uint8_t enability);
  PACK_STRUCT_FIELD(uint32_t interval);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

PACK_STRUCT_BEGIN
struct apicmd_cmddat_setrepquality_res_s
{
  PACK_STRUCT_FIELD(uint8_t result);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

PACK_STRUCT_BEGIN
struct apicmd_cmddat_repquality_s
{
  PACK_STRUCT_FIELD(uint8_t enability);
  PACK_STRUCT_FIELD(int16_t rsrp);
  PACK_STRUCT_FIELD(int16_t rsrq);
  PACK_STRUCT_FIELD(int16_t sinr);
  PACK_STRUCT_FIELD(int16_t rssi);
} PACK_STRUCT_STRUCT;
PACK_STRUCT_END

#endif /* __MODULES_LTE_FARAPI_INCLUDE_API_LTE_APICMD_REPQUALITY_H */
