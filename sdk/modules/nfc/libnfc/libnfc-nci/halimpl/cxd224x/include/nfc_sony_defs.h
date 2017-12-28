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

/******************************************************************************
 *
 *  This file contains the Sony-specific defintions that are shared
 *  between HAL, nfc stack, adaptation layer and applications.
 *
 ******************************************************************************/

#ifndef NFC_SONY_DEFS_H
#define NFC_SONY_DEFS_H

/*****************************************************************************
** Sony-specific NCI definitions
*****************************************************************************/
/**********************************************
 * NCI Proprietary Parameter IDs
 **********************************************/
#define NCI_PARAM_ID_PV_DATA_CODING	0xA0
#define NCI_PARAM_ID_PV_AFI	        0xA1
#define NCI_PARAM_ID_PV_SLOT	        0xA2
#define NCI_PARAM_ID_PV_MASK_LEN	0xA3
#define NCI_PARAM_ID_PV_MASK_VAL	0xA4
#define NCI_PARAM_ID_PV_SUB_CARRIER	0xA5
#define NCI_PARAM_ID_PV_REPLY_SPEED	0xA6

#define NCI_PARAM_ID_PDB_AFI	        0xB8
#define NCI_PARAM_ID_PDB_ATTRIB_PARAM1	0xB9

#define NCI_PARAM_ID_PDB_H_INFO	        0xC0
#define NCI_PARAM_ID_PIB_BIT_RATE	0xC1

#define NCI_PARAM_ID_T_LPS1	        0xD0
#define NCI_PARAM_ID_LISTEN_EXTENSION_TIME	        0xD1
#define NCI_PARAM_ID_LPP_N	                        0xD2
#define NCI_PARAM_ID_FSAM_SHUTDOWN_START_TIME	        0xD3
#define NCI_PARAM_ID_TYPEF_IGNORE_CNT_VALUE	        0xD4
#define NCI_PARAM_ID_TYPEF_IGNORE_CNT_CLEAR_LOOP_NUM	0xD5
#define NCI_PARAM_ID_PN_ACTV_ATR_REQ_TIMEOUT            0xD6
#define NCI_PARAM_ID_LN_ACTV_TIMER_VAL_NON_COMM         0xD7

#define NCI_PARAM_ID_PK_TIME	0xE0


/**********************************************
 * NCI Message Proprietary  Group       - F
 **********************************************/
#define NCI_MSG_NCI_PROP_SET_NFC_LOCK        0x10
#define NCI_MSG_NCI_PROP_RF_AUTO_POLLING     0x11
#define NCI_MSG_NCI_PROP_SIM_SELECT          0x12

#define NCI_MSG_OPTION_PATCH_VERSION         0x1b
#define NCI_MSG_OPTION_PATCH_START           0x1c
#define NCI_MSG_OPTION_PATCH_HEADER          0x1d
#define NCI_MSG_OPTION_PATCH_DATA            0x1e
#define NCI_MSG_OPTION_PATCH_END             0x1f
#define NCI_MSG_OPTION_GET_VERSION           0x20
#define NCI_MSG_OPTION_SET_TGT_OPTIONS       0x21
#define NCI_MSG_OPTION_DETECT_ABNORMAL       0x22
#define NCI_MSG_OPTION_GET_STATUS            0x23
#define NCI_MSG_OPTION_GET_INTERNAL_VERSION  0x24
#define NCI_MSG_OPTION_SET_AUTOTUNE          0x25
#define NCI_MSG_OPTION_READ_ADR              0x33
#define NCI_MSG_OPTION_WRITE_ADR             0x34
#define NCI_MSG_OPTION_FLASH_ACCESS          0x39

/* The events reported on tNFC_VS_CBACK */
/* The event is (NCI_NTF_BIT|oid) or (NCI_RSP_BIT|oid) */
#define NFC_VS_OPTION_PATCH_VERSION       (NCI_RSP_BIT|NCI_MSG_OPTION_PATCH_VERSION)
#define NFC_VS_OPTION_PATCH_START         (NCI_RSP_BIT|NCI_MSG_OPTION_PATCH_START)
#define NFC_VS_OPTION_PATCH_HEADER        (NCI_RSP_BIT|NCI_MSG_OPTION_PATCH_HEADER)
#define NFC_VS_OPTION_PATCH_DATA          (NCI_RSP_BIT|NCI_MSG_OPTION_PATCH_DATA)
#define NFC_VS_OPTION_PATCH_END           (NCI_RSP_BIT|NCI_MSG_OPTION_PATCH_END)

/* NVM Type (in GET_PATCH_VERSION RSP) */
#define NCI_SPD_NVM_TYPE_FLASH          0x00
#define NCI_SPD_NVM_TYPE_SRAM           0x01
#define NCI_SPD_NVM_TYPE_UICC           0x02
#define NCI_SPD_NVM_TYPE_NONE2          0xFF

#define NCI_STATUS_SPD_ERROR2_INVALID_PARAM      0x02
#define NCI_STATUS_SPD_ERROR2_MSG_LEN            0x03
#define NCI_STATUS_SPD_ERROR2_STATE              0x04
#define NCI_STATUS_SPD_ERROR2_NVM_CORRUPTED      0x05
#define NCI_STATUS_SPD_ERROR2_NVM_CORRUPTED_ALL  0x06
#define NCI_STATUS_SPD_ERROR2_INVALID_SIG        0x07

#define NFC_HAL_PRM_MAX_DOWNLOADFILE_SZ (64*1024)
#define NFC_HAL_DBGCMD_MAX_DOWNLOADFILE_SZ (64*1024)

#endif  /* NFC_SONY_DEFS_H */
