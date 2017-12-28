/******************************************************************************
 *
 *  Copyright (C) 2011-2013 Broadcom Corporation
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

#include <stdio.h>
#include "nfc_hal_int.h"
#include "nci_defs.h"
#include "nfc_sony_defs.h"

UINT8 *p_nfc_hal_dm_start_up_cfg = NULL;

/* RF_SET_LISTEN_MODE_ROUTING_CMD at start up */
const UINT8 nfc_hal_lmr_cfg[] = {
    /* TLV qlen */
    25,
    /* Routing Entry[1] */
    0x00,   /* TECH_ROUTING                             */
    0x03,   /* length=3                                 */
    0x00,   /* NFCEE_ID=DH                              */
    0x01,   /* Battery_off=0,Switch_off=0,Switch_on=1   */
    0x00,   /* TypeA                                    */
    /* Routing Entry[2] */
    0x00,   /* TECH_ROUTING                             */
    0x03,   /* length=3                                 */
    0x00,   /* NFCEE_ID=DH                              */
    0x01,   /* Battery_off=0,Switch_off=0,Switch_on=1   */
    0x01,   /* TypeB                                    */
    /* Routing Entry[3] */
    0x00,   /* TECH_ROUTING                             */
    0x03,   /* length=3                                 */
    0x00,   /* NFCEE_ID=DH                              */
    0x01,   /* Battery_off=0,Switch_off=0,Switch_on=1   */
    0x02,   /* TypeF                                    */
    /* Routing Entry[4] */
    0x01,   /* PROTOCOL_ROUTING                         */
    0x03,   /* length=3                                 */
    0x00,   /* NFCEE_ID=DH                              */
    0x01,   /* Battery_off=0,Switch_off=0,Switch_on=1   */
    0x04,   /* ISO-DEP                                  */
    /* Routing Entry[5] */
    0x01,   /* PROTOCOL_ROUTING                         */
    0x03,   /* length=3                                 */
    0x00,   /* NFCEE_ID=DH                              */
    0x01,   /* Battery_off=0,Switch_off=0,Switch_on=1   */
    0x05,   /* NFC-DEP                                  */
};
UINT8 *p_nfc_hal_dm_lmr_cfg = (UINT8 *)&nfc_hal_lmr_cfg[0];

/* the VSCs at start up:
 * The VSCs are specified in TLV format similar to nfa_start_up_cfg[]
 * first byte is the TLV total len.
 * B0 is the first T; i.e. the opcode for the VSC
 * B1 is the len of the VSC parameters/payload
 * */
UINT8 nfc_hal_dm_start_up_vsc_cfg[] = {
    /* TLV len */   5,
    /* B0 */        NCI_MTS_CMD|NCI_GID_PROP,
    /* B1 */        NCI_MSG_NCI_PROP_SIM_SELECT,
    /* B2 */        2,
    /* B3 */        0, /* 0:SWIO0, 1:SWIO1(SWP SAM) */
    /* B4 */        1  /* 0: Low, 1:Full */
};

UINT8 *p_nfc_hal_dm_start_up_vsc_cfg = (UINT8 *)&nfc_hal_dm_start_up_vsc_cfg[0];

/* the Read modified Writes at start up:
 * The RMWs are specified in TLV format similar to nfa_start_up_cfg[]
 * first byte is the TLV total len.
 * B0 to B3:  address
 * B4 to B7:  mask
 * B8 to B11: write data
 * */
UINT8 nfc_hal_dm_start_up_rmw_cfg[] = {
    /* TLV len */   0,
};
UINT8 *p_nfc_hal_dm_start_up_rmw_cfg = (UINT8 *)&nfc_hal_dm_start_up_rmw_cfg[0];

/* the VSCs at closing:
 * The VSCs are specified in TLV format similar to nfa_start_up_cfg[]
 * first byte is the TLV total len.
 * B0 is the first T; i.e. the opcode for the VSC
 * B1 is the len of the VSC parameters/payload
 * */
UINT8 nfc_hal_dm_closing_vsc_cfg[] = {
    /* TLV len */   0,
};

UINT8 *p_nfc_hal_dm_closing_vsc_cfg = (UINT8 *)&nfc_hal_dm_closing_vsc_cfg[0];

/* the SetConfig at HAL_NfcPreDiscover. This is done once after HAL_NfcOpen */
UINT8 nfc_hal_pre_discover_cfg[] = {
    /* TLV len */   0x0,
};

UINT8 *p_nfc_hal_pre_discover_cfg = (UINT8 *)&nfc_hal_pre_discover_cfg[0];

/* LPTD parameters (LowPowerTagDetection)
 * No typical values for CXD224x
 * The timing and threshold parameters used for a customer handset/hardware may vary
 * depending on antenna and should be verified during a customer testing phase.
 * the data fields without comments are too complicated. Please see ""
 * */
const UINT8 nfc_hal_dm_lptd_cfg[] =
{
    0,             /* total TLV length excluding itself */
};

UINT8 *p_nfc_hal_dm_lptd_cfg = (UINT8 *) &nfc_hal_dm_lptd_cfg[0];

tNFC_HAL_CFG nfc_hal_cfg =
{
    FALSE,                                  /* set nfc_hal_prm_nvm_required to TRUE, if the platform wants to abort PRM process without NVM */
    (UINT16) NFC_HAL_NFCC_ENABLE_TIMEOUT,   /* max time to wait for RESET NTF after setting REG_PU to high
                                            ** If NFCC doesn't have NVM or cannot load patch from NVM without Xtal setting
                                            ** then set it to short to optimize bootup time because NFCC cannot send RESET NTF.
                                            ** Otherwise, it depends on NVM type and size of patchram.
                                            */
    (UINT16) NFC_HAL_NFCC_ENABLE_TIMEOUT    /* max time to wait for RESET NTF after setting Xtal frequency
                                            ** It depends on NVM type and size of patchram.
                                            */
#if (defined(NFC_HAL_HCI_INCLUDED) && (NFC_HAL_HCI_INCLUDED == TRUE))
    ,
    TRUE,                                   /* set nfc_hal_first_boot to TRUE, if platform enables NFC for the first time after bootup */
    (HAL_NFC_HCI_UICC0_HOST | HAL_NFC_HCI_UICC1_HOST )  /* Set bit(s) for supported UICC(s) */
#endif
};

tNFC_HAL_CFG *p_nfc_hal_cfg= (tNFC_HAL_CFG *) &nfc_hal_cfg;


/* execute CMD List (for Debug and adjustment paramter)
 *
 * No typical values
 */
UINT8 *p_nfc_hal_dm_start_up_debug_cfg = NULL;
FILE  *p_nfc_hal_dm_start_up_debug_outfd = NULL;
