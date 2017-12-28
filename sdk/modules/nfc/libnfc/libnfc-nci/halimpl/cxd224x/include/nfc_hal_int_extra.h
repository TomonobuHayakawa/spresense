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

/******************************************************************************
 *
 *  this file contains the NCI transport internal definitions and functions.
 *
 ******************************************************************************/

#ifndef NFC_HAL_INT_EXTRA_H
#define NFC_HAL_INT_EXTRA_H

#include "nfc_hal_target.h"
#include "gki.h"
#include "nci_defs.h"
#include "nfc_hal_api.h"
#include "nfc_hal_int_api.h"
#include "nfc_hal_int.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (NFC_HAL_DEBUG == TRUE)
extern const char * const nfc_hal_init_state_str[];
#define NFC_HAL_SET_INIT_STATE2(state)  HAL_TRACE_DEBUG3 ("init state: %d->%d(%s)", \
        nfc_hal_cb2.dev_cb.initializing_state, state, nfc_hal_init_state_str[state]); nfc_hal_cb2.dev_cb.initializing_state = state;
#else
#define NFC_HAL_SET_INIT_STATE2(state)  nfc_hal_cb2.dev_cb.initializing_state = state;
#endif

/* NFC HAL event for low power mode */
enum
{
    NFC_HAL_LP_TX_DATA_EVT2,                 /* DH is sending data to NFCC   */
    NFC_HAL_LP_RX_DATA_EVT2,                 /* DH received data from NFCC   */
    NFC_HAL_LP_TIMEOUT_EVT2,                 /* Timeout                      */
    NFC_HAL_LP_DEFAULT_EVT2,                 /* add: Back to Default setting      */
    NFC_HAL_LP_KEEP_WAKE_EVT2,               /* add: goto KEEP WAKE mode          */
    NFC_HAL_LP_TIMER_FORCE_EXPIRE_EVT2,      /* add: force expire deassert timer and goto default */
    NFC_HAL_LP_LONG_TIMER_EVT2,              /* add: start lp timer with long timeout */
    NFC_HAL_LP_LAST_EVT2
};
typedef UINT8 tNFC_HAL_LP_EVT2;

/* PRM states */
enum
{
    NFC_HAL_PRM_ST2_IDLE,
    NFC_HAL_PRM_ST2_SPD_COMPARE_VERSION,
    NFC_HAL_PRM_ST2_SPD_GET_PATCH_HEADER,
    NFC_HAL_PRM_ST2_SPD_W4_START_RSP,
    NFC_HAL_PRM_ST2_SPD_W4_HEADER_RSP,
    NFC_HAL_PRM_ST2_SPD_DOWNLOADING,
    NFC_HAL_PRM_ST2_SPD_AUTHENTICATING,
    NFC_HAL_PRM_ST2_SPD_AUTH_DONE,
    NFC_HAL_PRM_ST2_W4_GET_VERSION
};
typedef UINT8 tNFC_HAL_PRM_STATE2;

/* NFC HAL - NFCC initializing state ( replace: NFC_HAL_INIT_STATE=>NFC_HAL_INIT_STATE2 ) */
enum
{
    NFC_HAL_INIT_STATE2_IDLE,                  /*  0: Initialization is done                */
    NFC_HAL_INIT_STATE2_W4_RESET,              /*  1: Waiting for reset rsp                 */
    NFC_HAL_INIT_STATE2_W4_HW_INFO,            /*  2: Waiting for hardware info rsp         */
    NFC_HAL_INIT_STATE2_W4_NFCC_ENABLE,        /*  3: Waiting for reset ntf atter REG_PU up */
    NFC_HAL_INIT_STATE2_W4_BUILD_INFO,         /*  4: Waiting for build info rsp            */
    NFC_HAL_INIT_STATE2_W4_PATCH_INFO,         /*  5: Waiting for patch info rsp            */
    NFC_HAL_INIT_STATE2_W4_APP_COMPLETE,       /*  6: Waiting for complete from application */
    NFC_HAL_INIT_STATE2_W4_POST_INIT_DONE,     /*  7: Waiting for complete of post init     */
    NFC_HAL_INIT_STATE2_W4_CONTROL_DONE,       /*  8: Waiting for control release           */
    NFC_HAL_INIT_STATE2_W4_PREDISCOVER_DONE,   /*  9: Waiting for complete of prediscover   */
    NFC_HAL_INIT_STATE2_W4_PRE_CLOSING_DONE,   /* 10: Waiting for Pre Closing               */
    NFC_HAL_INIT_STATE2_CLOSING,               /* 11: Shutting down                         */
    NFC_HAL_INIT_STATE2_W4_NVERASE,            /* 12: Do NvErase if not nfaStrage.bin1 exists */
};
typedef UINT8 tNFC_HAL_INIT_STATE2;

/* NFC HAL - NFCC config items during post initialization (replace: NFC_HAL_DM_CONFIG=>NFC_HAL_DM_CONFIG2)*/
enum
{
    NFC_HAL_DM_CONFIG2_LPTD,
    NFC_HAL_DM_CONFIG2_START_UP,
    NFC_HAL_DM_CONFIG2_LMR,                  /* Listen Mode Routing for CXD224X */
    NFC_HAL_DM_CONFIG2_START_UP_RMWPRE,      /* Read modified Write Device memory pre setup */
    NFC_HAL_DM_CONFIG2_START_UP_RMW,         /* Read modified Write Device memory issue */
    NFC_HAL_DM_CONFIG2_START_UP_RMWPOST,     /* Read modified Write Device memory finalize */
    NFC_HAL_DM_CONFIG2_START_UP_DEBUGPRE,    /* (debugmode only) execute cmd pre setup */
    NFC_HAL_DM_CONFIG2_START_UP_DEBUG,       /* (debugmode only) execute cmd issue */
    NFC_HAL_DM_CONFIG2_START_UP_DEBUGPOST,   /* (debugmode only) execute cmd finalize */
    NFC_HAL_DM_CONFIG2_START_UP_VSC,
    NFC_HAL_DM_CONFIG2_CLOSING_VSC,
    NFC_HAL_DM_CONFIG2_NONE
};
typedef UINT8 tNFC_HAL_DM_CONFIG2;

/* Control block for device initialization */
typedef struct
{
    tNFC_HAL_INIT_STATE2    initializing_state;     /* state of initializing NFCC               */

    UINT32                  dev_hw_id;              /* NFCC HW ID                          */
    UINT32                  dev_fw_build;           /* FW build number                     */
    tNFC_HAL_DM_CONFIG      next_dm_config;         /* next config in post initialization       */
    UINT8                   next_startup_vsc;       /* next start-up VSC offset in post init    */
    UINT8                   next_closing_vsc;       /* next closing VSC offset in HAL_close    */
    UINT16                  next_startup_debug;     /* next start-up cmdlist offset in post init*/

    tNFC_HAL_POWER_MODE     power_mode;             /* NFCC power mode                          */
    UINT8                   snooze_mode;            /* current snooze mode                      */
    UINT8                   new_snooze_mode;        /* next snooze mode after receiving cmpl    */
    UINT8                   nfc_wake_active_mode;   /* NFC_HAL_LP_ACTIVE_LOW/HIGH               */
    TIMER_LIST_ENT          lp_timer;               /* timer for low power mode                 */
    UINT32                  lp_idle_timeout;        /* transiton timeout(in ms) to low power mode (short or long timeout value are set) */
    UINT32                  lp_idle_short_timeout;  /* transiton short timeout(in ms) to low power mode */
    UINT32                  lp_idle_long_timeout;   /* transiton long  timeout(in ms) to low power mode */
    BOOLEAN                 lp_keep_wake;           /* lp keep assert wake */

    tHAL_NFC_STATUS_CBACK   *p_prop_cback;          /* callback to notify complete of proprietary update */
} tNFC_HAL_DEV_CB2;

typedef struct
{
    tNFC_HAL_PRM_STATE2 state;                  /* download state */
    UINT32              flags;                  /* internal flags */
    UINT32              cur_patch_len_remaining;/* bytes remaining in patchfile to process     */
    const UINT8*        p_cur_patch_data;       /* pointer to patch currently being downloaded */
    UINT32              cur_patch_offset;       /* offset of next byte to process              */
    UINT32              dest_ram;
    TIMER_LIST_ENT      timer;                  /* Timer for patch download                    */

    /* Secure Patch Download */
    UINT32              spd_patch_needed_mask;  /* Mask of patches that need to be downloaded */
    UINT8               spd_patch_count;        /* Number of patches left to download */

    tNFC_HAL_PRM_FORMAT format;                 /* format of patch ram              */
    tNFC_HAL_PRM_CBACK  *p_cback;               /* Callback for download status notifications */
    UINT32              patchram_delay;         /* the dealy after patch */
} tNFC_HAL_PRM_CB2;

/* Information about current patch in NVM */
typedef struct
{
    UINT32              project_id;             /* Current project_id of patch in nvm       */
    UINT16              ver_major;              /* Current major version of patch in nvm    */
    UINT16              ver_minor;              /* Current minor version of patch in nvm    */
    UINT32              ver_hw;                 /* Hardware vesion of target device         */
    UINT32              ver_fw;                 /* Firmware vesion of target device         */
    UINT8               flags;                  /* See NFC_HAL_NVM_FLAGS_* flag definitions */
    UINT8               nvm_type;               /* Current NVM Type - UICC/EEPROM           */
} tNFC_HAL_NVM2;

enum
{
    NFC_HAL_DM_RMW_STATE_READ,
    NFC_HAL_DM_RMW_STATE_WRITE,
};
typedef UINT8 tNFC_HAL_DM_RMW_STATE;

typedef struct
{
    UINT32                 data;                   /* return data */
    UINT8                  status;                 /* return status */
    UINT8                  cur;                    /* current pointer of cfg */
    tNFC_HAL_DM_RMW_STATE  state;                  /* RMW state */
} tNFC_HAL_RMW;

/*-- HAL Delayed NTF packets begin --*/
/* Save specific NTF(EE_DISC_REQ_NTF) paketes during initilization */
/* and drain it by upperlayer request(nfa_hci_main) via Halcc */
enum
{
    NFC_HAL_DM_DLYNTF_STATE_QUEUING=0,
    NFC_HAL_DM_DLYNTF_STATE_STOPED,
    NFC_HAL_DM_DLYNTF_STATE_CLEARED,
};
typedef UINT8 tNFC_HAL_DM_DLYNTF_STATE;

typedef struct
{
    UINT8                    pkt[NFC_HAL_DLYNTF_PKT_LEN];  /* EE_REQ_NTF_Packtes tlv*/
    UINT8                    cur;                  /* current pointer == packtes length */
    tNFC_HAL_DM_DLYNTF_STATE state;                /* DLYNTF state */
} tNFC_HAL_DLYNTF;
/*-- HAL Delayed NTF packets end --*/

enum
{
    NFC_HAL_NVERASE_PRESERVE=0,
    NFC_HAL_NVERASE_ERASE,
};
typedef UINT8 tNFC_HAL_NVERASE;

typedef struct
{
    tHAL_NFC_CBACK          *p_stack_cback;     /* Callback for HAL event notification  */
    tHAL_NFC_DATA_CBACK     *p_data_cback;      /* Callback for data event notification  */

    TIMER_LIST_Q            quick_timer_queue;  /* timer list queue                 */
    TIMER_LIST_ENT          timer;              /* timer for NCI transport task     */

    tNFC_HAL_NCIT_CB        ncit_cb;            /* NCI transport */
    tNFC_HAL_DEV_CB2        dev_cb;             /* device initialization */
    tNFC_HAL_NVM2           nvm_cb;             /* Information about current patch in NVM */

    /* Patchram control block */
    tNFC_HAL_PRM_CB2         prm;

    UINT8                   pre_discover_done;  /* TRUE, when the prediscover config is complete */
    UINT8                   pre_set_mem_idx;

    UINT8                   max_rf_credits;     /* NFC Max RF data credits */
    UINT8                   max_ee;             /* NFC Max number of NFCEE supported by NFCC */
    UINT8                   trace_level;        /* NFC HAL trace level */

    NFC_HDR                 *prediscover_msg;   /* PreDiscover NCI message */
    UINT32                  hal_init_ctrl;      /* for compatible brcm, should be remove */
    tNFC_HAL_RMW            rmw;
#if (defined (NFC_HAL_DELAYED_NTF_INCLUDED) && NFC_HAL_DELAYED_NTF_INCLUDED==TRUE)
    tNFC_HAL_DLYNTF         dly_ntf;
#endif /*(defined (NFC_HAL_DELAYED_NTF_INCLUDED) && NFC_HAL_DELAYED_NTF_INCLUDED==TRUE) */
    tNFC_HAL_NVERASE        nverase;            /* erase device parameter NvStore */
} tNFC_HAL_CB2;

/* Global NCI data */
#if NFC_DYNAMIC_MEMORY == FALSE
extern tNFC_HAL_CB2   nfc_hal_cb2;
#else
#define nfc_hal_cb2    (*nfc_hal_cb2_ptr)
extern tNFC_HAL_CB2   *nfc_hal_cb2_ptr;
#endif

/* HAL_NfcSetoptionalparm entry */
enum
{
    NFC_OPTPARM_HAL_INIT_CTRL=1,    /* hal_init_ctrl */
    NFC_OPTPARM_HAL_NVERASE,        /* nverase */
};
/* hal_init_ctrl: HAL initilize sequence enable/disable ctrl flag */
#define NFC_DEVICE_RESET         (1<<0)
#define NFC_NFASTORAGE_NVERASE   (1<<1) /* if not found nfaStorage.bin1, do NvErase and DeviceReset */
#define NFC_CLEAR_UART           (1<<4)

/* android_logmsg.cpp */
void DispHciCmd_Hal (BT_HDR *p_buf);
void DispHciEvt_Hal (BT_HDR *p_buf);

#define NFC_HAL_FOURCC( a, b, c, d ) \
    (((UINT32 )(a)<<24)|((UINT32 )(b)<<16)|((UINT32 )(c)<<8)|((UINT32 )(d)<<0))

#define NFC_HAL_PROJECTID NFC_HAL_FOURCC('p','F','u','l')

#define NFC_HAL_PRM_HW_VERSION_MASK(ver) ( (ver) & 0xFFFF0000 )
#define NFC_HAL_PRM_FW_VERSION_MASK(ver) ( (ver) & 0x0000FFFF )

/* NFC HAL Message NFA_SendHalControlCommnad() id and opecode */
#define NFC_HAL_SENDCMD_SNOOZE_ID                    0x80
#define  NFC_HAL_SENDCMD_SNOOZE_OP_DEFAULT            0x0
#define  NFC_HAL_SENDCMD_SNOOZE_OP_KEEP_WAKE          0x1
#define  NFC_HAL_SENDCMD_SNOOZE_OP_LP_TIMER_FORCE_EXPIRE  0x2
#define  NFC_HAL_SENDCMD_SNOOZE_OP_LP_LONG_TIMER      0x3
#define NFC_HAL_PREDISCOVER_CMD_REG                  0x81
#define NFC_HAL_GET_DEVICE_INFO                      0x82
#define NFC_HAL_REQ_DELAYED_PKTS                     0x83 /* #ifdef NFC_HAL_DELAYED_NTF_INCLUDED */
#define  NFC_HAL_REQ_DELAYED_PKTS_DRAIN               0x0 /* drain and clear */
#define  NFC_HAL_REQ_DELAYED_PKTS_STOP                0x1 /* stop queing incomming packet */
#define  NFC_HAL_REQ_DELAYED_PKTS_CLEAR               0x2 /* just clear */
#define NFC_HAL_DEVICE_RESET                         0x84

/* From nfc_hal_main.c */
void nfc_hal_main_send_devinfo(void);
void nfc_hal_main_delayed_pkt_init(void);
void nfc_hal_main_delayed_pkt_queuing (NFC_HDR *p_msg);
void nfc_hal_main_destroy (void);

/* nfc_hal_dm.c */
void nfc_hal_dm_config_nfcc_closing (void);

#ifdef __cplusplus
}
#endif

#endif /* NFC_HAL_INT_EXTRA_H */
