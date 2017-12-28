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
#ifndef _parse_INTERNAL_H_
#define _parse_INTERNAL_H_

#include <stdint.h>
#include "cxd224x_debug.h"

// 0:ok, -1:fail
typedef int32_t (*CXD224X_NCI_PARSE_FUNC)(uint32_t indent, const uint8_t *p, int32_t len);

typedef struct {
    const char *m_pName;
    uint8_t m_mt;
    uint8_t m_gid;
    uint8_t m_oid_start;
    uint8_t m_oid_end;
    CXD224X_NCI_PARSE_FUNC m_func;
} CXD224X_NCI_CMD_REC;

typedef struct  {
    const char *m_pName;
    uint8_t m_start;
    uint8_t m_end;
    CXD224X_NCI_PARSE_FUNC m_func;
} CXD224X_NCI_ID_NAME_REC;

#define CXD224X_CNT(X)    (sizeof(X)/sizeof(X[0]))
#define CXD224X_CHECK_NULL(PTR,RET_VAL)           if(PTR==NULL){return RET_VAL;}
#define CXD224X_DEF_START                         static const CXD224X_NCI_ID_NAME_REC data[] = {

#define CXD224X_DEF1(START,NAME,FUNC)                 {#NAME, START, START, FUNC},
#define CXD224X_DEF2(START,END,NAME,FUNC)             {#NAME, START, END,   FUNC},
#define CXD224X_DEF1N(START,NAME)                     {#NAME, START, START, NULL},
#define CXD224X_DEF2N(START,END,NAME)                 {#NAME, START, END,   NULL},

#define CXD224X_XDEF1(NAME,START,FUNC)                {#NAME, START, START, FUNC},
#define CXD224X_XDEF2(NAME,START,END,FUNC)            {#NAME, START, END,   FUNC},
#define CXD224X_XDEF1N(NAME,START)                    {#NAME, START, START, NULL},
#define CXD224X_XDEF2N(NAME,START,END)                {#NAME, START, END,   NULL},

#define CXD224X_DEF_END                             }; return get_name(id, data, CXD224X_CNT(data));

#define CXD224X_SKIP(SRC,LEN,WANT)    \
    if(LEN<WANT){return -1;}\
    else{SRC+=WANT;LEN-=WANT;}

#define CXD224X_GET_HEX(SRC,LEN,WANT,DST)    \
    if(LEN<WANT){return -1;}\
    else{memcpy(DST,SRC,WANT);SRC+=WANT;LEN-=WANT;}

#define CXD224X_DUMP_HEX(INDENT,SRC,LEN,WANT,TITLE)    \
    if(LEN<WANT){return -1;}\
    else{cxd224x_dump2(INDENT,TITLE,SRC,WANT);SRC+=WANT;LEN-=WANT;}

#define CXD224X_PRINT_1BF(INDENT,SRC,LEN,TITLE,FUNC)    \
    if(LEN<1){\
        return -1;\
    }else{        \
        uint8_t id=SRC[0];\
        cxd224x_debug(INDENT, "%s=0x%02X(%s)", TITLE, id, FUNC(id));\
        SRC+=1;LEN-=1;\
    }

#define CXD224X_PRINT_1BF_GET_DATA(INDENT,SRC,LEN,TITLE,FUNC,DATA)    \
    if(LEN<1){\
        return -1;\
    }else{        \
        uint8_t id=SRC[0];\
        cxd224x_debug(INDENT, "%s=0x%02X(%s)", TITLE, id, FUNC(id));\
        SRC+=1;LEN-=1;\
        DATA=id;\
    }

#define CXD224X_PRINT_1BN(INDENT,SRC,LEN,TITLE)    \
    if(LEN<1){\
        return -1;\
    }else{        \
        uint8_t id=SRC[0];\
        cxd224x_debug(INDENT, "%s=0x%02X", TITLE, id);\
        SRC+=1;LEN-=1;\
    }

#define CXD224X_PRINT_2BN(INDENT,SRC,LEN,TITLE)    CXD224X_DUMP_HEX(INDENT,SRC,LEN,2,TITLE)
#define CXD224X_PRINT_4BN(INDENT,SRC,LEN,TITLE)    CXD224X_DUMP_HEX(INDENT,SRC,LEN,4,TITLE)

static int32_t parse_data(uint32_t indent, uint8_t conn_id, const uint8_t *p, int32_t len);
static int32_t parse_llcp(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_rsp_status_code(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_none(uint32_t indent, const uint8_t *p, int32_t len);

static const char* get_reset_type(uint8_t id);
static const char* get_reset_reason(uint8_t id);
static int32_t parse_core_reset_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_core_reset_rsp(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_core_reset_ntf(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_core_init_rsp(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_core_set_config_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_core_set_config_rsp(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_core_get_config_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_core_get_config_rsp(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_func_nci_r100_tbl15_rf(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_func_nci_r100_tbl15_nfcee(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_core_conn_create_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_core_conn_create_rsp(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_core_conn_close_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_core_conn_credits_ntf(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_core_interface_error_ntf(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_rf_discover_map_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_func_nci_r100_tbl47(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_func_nci_r100_tbl48(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_func_nci_r100_tbl49(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_rf_set_listen_mode_routing_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_rf_get_listen_mode_routing_ntf(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_rf_discover_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static const char* get_notification_type(uint8_t id);
static int32_t parse_nfc_r100_tbl54(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_nfc_r100_tbl56(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_nfc_r100_tbl58(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_nfc_r100_tbl59(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_rf_discover_ntf(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_rf_discover_select_cmd(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_nfc_r100_tbl76(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_nfc_r100_tbl77(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_nfc_r100_tbl78(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_nfc_r100_tbl79(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_nfc_r100_tbl82(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_nfc_r100_tbl83(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_rf_intf_activated_ntf(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_rf_deactivate_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_rf_deactivate_ntf(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_rf_field_info_ntf(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_rf_t3t_polling_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_rf_t3t_polling_ntf(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_rf_nfcee_action_ntf(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_func_nci_r100_tbl67(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_rf_nfcee_discovery_req_ntf(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_func_nci_r100_tbl72_00(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_func_nci_r100_tbl72_01_02(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_func_nci_r100_tbl72_03(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_rf_parameter_update_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_rf_parameter_update_rsp(uint32_t indent, const uint8_t *p, int32_t len);

static int32_t parse_func_nci_r100_tbl87(uint32_t indent, const uint8_t *p, int32_t len);
static const char* get_nfcee_discovery_action(uint8_t id);
static int32_t parse_nfcee_discover_cmd(uint32_t indent, const uint8_t *p, int32_t len);
static int32_t parse_nfcee_discover_rsp(uint32_t indent, const uint8_t *p, int32_t len);
static const char* get_nfcee_status(uint8_t id);
static int32_t parse_nfcee_discover_ntf(uint32_t indent, const uint8_t *p, int32_t len);
static const char* get_nfcee_mode(uint8_t id);
static int32_t parse_nfcee_mode_set_cmd(uint32_t indent, const uint8_t *p, int32_t len);


static int32_t print_tlv_by_len(uint32_t indent, const uint8_t *p, uint32_t len, 
    const CXD224X_NCI_ID_NAME_REC *p_rec, uint32_t rec_cnt);

static int32_t print_tlv_by_cnt(uint32_t indent,
    const uint8_t *p, uint32_t len,
    uint32_t tag_cnt, 
    const CXD224X_NCI_ID_NAME_REC *p_rec, uint32_t rec_cnt);

static void print_tlv_tag(uint32_t indent, 
    const uint8_t *p, uint32_t len, 
    const CXD224X_NCI_ID_NAME_REC *p_rec, uint32_t rec_cnt);

static const char* get_name(uint8_t id, const CXD224X_NCI_ID_NAME_REC *p, uint32_t cnt);
static int32_t get_index(uint8_t id, const CXD224X_NCI_ID_NAME_REC *p, uint32_t cnt);
static const char* get_nci_r100_tbl7(uint8_t id);
static void print_nci_r100_tbl9_obt0(uint32_t indent, uint8_t id);
static void print_nci_r100_tbl9_obt1(uint32_t indent, uint8_t id);
static void print_nci_r100_tbl9_obt2(uint32_t indent, uint8_t id);
static void print_nci_r100_tbl9_obt3(uint32_t indent, uint8_t id);
static const char* get_nci_r100_tbl12(uint8_t id);
static const char* get_nci_r100_tbl21(uint8_t id);
static const char* get_nci_r100_tbl45(uint8_t id);
static const char* get_nci_r100_tbl50(uint8_t id);
static const char* get_nci_r100_tbl63(uint8_t id);
static const char* get_nci_r100_tbl64(uint8_t id);
static const char* get_nci_r100_tbl69(uint8_t id);
static const char* get_nci_r100_tbl94(uint8_t id);
static const char* get_nci_r100_tbl95(uint8_t id);
static const char* get_nci_r100_tbl96(uint8_t id);
static const char* get_nci_r100_tbl97(uint8_t id);
static const char* get_nci_r100_tbl98(uint8_t id);
static const char* get_nci_r100_tbl99(uint8_t id);
static const char* get_nci_r100_tbl100(uint8_t id);

#endif // _parse_INTERNAL_H_
