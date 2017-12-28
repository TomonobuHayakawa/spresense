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
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include <stdarg.h>
#include <pthread.h>
#include <stdint.h>

#include <cutils/log.h>

#include "cxd224x_debug.h"
#include "cxd224x_parse.h"
#include "cxd224x_parse_internal.h"
#include "nfc_types_extra.h"
#include "nfc_target.h"
#include "nfa_api.h"
#include "nfa_hci_api.h"
#include "nfa_hci_defs.h"
#include "tags_defs.h"

enum {
    DATA_TYPE_NONE=0x1ff,
    DATA_TYPE_T1T=NFA_PROTOCOL_T1T,
    DATA_TYPE_T2T=NFA_PROTOCOL_T2T,
    DATA_TYPE_T3T=NFA_PROTOCOL_T3T,
    DATA_TYPE_ISODEP=NFA_PROTOCOL_ISO_DEP,
    DATA_TYPE_NFCDEP=NFA_PROTOCOL_NFC_DEP,
    DATA_TYPE_I93=0x83,//NFA_PROTOCOL_ISO15693,
    DATA_TYPE_HCI=0x100,
    DATA_TYPE_UNKNOWN
};
typedef int32_t tDATA_TYPE;

static tDATA_TYPE g_data_type;
static int32_t g_last_cmd=0;
static int32_t g_return_len=0;
static int32_t g_last_dir=-1;

static void set_data_type_protocol( const uint8_t *p, int32_t len )
{
    if( len > 1 )
    {
        if((*p    == NFA_PROTOCOL_T1T)        /* Type1Tag         - NFC-A             */
           || (*p == NFA_PROTOCOL_T2T)        /* MIFARE/Type2Tag  - NFC-A             */
           || (*p == NFA_PROTOCOL_T3T)        /* Felica/Type3Tag  - NFC-F             */
           || (*p == NFA_PROTOCOL_ISO_DEP)    /* Type 4A,4B       - NFC-A or NFC-B    */
           || (*p == NFA_PROTOCOL_NFC_DEP)    /* NFCDEP/LLCP      - NFC-A or NFC-F    */
           || (*p == 0x83/*NFA_PROTOCOL_ISO15693*/))
        {
            g_data_type = *p;
        }
        else
        {
            g_data_type = DATA_TYPE_NONE;
        }
    }
}
static void set_data_type_desttype( const uint8_t *p, int32_t len )
{
    if( len > 1 )
    {
        switch(*p){
        case NCI_DEST_TYPE_NFCC:
        case NCI_DEST_TYPE_REMOTE:
            g_data_type = DATA_TYPE_UNKNOWN;
            break;
        case NCI_DEST_TYPE_NFCEE:
            if( p[1] > 0 )
            {
                int n=2;
                while( n < len ){
                    switch( p[n] ){
                    case 0x0:
                        n+=3;
                        break;
                    case 0x1:
                        if( p[n+2] ==  NCI_NFCEE_INTERFACE_HCI_ACCESS )
                        {
                            g_data_type = DATA_TYPE_HCI;
                            return;
                        }
                        else if( p[n+2] ==  NCI_NFCEE_INTERFACE_T3T )
                        {
                            g_data_type = DATA_TYPE_T3T;
                            return;
                        }
                        else if( p[n+2] ==  NCI_NFCEE_INTERFACE_APDU )
                        {
                            g_data_type = DATA_TYPE_ISODEP;
                            return;
                        } 
                        n+=3;
                        break;
                    default:
                        return;
                    }
                }
            }
            break;
        default:
            ;
        }
    }
    g_data_type = DATA_TYPE_NONE;
}
void clr_data_type()
{
    g_data_type = DATA_TYPE_NONE;
    g_last_cmd = 0;
    g_last_dir = -1;
}

tDATA_TYPE get_data_type()
{
    return g_data_type;
}


//----------------------------------------------------------------
// these parse APIs are base on 
//      "NFCForum-TS-NCI-1.0.pdf"
//      "2012-11-06"
//----------------------------------------------------------------

static const CXD224X_NCI_CMD_REC cxd224x_nci_r100_tbl102[] = {
// NCI Core
    {"CORE_RESET_CMD",                      0x01, 0x00, 0x00, 0x00, &parse_core_reset_cmd},
    {"CORE_RESET_RSP",                      0x02, 0x00, 0x00, 0x00, &parse_core_reset_rsp},
    {"CORE_RESET_NTF",                      0x03, 0x00, 0x00, 0x00, &parse_core_reset_ntf},

    {"CORE_INIT_CMD",                       0x01, 0x00, 0x01, 0x01, &parse_none},
    {"CORE_INIT_RSP",                       0x02, 0x00, 0x01, 0x01, &parse_core_init_rsp},

    {"CORE_SET_CONFIG_CMD",                 0x01, 0x00, 0x02, 0x02, &parse_core_set_config_cmd},
    {"CORE_SET_CONFIG_RSP",                 0x02, 0x00, 0x02, 0x02, &parse_core_set_config_rsp},

    {"CORE_GET_CONFIG_CMD",                 0x01, 0x00, 0x03, 0x03, &parse_core_get_config_cmd},
    {"CORE_GET_CONFIG_RSP",                 0x02, 0x00, 0x03, 0x03, &parse_core_get_config_rsp},

    {"CORE_CONN_CREATE_CMD",                0x01, 0x00, 0x04, 0x04, &parse_core_conn_create_cmd},
    {"CORE_CONN_CREATE_RSP",                0x02, 0x00, 0x04, 0x04,&parse_core_conn_create_rsp},

    {"CORE_CONN_CLOSE_CMD",                 0x01, 0x00, 0x05, 0x05, &parse_core_conn_close_cmd},
    {"CORE_CONN_CLOSE_RSP",                 0x02, 0x00, 0x05, 0x05, &parse_none},

    {"CORE_CONN_CREDITS_NTF",               0x03, 0x00, 0x06, 0x06, &parse_core_conn_credits_ntf},
    {"CORE_GENERIC_ERROR_NTF",              0x03, 0x00, 0x07, 0x07, &parse_rsp_status_code},
    {"CORE_INTERFACE_ERROR_NTF",            0x03, 0x00, 0x08, 0x08, &parse_core_interface_error_ntf},

// RF Management
    {"RF_DISCOVER_MAP_CMD",                 0x01, 0x01, 0x00, 0x00, &parse_rf_discover_map_cmd},
    {"RF_DISCOVER_MAP_RSP",                 0x02, 0x01, 0x00, 0x00, &parse_none},

    {"RF_SET_LISTEN_MODE_ROUTING_CMD",      0x01, 0x01, 0x01, 0x01, &parse_rf_set_listen_mode_routing_cmd},
    {"RF_SET_LISTEN_MODE_ROUTING_RSP",      0x02, 0x01, 0x01, 0x01, &parse_none},

    {"RF_GET_LISTEN_MODE_ROUTING_CMD",      0x01, 0x01, 0x02, 0x02, &parse_none},
    {"RF_GET_LISTEN_MODE_ROUTING_RSP",      0x02, 0x01, 0x02, 0x02, &parse_none},
    {"RF_GET_LISTEN_MODE_ROUTING_NTF",      0x03, 0x01, 0x02, 0x02, &parse_rf_get_listen_mode_routing_ntf},

    {"RF_DISCOVER_CMD",                     0x01, 0x01, 0x03, 0x03, &parse_rf_discover_cmd},
    {"RF_DISCOVER_RSP",                     0x02, 0x01, 0x03, 0x03, &parse_none},
    {"RF_DISCOVER_NTF",                     0x03, 0x01, 0x03, 0x03, &parse_rf_discover_ntf},

    {"RF_DISCOVER_SELECT_CMD",              0x01, 0x01, 0x04, 0x04, &parse_rf_discover_select_cmd},
    {"RF_DISCOVER_SELECT_RSP",              0x02, 0x01, 0x04, 0x04, &parse_none},

    {"RF_INTF_ACTIVATED_NTF",               0x03, 0x01, 0x05, 0x05, &parse_rf_intf_activated_ntf},

    {"RF_DEACTIVATE_CMD",                   0x01, 0x01, 0x06, 0x06, &parse_rf_deactivate_cmd},
    {"RF_DEACTIVATE_RSP",                   0x02, 0x01, 0x06, 0x06, &parse_none},
    {"RF_DEACTIVATE_NTF",                   0x03, 0x01, 0x06, 0x06, &parse_rf_deactivate_ntf},

    {"RF_FIELD_INFO_NTF",                   0x03, 0x01, 0x07, 0x07, &parse_rf_field_info_ntf},

    {"RF_T3T_POLLING_CMD",                  0x01, 0x01, 0x08, 0x08, &parse_rf_t3t_polling_cmd},
    {"RF_T3T_POLLING_RSP",                  0x02, 0x01, 0x08, 0x08, &parse_none},
    {"RF_T3T_POLLING_NTF",                  0x03, 0x01, 0x08, 0x08, &parse_rf_t3t_polling_ntf},

    {"RF_NFCEE_ACTION_NTF",                 0x03, 0x01, 0x09, 0x09, &parse_rf_nfcee_action_ntf},

    {"RF_NFCEE_DISCOVERY_REQ_NTF",          0x03, 0x01, 0x0A, 0x0A, &parse_rf_nfcee_discovery_req_ntf},

    {"RF_PARAMETER_UPDATE_CMD",             0x01, 0x01, 0x0B, 0x0B, &parse_rf_parameter_update_cmd},
    {"RF_PARAMETER_UPDATE_RSP",             0x02, 0x01, 0x0B, 0x0B, &parse_rf_parameter_update_rsp},

// NFCEE Management
    {"NFCEE_DISCOVER_CMD",                  0x01, 0x02, 0x00, 0x00, &parse_nfcee_discover_cmd},
    {"NFCEE_DISCOVER_RSP",                  0x02, 0x02, 0x00, 0x00, &parse_nfcee_discover_rsp},
    {"NFCEE_DISCOVER_NTF",                  0x03, 0x02, 0x00, 0x00, &parse_nfcee_discover_ntf},

    {"NFCEE_MODE_SET_CMD",                  0x01, 0x02, 0x01, 0x01, &parse_nfcee_mode_set_cmd},
    {"NFCEE_MODE_SET_RSP",                  0x02, 0x02, 0x01, 0x01, &parse_none},

// Proprietary
    {"PROPRIETARY_CMD",                     0x01, 0x0F, 0x00, 0xFF, &parse_none},
    {"PROPRIETARY_RSP",                     0x02, 0x0F, 0x00, 0xFF, &parse_none},
    {"PROPRIETARY_NTF",                     0x03, 0x0F, 0x00, 0xFF, &parse_none},

};

static const CXD224X_NCI_ID_NAME_REC cxd224x_nci_r100_tbl101[] = {
    // Common Discovery Parameters
    CXD224X_XDEF1(TOTAL_DURATION,                  0x00, NULL)
    CXD224X_XDEF1(CON_DEVICES_LIMIT,               0x01, NULL)
    CXD224X_XDEF2(RFU,                             0x02,0x07, NULL)
    // Poll Mode NFC-A Discovery Parameters
    CXD224X_XDEF1(PA_BAIL_OUT,                     0x08, NULL)
    CXD224X_XDEF2(RFU,                             0x09,0x0F, NULL)
    // Poll Mode NFC-B Discovery Parameters
    CXD224X_XDEF1(PB_AFI,                          0x10, NULL)
    CXD224X_XDEF1(PB_BAIL_OUT,                     0x11, NULL)
    CXD224X_XDEF1(PB_ATTRIB_PARAM1,                0x12, NULL)
    CXD224X_XDEF1(PB_SENSB_REQ_PARAM,              0x13, NULL)
    CXD224X_XDEF2(RFU,                             0x14,0x17, NULL)
    // Poll Mode NFC-F Discovery Parameters
    CXD224X_XDEF1(PF_BIT_RATE,                     0x18, NULL)
    CXD224X_XDEF1(PF_RC_CODE,                      0x19, NULL)
    CXD224X_XDEF2(RFU,                             0x1A,0x1F, NULL)
    // Poll Mode ISO-DEP Discovery Parameters
    CXD224X_XDEF1(PB_H_INFO,                       0x20, NULL)
    CXD224X_XDEF1(PI_BIT_RATE,                     0x21, NULL)
    CXD224X_XDEF1(PA_ADV_FEAT,                     0x22, NULL)
    CXD224X_XDEF2(RFU,                             0x23,0x27, NULL)
    // Poll Mode NFC-DEP Discovery Parameters
    CXD224X_XDEF1(PN_NFC_DEP_SPEED,                0x28, NULL)
    CXD224X_XDEF1(PN_ATR_REQ_GEN_BYTES,            0x29, NULL)
    CXD224X_XDEF1(PN_ATR_REQ_CONFIG,               0x2A, NULL)
    CXD224X_XDEF2(RFU,                             0x2B,0x2F, NULL)
    // Listen Mode NFC-A Discovery Parameters
    CXD224X_XDEF1(LA_BIT_FRAME_SDD,                0x30, NULL)
    CXD224X_XDEF1(LA_PLATFORM_CONFIG,              0x31, NULL)
    CXD224X_XDEF1(LA_SEL_INFO,                     0x32, NULL)
    CXD224X_XDEF1(LA_NFCID1,                       0x33, NULL)
    CXD224X_XDEF2(RFU,                             0x34,0x37, NULL)
    // Listen Mode NFC-B Discovery Parameters
    CXD224X_XDEF1(LB_SENSB_INFO,                   0x38, NULL)
    CXD224X_XDEF1(LB_NFCID0,                       0x39, NULL)
    CXD224X_XDEF1(LB_APPLICATION_DATA,             0x3A, NULL)
    CXD224X_XDEF1(LB_SFGI,                         0x3B, NULL)
    CXD224X_XDEF1(LB_ADC_FO,                       0x3C, NULL)
    CXD224X_XDEF2(RFU,                             0x3D,0x3F, NULL)
    // Listen Mode NFC-F Discovery Parameters
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_1,            0x40, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_2,            0x41, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_3,            0x42, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_4,            0x43, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_5,            0x44, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_6,            0x45, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_7,            0x46, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_8,            0x47, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_9,            0x48, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_10,           0x49, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_11,           0x4A, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_12,           0x4B, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_13,           0x4C, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_14,           0x4D, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_15,           0x4E, NULL)
    CXD224X_XDEF1(LF_T3T_IDENTIFIERS_16,           0x4F, NULL)
    CXD224X_XDEF1(LF_PROTOCOL_TYPE,                0x50, NULL)
    CXD224X_XDEF1(LF_T3T_PMM,                      0x51, NULL)
    CXD224X_XDEF1(LF_T3T_MAX,                      0x52, NULL)
    CXD224X_XDEF1(LF_T3T_FLAGS,                    0x53, NULL)
    CXD224X_XDEF1(LF_CON_BITR_F,                   0x54, NULL)
    CXD224X_XDEF1(LF_ADV_FEAT,                     0x55, NULL)
    CXD224X_XDEF2(RFU,                             0x56,0x57, NULL)
    // Listen Mode ISO-DEP Discovery Parameters
    CXD224X_XDEF1(LI_FWI,                          0x58, NULL)
    CXD224X_XDEF1(LA_HIST_BY,                      0x59, NULL)
    CXD224X_XDEF1(LB_H_INFO_RESP,                  0x5A, NULL)
    CXD224X_XDEF1(LI_BIT_RATE,                     0x5B, NULL)
    CXD224X_XDEF2(RFU,                             0x5C,0x5F, NULL)
    // Listen Mode NFC-DEP Discovery Parameters
    CXD224X_XDEF1(LN_WT,                           0x60, NULL)
    CXD224X_XDEF1(LN_ATR_RES_GEN_BYTES,            0x61, NULL)
    CXD224X_XDEF1(LN_ATR_RES_CONFIG,               0x62, NULL)
    CXD224X_XDEF2(RFU,                             0x63,0x7F, NULL)
    // Other Parameters
    CXD224X_XDEF1(RF_FIELD_INFO,                   0x80, NULL)
    CXD224X_XDEF1(RF_NFCEE_ACTION,                 0x81, NULL)
    CXD224X_XDEF1(NFCDEP_OP,                       0x82, NULL)
    CXD224X_XDEF2(RFU,                             0x83,0x9F, NULL)
    // Outside of Poll 15693 Provisions
    CXD224X_XDEF1(PV_DATA_CODING,                  0xA0, NULL)
    CXD224X_XDEF1(PV_AFI,                          0xA1, NULL)
    CXD224X_XDEF1(PV_SLOT,                         0xA2, NULL)
    CXD224X_XDEF1(PV_MASK_LEN,                     0xA3, NULL)
    CXD224X_XDEF1(PV_MASK_VAL,                     0xA4, NULL)
    CXD224X_XDEF1(PV_SUB_CARRIER,                  0xA5, NULL)
    CXD224X_XDEF1(PV_REPLY_SPEED,                  0xA6, NULL)
    // Reserved for Proprietary Use
    CXD224X_XDEF2(Reserved,                        0xA7,0xB7, NULL)
    // DCLB Poll B
    CXD224X_XDEF1(PDB_AFI,                         0xB8, NULL)
    CXD224X_XDEF1(PDB_ATTRIB_PARAM1,               0xB9, NULL)
    // Reserved for Proprietary Use
    CXD224X_XDEF2(Reserved,                        0xBA,0xBF, NULL)
    // DCLB Poll ISO-DEP
    CXD224X_XDEF1(PDB_H_INFO,                      0xC0, NULL)
    CXD224X_XDEF1(PIB_BIT_RATE,                    0xC1, NULL)
    // Reserved for Proprietary Use
    CXD224X_XDEF2(Reserved,                        0xC2,0xCF, NULL)
    // Proprietary Parameters
    CXD224X_XDEF1(T_LPS1,                          0xD0, NULL)
    CXD224X_XDEF1(LISTEN_EXTENSION_TIME,           0xD1, NULL)
    CXD224X_XDEF1(LPP_N,                           0xD2, NULL)
    CXD224X_XDEF1(FSAM_SHUTDOWN_START_TIME,        0xD3, NULL)
    CXD224X_XDEF1(TYPEF_IGNORE_CNT_VALUE,          0xD4, NULL)
    CXD224X_XDEF1(TYPEF_IGNORE_CNT_CLEAR_LOOP_NUM, 0xD5, NULL)
    CXD224X_XDEF1(PN_ACTV_ATR_REQ_TIMEOUT,         0xD6, NULL)
    CXD224X_XDEF1(LN_ACTV_TIMER_VAL_NON_COMM,      0xD7, NULL)
    CXD224X_XDEF1(LI_FORCE_CID_SUPPORT,            0xD8, NULL)
    // Reserved for Proprietary Use
    CXD224X_XDEF2(Reserved,                        0xD9,0xDF, NULL)
    // KOVIO
    CXD224X_XDEF1(PK_TIME,                         0xE0, NULL)
    // Reserved for Proprietary Use
    CXD224X_XDEF2(Reserved,                        0xE1,0xFE, NULL)
    // Reserved for Extension
    CXD224X_XDEF1(RFU,                             0xFF, NULL)
};

/* Global NFC data */
tCXD224X_PARSE  g_cxd224x;

void cxd224x_init()
{
    memset( &g_cxd224x, 0, sizeof(g_cxd224x));
//    g_cxd224x.rf_intf_activated_ntf_only=0;
    g_cxd224x.data_write_flag=-1;
//    g_cxd224x.parse_dtype=0;
//    g_cxd224x.rf_intf_activated_ntf_cnt=0;
//    g_cxd224x.status_color=0;
}

void cxd224x_parse(const char *str, const uint8_t *p, int32_t len, int32_t detail_flag)
{
uint8_t mt;
uint8_t gid;
uint8_t oid;
uint8_t conn_id=0;
uint32_t i;
const CXD224X_NCI_CMD_REC *find;
uint32_t indent=5;


    if(str== NULL || p == NULL || len < 3){
        return;
    }

#if 1
    // these logs are too busy, so skip to display
    uint8_t buf1[] = {0x00, 0x00, 0x02, 0x00, 0x00};          // NCI DATA:LLCP SYMM
    uint8_t buf2[] = {0x60, 0x06, 0x03, 0x01, 0x00, 0x01};    // NCI CORE_CONN_CREDITS_NTF
    uint8_t buf3[] = {0x61, 0x07, 0x01, 0x01}; // NCI RF_FIELD_INFO_NTF
    uint8_t buf4[] = {0x61, 0x07, 0x01, 0x00}; // NCI RF_FIELD_INFO_NTF

    if(len == sizeof(buf1)){
        if(memcmp(p, buf1, len)==0){
            return;
        }
    }

    if(len == sizeof(buf2)){
        if(memcmp(p, buf2, len)==0){
            return;
        }
    }
    if(len == sizeof(buf3)){
        if(memcmp(p, buf3, len)==0){
            return;
        }
    }
    if(len == sizeof(buf4)){
        if(memcmp(p, buf4, len)==0){
            return;
        }
    }
#endif

#if 0
    cxd224x_dump1(indent, str, p, len);
    indent += 1;
#endif

    mt  = (p[0]>>5)&0x7;
    gid = p[0]&0x0F;
    oid = p[1]&0x3F;
    conn_id=0;
    if(mt==0){
        conn_id = p[0]&0x0F;
    }
    if(len < p[2] + 3){
        return;
    }

    find = NULL;
    for(i=0;i<CXD224X_CNT(cxd224x_nci_r100_tbl102);i++){
        const CXD224X_NCI_CMD_REC *r=&cxd224x_nci_r100_tbl102[i];
        if(mt == r->m_mt && gid == r->m_gid && (oid >= r->m_oid_start && oid <= r->m_oid_end)){
            find = r;
            break;
        }
    }

    if(g_cxd224x.rf_intf_activated_ntf_only==1){
        if(find != NULL){
            if(strcmp(find->m_pName, "RF_INTF_ACTIVATED_NTF")==0){
                g_cxd224x.rf_intf_activated_ntf_cnt++;
                cxd224x_debug(indent-1, ">>> cnt=%d", g_cxd224x.rf_intf_activated_ntf_cnt);
                cxd224x_debug(indent, "%s", find->m_pName);
                if(find->m_func != NULL){
                    (*find->m_func)(indent+1, p+3,len-3);
                }
            }
        }
    }else{
        if(mt == 0){    // DATA PACKET
            cxd224x_debug(indent, "DATA");
            parse_data(indent+1, conn_id, p+3,len-3);
        }else{
            if(find == NULL){
                const char *str1;

                if(mt == 0x01){
                    str1="CMD";
                }else if(mt == 0x02){
                    str1="RSP";
                }else if(mt == 0x03){
                    str1="NTF";
                }else{
                    str1="XXX";
                }
                cxd224x_debug(indent, "XXX_XXX_%s", str1);
                return;
            }else{
                cxd224x_debug(indent, "%s", find->m_pName);
                if(mt == 0x02){    // RSP
                    parse_rsp_status_code(indent+1, p+3,len-3);
                }
                if(find->m_func != NULL){
                    (*find->m_func)(indent+1, p+3,len-3);
                }
            }
        }
    }
}

//---------------------------------------------
// parse response status code
//---------------------------------------------
static int32_t parse_rsp_status_code(uint32_t indent, const uint8_t *p, int32_t len)
{
int color_flag=0;

    CXD224X_CHECK_NULL(p,-1)
    
    if(g_cxd224x.status_color){
        if( p!= NULL && len > 0){
            if(*p != 0x00){
                color_flag=1;
            }
        }
    }
    if(color_flag)
        cxd224x_debug(indent,"\033[31m");    // red color

    CXD224X_PRINT_1BF(indent, p, len, "status", get_nci_r100_tbl94);
    if(color_flag)
        cxd224x_debug(indent,"\033[39m");    // normal color

    return 0;
}

//---------------------------------------------
// no parse operation
//---------------------------------------------
static int32_t parse_none(uint32_t indent, const uint8_t *p, int32_t len)
{
    return 0;
}

//*********************************************
// CORE
//*********************************************
//---------------------------------------------
// CORE_RESET
//---------------------------------------------
static const char* get_reset_type(uint8_t id)
{
    if(id==0x00){
        return "Keep Config";
    }else if(id==0x01){
        return "Reset Config";
    }else{
        return "RFU";
    }
}

static const char* get_reset_reason(uint8_t id)
{
    if(id == 0x00){
        return "Unspeciofied reason";
    }else if(id>=0x01&&id<=0x9F){
        return "RFU";
    }else{
        return "For proprietary use";
    }
}

static int32_t parse_core_reset_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p,len, "Reset Type", get_reset_type);
    return 0;
}

static int32_t parse_core_reset_rsp(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_SKIP(p,len,1);
    CXD224X_PRINT_1BN(indent, p, len, "NCV version");
    CXD224X_PRINT_1BF(indent, p, len, "ConfigurationStatus", get_nci_r100_tbl7);
    return 0;
}

static int32_t parse_core_reset_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p, len, "Reason Code", get_reset_reason);
    CXD224X_PRINT_1BF(indent, p, len, "Configuration Status", get_nci_r100_tbl7);
    return 0;
}

//---------------------------------------------
// CORE_INIT
//---------------------------------------------
static int32_t parse_core_init_rsp(uint32_t indent, const uint8_t *p, int32_t len)
{
int32_t i;
uint8_t rf_cnt;
const uint8_t *p0;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_SKIP(p,len,1);
    p0 = p;
    CXD224X_PRINT_4BN(indent, p, len, "NFCC Features")    

    print_nci_r100_tbl9_obt0(indent+1, p0[0]);
    print_nci_r100_tbl9_obt1(indent+1, p0[1]);
    print_nci_r100_tbl9_obt2(indent+1, p0[2]);
    print_nci_r100_tbl9_obt3(indent+1, p0[3]);

    CXD224X_GET_HEX(p, len, 1, &rf_cnt)
    cxd224x_debug(indent, "Number of Supported RF Interfaces=0x%02X", rf_cnt);
    for(i=0;i<rf_cnt;i++){
        CXD224X_PRINT_1BF(indent+1, p, len, "+Supported RF Interface", get_nci_r100_tbl99);
    }
    CXD224X_PRINT_1BN(indent, p, len, "Max Logical Connections");
    CXD224X_PRINT_2BN(indent, p, len, "Max Routing Table Size")
    CXD224X_PRINT_1BN(indent, p, len, "Max Control Packet Payload Size")
    CXD224X_PRINT_2BN(indent, p, len, "Max Size for Large Parameters")
    CXD224X_PRINT_1BN(indent, p, len, "Manufacturer ID")
    CXD224X_PRINT_4BN(indent, p, len, "Manufacturer Specific Information")
    return 0;
}

//---------------------------------------------
// CORE_SET_CONFIG
//---------------------------------------------
static int32_t parse_core_set_config_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p, len, 1, &cnt)
    cxd224x_debug(indent, "Number of Parameters=0x%02X", cnt);
    cxd224x_debug(indent, "Parameter >>>");
    print_tlv_by_cnt(indent+1, p, len, cnt, 
        cxd224x_nci_r100_tbl101, CXD224X_CNT(cxd224x_nci_r100_tbl101));

    return 0;
}

static int32_t parse_core_set_config_rsp(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_SKIP(p,len,1);
    CXD224X_GET_HEX(p, len, 1, &cnt)
    cxd224x_debug(indent, "Number of Parameters=0x%02X", cnt);
    if(len < cnt){
        return -1;
    }
    cxd224x_debug(indent, "Parameter ID >>>");
    print_tlv_tag(indent+1, p, cnt, 
        cxd224x_nci_r100_tbl101, CXD224X_CNT(cxd224x_nci_r100_tbl101));
    return 0;
}

//---------------------------------------------
// CORE_GET_CONFIG
//---------------------------------------------
static int32_t parse_core_get_config_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p, len, 1, &cnt)
    cxd224x_debug(indent, "Number of Parameters=0x%02X", cnt);
    if(len < cnt){
        return -1;
    }
    cxd224x_debug(indent, "Parameter ID >>>");
    print_tlv_tag(indent+1, p, cnt, 
        cxd224x_nci_r100_tbl101, CXD224X_CNT(cxd224x_nci_r100_tbl101));
    return 0;
}

static int32_t parse_core_get_config_rsp(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_SKIP(p,len,1);
    CXD224X_GET_HEX(p, len, 1, &cnt)
    cxd224x_debug(indent, "Number of Parameters=0x%02X", cnt);
    cxd224x_debug(indent, "Parameter >>>");
    print_tlv_by_cnt(indent+1, p, len, cnt, 
        cxd224x_nci_r100_tbl101, CXD224X_CNT(cxd224x_nci_r100_tbl101));
    return 0;
}

//---------------------------------------------
// CORE_CONN_CREATE
//---------------------------------------------
//---------------------------
// Table 15
//---------------------------
static int32_t parse_func_nci_r100_tbl15_rf(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"RF Discovery ID");
    CXD224X_PRINT_1BF(indent, p,len,"RF Protocol", get_nci_r100_tbl98);
    return 0;
}

static int32_t parse_func_nci_r100_tbl15_nfcee(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"NFCEE ID");
    CXD224X_PRINT_1BF(indent, p,len,"NFCEE Interface Protocol", get_nci_r100_tbl100);
    return 0;
}

static const CXD224X_NCI_ID_NAME_REC cxd224x_nci_r100_tbl15[] = {
    {"RF",                  0x00, 0x00, &parse_func_nci_r100_tbl15_rf},
    {"NFCEE",               0x01, 0x01, &parse_func_nci_r100_tbl15_nfcee},
    {"RFU",                 0x02, 0x9F, NULL},
    {"For_proprietary_use", 0xA0, 0xFF, NULL},
};

static int32_t parse_core_conn_create_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
    uint8_t cnt;
    
    set_data_type_desttype( p,len);

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p,len,"Destination Type", get_nci_r100_tbl12);
    CXD224X_GET_HEX(p,len,1,&cnt);
    cxd224x_debug(indent, "Number of Destination-specific Parameters=0x%02X", cnt);
    print_tlv_by_cnt(indent+1, p, len, cnt, 
        cxd224x_nci_r100_tbl15, CXD224X_CNT(cxd224x_nci_r100_tbl15));
    return 0;
}

static int32_t parse_core_conn_create_rsp(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_SKIP(p,len,1);
    CXD224X_PRINT_1BN(indent, p,len,"Max Data Packet Payload Size");
    CXD224X_PRINT_1BN(indent, p,len,"Initial Number of Credits");
    CXD224X_PRINT_1BN(indent, p,len,"Conn ID");
    return 0;
}

//---------------------------------------------
// CORE_CONN_CLOSE
//---------------------------------------------
static int32_t parse_core_conn_close_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"Conn ID");
    return 0;
}

//---------------------------------------------
// CORE_CONN_CREDITS_NTF
//---------------------------------------------
static int32_t parse_core_conn_credits_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;
int32_t i;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&cnt);
    cxd224x_debug(indent, "Number of Entries=0x%02X", cnt);
    for(i=0;i<cnt;i++){
        CXD224X_PRINT_1BN(indent+1, p,len,"+Conn ID");
        CXD224X_PRINT_1BN(indent+1, p,len,"|Credits");
    }
    return 0;
}

//---------------------------------------------
// CORE_INTERFACE_ERROR_NTF
//---------------------------------------------
static int32_t parse_core_interface_error_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p, len, "status", get_nci_r100_tbl94);
    CXD224X_PRINT_1BN(indent, p,len,"Conn ID");
    return 0;
}

//*********************************************
// RF
//*********************************************
//---------------------------------------------
// RF_DISCOVER_MAP
//---------------------------------------------
static int32_t parse_rf_discover_map_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;
uint8_t mode;
int32_t i;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&cnt);
    cxd224x_debug(indent, "Number of Mapping Configurations=0x%02X", cnt);
    cxd224x_debug(indent, "Mapping Configuration >>>");
    for(i=0;i<cnt;i++){
        CXD224X_PRINT_1BF(indent+1, p,len,"+RF Protocol ", get_nci_r100_tbl98);
        CXD224X_GET_HEX(p,len,1,&mode);
        cxd224x_debug(indent+1, "|mode:LISTEN=%d,POLL=%d", (mode&0x02?1:0), (mode&0x01?1:0));
        CXD224X_PRINT_1BF(indent+1, p,len,"|RF Interface ", get_nci_r100_tbl99);
    }
    return 0;
}

//---------------------------------------------
// RF_SET_LISTEN_MODE_ROUTING
//---------------------------------------------
static int32_t parse_func_nci_r100_tbl47(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"Route      ");
    CXD224X_PRINT_1BF(indent, p,len,"Power State", get_nci_r100_tbl50);
    CXD224X_PRINT_1BF(indent, p,len,"Technology ", get_nci_r100_tbl95);
    return 0;
}

static int32_t parse_func_nci_r100_tbl48(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"Route      ");
    CXD224X_PRINT_1BF(indent, p,len,"Power State", get_nci_r100_tbl50);
    CXD224X_PRINT_1BF(indent, p,len,"Protocol   ", get_nci_r100_tbl98);
    return 0;
}

static int32_t parse_func_nci_r100_tbl49(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"Route      ");
    CXD224X_PRINT_1BF(indent, p,len,"Power State", get_nci_r100_tbl50);
    cxd224x_dump2(indent, "AID", p, len);
    return 0;
}

//--------------------------
// Table 46
//--------------------------
static const CXD224X_NCI_ID_NAME_REC cxd224x_nci_r100_tbl46[] = {
    {"Technology-based routing entry",  0x00, 0x00, &parse_func_nci_r100_tbl47},
    {"Protocol-based routing entry",    0x01, 0x01, &parse_func_nci_r100_tbl48},
    {"AID-based routing entry",         0x02, 0x02, &parse_func_nci_r100_tbl49},
    {"RFU",                             0x03, 0x9F, NULL},
    {"For_proprietary_use",             0xA0, 0xFF, NULL},
};

static char *get_str_table95(uint8_t n)
{
char *p;

    switch(n){
    case 0x00:
         p="TYPE_A";
        break;
    case 0x01:
         p="TYPE_B";
        break;
    case 0x02:
         p="TYPE_F";
        break;
    case 0x03:
         p="TYPE_15693";
        break;
    default:
         p="--";
        break;
    }
    return p;
}

static char *get_str_table98(uint8_t n)
{
char *p;

    switch(n){
    case 0x00:
         p="UNDEF";
        break;
    case 0x01:
         p="T1T";
        break;
    case 0x02:
         p="T2T";
        break;
    case 0x03:
         p="T3T";
        break;
    case 0x04:
         p="ISO_DEP";
        break;
    case 0x05:
         p="NFC_DEP";
        break;
    default:
         p="--";
        break;
    }
    return p;
}


static int32_t parse_rf_set_listen_mode_routing_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
#if 0 // org
uint8_t cnt;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p,len,"More ", get_nci_r100_tbl45);
    CXD224X_GET_HEX(p,len,1,&cnt);
    cxd224x_debug(indent, "Number of Routing Entries=0x%02X", cnt);
    cxd224x_debug(indent, "type||route|BA_OFF|SW_OFF|SW_ON|");

    print_tlv_by_cnt(indent+1, p, len, cnt, 
        cxd224x_nci_r100_tbl46, CXD224X_CNT(cxd224x_nci_r100_tbl46));
    return 0;
#else // new
#define CXD224X_PARSE_AID_MAX_LEN 16

uint8_t cnt;
const uint8_t *p0;
int32_t i;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p,len,"More ", get_nci_r100_tbl45);
    CXD224X_GET_HEX(p,len,1,&cnt);
    cxd224x_debug(indent, "Number of Routing Entries=0x%02X", cnt);
    cxd224x_debug(indent, "  type|nfcee_id|BA_OFF|SW_OFF|SW_ON|");
    cxd224x_debug(indent, "  ----+--------+------+------+-----+-------");

    p0=p;
    while(1){
        uint8_t tag;
        uint8_t len0;
        char *p_type="";
        uint8_t nfcee_id=0xFF;
        uint8_t power_state=0;
        char *p_str="";
        char aid_buf[CXD224X_PARSE_AID_MAX_LEN*2+1];

        if(cnt == 0){
            break;
        }

        if(len >= 2){
            tag=*p0++;
            len0=*p0++;
            len -= 2;    
        }else{
            break;
        }
        if(len >= len0){
            if(tag==0x00){
                p_type="TEC__";
                if(len0 == 3){
                    nfcee_id = *p0;
                    power_state = *(p0+1);
                    p_str=get_str_table95(*(p0+2));
                }
            }else if(tag==0x01){
                p_type="PROT_";
                if(len0 == 3){
                    nfcee_id = *p0;
                    power_state = *(p0+1);
                    p_str=get_str_table98(*(p0+2));
                }

            }else if(tag==0x02){
                p_type="AID__";
                if(len0 >= 2 && len0 <= 2+CXD224X_PARSE_AID_MAX_LEN){  
                    nfcee_id = *p0;
                    power_state = *(p0+1);
                    for(i=0;i<len0-2;i++){
                        sprintf(&aid_buf[i*2], "%02X", *(p0+2+i));
                    }
                    aid_buf[i*2]='\0';
                    p_str=aid_buf;                    
                }
            }else if(tag>=0x03 && tag<=0x9F){
                p_type="RFU";
            }else{
                p_type="prop";
            }

        }else{
            break;
        }
        cxd224x_debug(indent, " %5s|%02X      |%6d|%6d|%5d|%s", p_type, nfcee_id, (power_state&0x04?1:0),(power_state&0x02?1:0),(power_state&0x01), p_str);

        p0 += len0;
        len -= len0;
        cnt--;
    }
    return 0;
#endif
}

//---------------------------------------------
// RF_GET_LISTEN_MODE_ROUTING
//---------------------------------------------
//     same operation as parse_rf_set_listen_mode_routing_cmd()
static int32_t parse_rf_get_listen_mode_routing_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p,len,"More ", get_nci_r100_tbl45);
    CXD224X_GET_HEX(p,len,1,&cnt);
    cxd224x_debug(indent, "Number of Routing Entries=0x%02X", cnt);
    print_tlv_by_cnt(indent+1, p, len, cnt, 
        cxd224x_nci_r100_tbl46, CXD224X_CNT(cxd224x_nci_r100_tbl46));
    return 0;
}

//---------------------------------------------
// RF_DISCOVER
//---------------------------------------------
static int32_t parse_rf_discover_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;
int32_t i;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&cnt);
    cxd224x_debug(indent, "Number of Configurations=0x%02X", cnt);
    for(i=0;i<cnt;i++){
        CXD224X_PRINT_1BF(indent+1, p,len,"+RF Technology and Mode", get_nci_r100_tbl96);
        CXD224X_PRINT_1BN(indent+1, p,len,"|Discovery Frequency");
    }
    return 0;
}

static const char* get_notification_type(uint8_t id)
{
    if(id==0){
        return "Last Notification";
    }else if(id==1){
        return "Last Notification (due to NFCC reaching it's resource limit)";
    }else if(id==2){
        return "More Notification to follow";
    }else{
        return "RFU";
    }
}

static int32_t parse_nfc_r100_tbl54(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t len1;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_2BN(indent, p, len, "SENS_RES Response");

    CXD224X_GET_HEX(p,len,1,&len1);
    cxd224x_debug(indent, "NFCID1 Length=0x%02X", len1);
    if(len < len1){
        return -1;
    }
    cxd224x_dump2(indent+1, "NFCID1", p, len1);
    p += len1;
    len -= len1;

    CXD224X_GET_HEX(p,len,1,&len1);
    cxd224x_debug(indent, "SEL_RES Response Length=0x%02X", len1);
    if(len < len1){
        return -1;
    }
    cxd224x_dump2(indent+1, "SEL_RES Response", p, len1);

    return 0;
}

static int32_t parse_nfc_r100_tbl56(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t len1;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&len1);
    cxd224x_debug(indent, "SENSB_RES Response Length=0x%02X", len1);

    if(len < len1){
        return -1;
    }
    cxd224x_dump2(indent+1, "SENSB_RES Response", p, len1);
    return 0;
}

static int32_t parse_nfc_r100_tbl58(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t bit_rate;
uint8_t len1; 
const char *str;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&bit_rate);
    if(bit_rate == 0x01){
        str="212 kbps";
    }else if(bit_rate == 0x02){
        str="424 kbps";
    }else{
        str="RFU";
    }
    cxd224x_debug(indent, "Bit Rate=0x%02X(%s)", bit_rate, str);

    CXD224X_GET_HEX(p,len,1,&len1);
    cxd224x_debug(indent, "SENSF_RES Response Length=0x%02X", len1);

    if(len < len1){
        return -1;
    }
    cxd224x_dump2(indent+1, "SENSF_RES Response", p, len1);
    return 0;
}

static int32_t parse_nfc_r100_tbl59(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t len1;

    CXD224X_CHECK_NULL(p,-1)

    CXD224X_GET_HEX(p,len,1,&len1);
    cxd224x_debug(indent, "Local NFCID2 Length=0x%02X", len1);

    if(len < len1){
        return -1;
    }
    cxd224x_dump2(indent+1, "Local NFCID2", p, len1);
    return 0;
}

static int32_t parse_rf_discover_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t param_len;
uint8_t t_mode;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"RF Discovery ID");
    CXD224X_PRINT_1BF(indent, p,len,"RF Protocol",get_nci_r100_tbl98);
    CXD224X_PRINT_1BF_GET_DATA(indent, p,len,"RF Technology and Mode",get_nci_r100_tbl96, t_mode);
    CXD224X_GET_HEX(p,len,1,&param_len);
    cxd224x_debug(indent, "Length of RF Technology Specific Parameters=0x%02X", param_len);
    if(len < param_len+1){    // +1 for NotificationType
        return -1;
    }
    cxd224x_dump2(indent+1, "RF Technology Specific Parameters", p, param_len);

    if(t_mode == 0x00){         // NFC_A_PASSIVE_POLL_MODE
        parse_nfc_r100_tbl54(indent+2, p, param_len);
    }else if(t_mode == 0x01){   // NFC_B_PASSIVE_POLL_MODE
        parse_nfc_r100_tbl56(indent+2, p, param_len);
    }else if(t_mode == 0x02){   // NFC_F_PASSIVE_POLL_MODE
        parse_nfc_r100_tbl58(indent+2, p, param_len);
    }else{
        // do nothing
    }

    p += param_len;
    len -= param_len;
    CXD224X_PRINT_1BF(indent, p,len,"Notification Type", get_notification_type);
    return 0;
}

//---------------------------------------------
// RF_DISCOVER_SELECT
//---------------------------------------------
static int32_t parse_rf_discover_select_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"RF Discovery ID");
    CXD224X_PRINT_1BF(indent, p,len,"RF Protocol",get_nci_r100_tbl98);
    CXD224X_PRINT_1BF(indent, p,len,"RF Interface",get_nci_r100_tbl99);
    return 0;
}
//---------------------------------------------
// RF_INTF_ACTIVATED_NTF
//---------------------------------------------
static int32_t parse_nfc_r100_tbl76(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t len1;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&len1);
    cxd224x_debug(indent, "RATS Response Length=0x%02X", len1);
    if(len < len1){
        return -1;
    }
    cxd224x_dump2(indent+1, "RATS Response", p, len1);

    return 0;
}

static int32_t parse_nfc_r100_tbl77(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t len1;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&len1);
    cxd224x_debug(indent, "ATTRIB Response Length=0x%02X", len1);
    if(len < len1){
        return -1;
    }
    cxd224x_dump2(indent+1, "ATTRIB Response", p, len1);

    return 0;
}

static int32_t parse_nfc_r100_tbl78(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"RATS Command Param");
    return 0;
}

static int32_t parse_nfc_r100_tbl79(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t len1;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&len1);
    cxd224x_debug(indent, "ATTRIB Command Length=0x%02X", len1);
    if(len < len1){
        return -1;
    }
    cxd224x_dump2(indent+1, "ATTRIB Command", p, len1);

    return 0;
}

static int32_t parse_nfc_r100_tbl82(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t len1;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&len1);
    cxd224x_debug(indent, "ATR_RES Response Length=0x%02X", len1);
    if(len < len1){
        return -1;
    }
    cxd224x_dump2(indent+1, "ATR_RES Response", p, len1);

    return 0;
}

static int32_t parse_nfc_r100_tbl83(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t len1;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&len1);
    cxd224x_debug(indent, "ATR_REQ Command Length=0x%02X", len1);
    if(len < len1){
        return -1;
    }
    cxd224x_dump2(indent+1, "ATR_REQ Command", p, len1);

    return 0;
}

static int32_t parse_rf_intf_activated_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t rf_param_len;
uint8_t act_param_len;
uint8_t t_mode;
uint8_t rf_intf;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"RF Discovery ID");
    if(len > 0){
        uint8_t rf_interface=*p;
        // according to the NCI doc
        //   NFCForum-TS-NCI-1.0.pdf  
        //   page84
        // If this contains a value of 0x00 (NFCEE Direct RF
        // Interface) then all following parameters SHALL contain a
        // value of 0 and SHALL be ignored.
        if(rf_interface==0){
            return 0;       
        }
    }
    CXD224X_PRINT_1BF_GET_DATA(indent, p,len,"RF Interface",get_nci_r100_tbl99, rf_intf);
    set_data_type_protocol( p, len );
    CXD224X_PRINT_1BF(indent, p,len,"RF Protocol",get_nci_r100_tbl98);
    CXD224X_PRINT_1BF_GET_DATA(indent, p,len,"Activation RF Technology and Mode",get_nci_r100_tbl96, t_mode);
    CXD224X_PRINT_1BN(indent, p,len,"Max Data Packet Payload Size");
    CXD224X_PRINT_1BN(indent, p,len,"Initial Number of Credits");

    CXD224X_GET_HEX(p,len,1,&rf_param_len);
    cxd224x_debug(indent, "Length of RF Technology Specific Parameters=0x%02X", rf_param_len);
    if(len < rf_param_len){
        return -1;
    }
    cxd224x_dump2(indent+1, "RF Technology Specific Parameters", p, rf_param_len);

    if(t_mode == 0x00){            // NFC_A_PASSIVE_POLL_MODE
        parse_nfc_r100_tbl54(indent+2, p, rf_param_len);
    }else if(t_mode == 0x80){     // NFC_A_PASSIVE_LISTEN_MODE
        // do nothing, table 55 is empty
    }else if(t_mode == 0x01){     // NFC_B_PASSIVE_POLL_MODE
        parse_nfc_r100_tbl56(indent+2, p, rf_param_len);
    }else if(t_mode == 0x81){     // NFC_B_PASSIVE_LISTEN_MODE
        // do nothing, table 57 is empty
    }else if(t_mode == 0x02){     // NFC_F_PASSIVE_POLL_MODE
        parse_nfc_r100_tbl58(indent+2, p, rf_param_len);
    }else if(t_mode == 0x82){     // NFC_F_PASSIVE_LISTEN_MODE
        parse_nfc_r100_tbl59(indent+2, p, rf_param_len);
    }else{
        // do nothing
    }
    p += rf_param_len;
    len -= rf_param_len;

    CXD224X_PRINT_1BF(indent, p,len,"Data Exchange RF Technology and Mode",get_nci_r100_tbl96);
    CXD224X_PRINT_1BF(indent, p,len,"Data Exchange Transmit Bit Rate",get_nci_r100_tbl97);
    CXD224X_PRINT_1BF(indent, p,len,"Data Exchange Receive  Bit Rate",get_nci_r100_tbl97);

    CXD224X_GET_HEX(p,len,1,&act_param_len);
    cxd224x_debug(indent, "Length of Activation Parameters=0x%02X", act_param_len);
    if(len < act_param_len){
        return -1;
    }
    cxd224x_dump2(indent+1, "Activation Parameters", p, act_param_len);

    if(rf_intf == 0x00){            // NFCEE_Direct_RF_Interface
        // TBD?
    }else if(rf_intf == 0x01){        // Frame_RF_Interface
        // There are no Activation Parameters defined for the Frame RF Interface
    }else if(rf_intf == 0x02){        // ISO_DEP_RF_Interface
        // see Table 76, Table 77, Table 78, and Table 79.
        if(t_mode == 0x00){                // NFC_A_PASSIVE_POLL_MODE
            parse_nfc_r100_tbl76(indent+2, p, rf_param_len);
        }else if(t_mode == 0x01){         // NFC_B_PASSIVE_POLL_MODE
            parse_nfc_r100_tbl77(indent+2, p, rf_param_len);
        }else if(t_mode == 0x80){         // NFC_A_PASSIVE_LISTEN_MODE
            parse_nfc_r100_tbl78(indent+2, p, rf_param_len);
        }else if(t_mode == 0x81){         // NFC_B_PASSIVE_LISTEN_MODE
            parse_nfc_r100_tbl79(indent+2, p, rf_param_len);
        }else{
            // do nothing
        }
    }else if(rf_intf == 0x03){        // NFC_DEP_RF_Interface
        //  see Table 82 and Table 83.
        if(t_mode == 0x00 || t_mode == 0x02){           // NFC_A_PASSIVE_POLL_MODE or NFC_F_PASSIVE_POLL_MODE
            parse_nfc_r100_tbl82(indent+2, p, rf_param_len);
        }else if(t_mode == 0x80 || t_mode == 0x82){    // NFC_A_PASSIVE_LISTEN_MODE or NFC_F_PASSIVE_LISTEN_MODE
            parse_nfc_r100_tbl83(indent+2, p, rf_param_len);
        }else{
            // do nothing
        }
    }else{
        // do nothing 
    }

    return 0;
}

//---------------------------------------------
// RF_DEACTIVATE
//---------------------------------------------
static int32_t parse_rf_deactivate_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p,len,"Deactivation Type", get_nci_r100_tbl63);
    clr_data_type();
    return 0;
}

static int32_t parse_rf_deactivate_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p,len,"Deactivation Type", get_nci_r100_tbl63);
    CXD224X_PRINT_1BF(indent, p,len,"Deactivation Reason", get_nci_r100_tbl64);
    clr_data_type();
    return 0;
}

//---------------------------------------------
// RF_FIELD_INFO
//---------------------------------------------
static int32_t parse_rf_field_info_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p,len,"RF Field Status", get_nci_r100_tbl21);
    return 0;
}

//---------------------------------------------
// RF_T3T_POLLING
//---------------------------------------------
static int32_t parse_rf_t3t_polling_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    if(len < 4){
        return -1;
    }
    cxd224x_dump2(indent, "SENSF_REQ_PARAMS", p, 4);
    return 0;
}

static int32_t parse_rf_t3t_polling_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t status, cnt;
uint8_t i;
const char *fmt;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&status);
    CXD224X_GET_HEX(p,len,1,&cnt);
    cxd224x_debug(indent, "status_code=0x%02X(%s)", status, get_nci_r100_tbl94(status));
    cxd224x_debug(indent, "Number of Responses=%d", cnt);

    if(cnt<=9){
        fmt="SENSF_RES[%d]";
    }else if(cnt>=10&&cnt<=99){
        fmt="SENSF_RES[%2d]";
    }else{
        fmt="SENSF_RES[%3d]";
    }

    for(i=0;i<cnt;i++){
        uint8_t data_len;
        char buf[64];
        if(len < 1){
            break;
        }
        CXD224X_GET_HEX(p,len,1,&data_len);
        if(len < data_len){
            break;
        }
        sprintf(buf, fmt, i);
        cxd224x_dump2(indent+1, buf, p, data_len);

        p += data_len;
        len -= data_len;
    }
    return 0;
}

//---------------------------------------------
// RF_NFCEE_ACTION_NTF
//---------------------------------------------
static int32_t parse_rf_nfcee_action_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t s_data_len;
uint8_t data;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"NFCEE ID");
    CXD224X_PRINT_1BF_GET_DATA(indent, p,len,"Trigger", get_nci_r100_tbl69, data);

    CXD224X_GET_HEX(p,len,1,&s_data_len);
    if(len < s_data_len){
        return -1;
    }
    cxd224x_dump2(indent+1, "Supporting Data", p, s_data_len);

    if(data==0x01){
        CXD224X_PRINT_1BF(indent+2, p,s_data_len,"RF Protocol", get_nci_r100_tbl98);
    }else if(data==0x02){
        CXD224X_PRINT_1BF(indent+2, p,s_data_len,"RF Technology", get_nci_r100_tbl95);
    }else{
        // do nothing
    }

    return 0;
}

//---------------------------------------------
// RF_NFCEE_DISCOVERY_REQ_NTF
//---------------------------------------------
static int32_t parse_func_nci_r100_tbl67(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"NFCEE ID              ");
    CXD224X_PRINT_1BF(indent, p,len,"RF Technology and Mode", get_nci_r100_tbl96);
    CXD224X_PRINT_1BF(indent, p,len,"RF Protocol           ", get_nci_r100_tbl98);
    return 0;
}

//--------------------------
// Table 66
//--------------------------
static const CXD224X_NCI_ID_NAME_REC cxd224x_nci_r100_tbl66[] = {
    {"ADD",                 0x00, 0x00, &parse_func_nci_r100_tbl67},
    {"REMOVE",              0x01, 0x01, &parse_func_nci_r100_tbl67},
    {"RFU",                 0x02, 0x7F, NULL},
    {"For_proprietary_use", 0x80, 0xFF, NULL},
};

static int32_t parse_rf_nfcee_discovery_req_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&cnt);
    cxd224x_debug(indent, "Information Entry >>>");
    print_tlv_by_cnt(indent+1, p, len, cnt, cxd224x_nci_r100_tbl66, CXD224X_CNT(cxd224x_nci_r100_tbl66));

    return 0;
}

//---------------------------------------------
// RF_PARAMETER_UPDATE
//---------------------------------------------
static int32_t parse_func_nci_r100_tbl72_00(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p,len,"", get_nci_r100_tbl96);
    return 0;
}

static int32_t parse_func_nci_r100_tbl72_01_02(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BF(indent, p,len,"", get_nci_r100_tbl97);
    return 0;
}

static int32_t parse_func_nci_r100_tbl72_03(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t data;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&data);

    // Table73
    cxd224x_debug(indent, "Minimum TR0       =0x%02X", (data>>6)&0x03);
    cxd224x_debug(indent, "Minimum TR1       =0x%02X", (data>>4)&0x03);
    cxd224x_debug(indent, "Suppression of EoS=0x%02X", (data>>3)&0x01);
    cxd224x_debug(indent, "Suppression of SoS=0x%02X", (data>>2)&0x01);
    cxd224x_debug(indent, "Minimum TR2       =0x%02X", data&0x03);

    return 0;
}

//--------------------------
// Table 72
//--------------------------
static const CXD224X_NCI_ID_NAME_REC cxd224x_nci_r100_tbl72[] = {
    {"RF Technology and Mode",              0x00, 0x00, &parse_func_nci_r100_tbl72_00},
    {"Transmit Bit Rate",                   0x01, 0x01, &parse_func_nci_r100_tbl72_01_02},
    {"Receive  Bit Rate",                   0x02, 0x02, &parse_func_nci_r100_tbl72_01_02},
    {"NFC-B Data Exchange Configuration",   0x03, 0x03, &parse_func_nci_r100_tbl72_03},
    {"RFU",                                 0x04, 0x7F, NULL},
    {"Proprietary",                         0x80, 0xFF, NULL},
};

static int32_t parse_rf_parameter_update_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_GET_HEX(p,len,1,&cnt);
    cxd224x_debug(indent, "RF Communication Parameter  >>>");
    print_tlv_by_cnt(indent+1, p, len, cnt, 
        cxd224x_nci_r100_tbl72, CXD224X_CNT(cxd224x_nci_r100_tbl72));

    return 0;
}

static int32_t parse_rf_parameter_update_rsp(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;

    CXD224X_CHECK_NULL(p,-1)
    CXD224X_SKIP(p,len,1);
    CXD224X_GET_HEX(p,len,1,&cnt);
    cxd224x_debug(indent, "RF Communication Parameter ID >>>");
    print_tlv_tag(indent+1, p, cnt, 
        cxd224x_nci_r100_tbl72, CXD224X_CNT(cxd224x_nci_r100_tbl72));
    return 0;
}

//**********************************************************
// NFCEE
//**********************************************************

//---------------------------------------------
// NFCEE_DISCOVER
//---------------------------------------------
static const char* get_nfcee_discovery_action(uint8_t id)
{
    if(id == 0x00){
        return "Disable discovery of NFCEE";
    }else if(id == 0x01){
        return "Enable discovery of NFCEE";
    }else{
        return "RFU";
    }
}

static int32_t parse_nfcee_discover_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
    if(p==NULL){
        return -1;
    }
    CXD224X_PRINT_1BF(indent, p, len, "Discovery Action", get_nfcee_discovery_action);
    return 0;
}

static int32_t parse_nfcee_discover_rsp(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t status;

    if(p==NULL){
        return -1;
    }
    CXD224X_GET_HEX(p,len,1,&status);
    CXD224X_PRINT_1BN(indent, p,len,"Number of NFCEEs")
    return 0;
}

static const char* get_nfcee_status(uint8_t id)
{
    if(id==0x00){
        return "connected and enabled";
    }else if(id==0x01){
        return "connected and disabled";
    }else if(id==0x02){
        return "removed";
    }else{
        return "RFU";
    }
}

static int32_t parse_func_nci_r100_tbl87(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt;
int32_t i;
int32_t nn;

    CXD224X_CHECK_NULL(p,-1)

    nn=8;
    if(len < nn){
        return -1;
    }
    cxd224x_dump2(indent+1, "PMm", p, nn);
    p += nn;
    len -= nn;

    CXD224X_GET_HEX(p,len,1,&cnt);
    for(i=0;i<cnt;i++){
        nn = 2;
        if(len < nn){
            return -1;
        }
        cxd224x_dump2(indent+1, "System Code", p, nn);
        p += nn;
        len -= nn;

        nn = 8;
        if(len < nn){
            return -1;
        }
        cxd224x_dump2(indent+1, "Idm", p, nn);
        p += nn;
        len -= nn;
    }
    return 0;
}

//--------------------------
// Table 86
//--------------------------
static const CXD224X_NCI_ID_NAME_REC cxd224x_nci_r100_tbl86[] = {
    {"Hardware/Registration Identification",                    0x00, 0x00, NULL},
    {"ATR Bytes",                                               0x01, 0x01, NULL},
    {"T3T Command Set Interface Supplementary Information",     0x02, 0x02, &parse_func_nci_r100_tbl87},
    {"RFU",                                                     0x03, 0x9F, NULL},
    {"For proprietary use",                                     0xA0, 0xFF, NULL},
};

static int32_t parse_nfcee_discover_ntf(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t cnt_prot;
uint8_t tlv_cnt;
int32_t i;

    CXD224X_CHECK_NULL(p,-1)

    CXD224X_PRINT_1BN(indent, p,len,"NFCEE ID")
    CXD224X_PRINT_1BF(indent, p,len,"NFCEE Status", get_nfcee_status);

    CXD224X_GET_HEX(p,len,1,&cnt_prot);
    cxd224x_debug(indent, "Number of Protocol Information Entries=0x%02X", cnt_prot);
    for(i=0;i<cnt_prot;i++){
        CXD224X_PRINT_1BF(indent+1, p,len,"+Supported NFCEE Protocols", get_nci_r100_tbl100);
    }

    CXD224X_GET_HEX(p,len,1,&tlv_cnt);
    cxd224x_debug(indent, "Number of NFCEE Information TLVs=0x%02X", tlv_cnt);
    cxd224x_debug(indent, "NFCEE Information TLV >>>", tlv_cnt);
    print_tlv_by_cnt(indent+1, p, len, tlv_cnt, 
        cxd224x_nci_r100_tbl86, CXD224X_CNT(cxd224x_nci_r100_tbl86));
    return 0;
}

//---------------------------------------------
// NFCEE_MODE_SET
//---------------------------------------------
static const char* get_nfcee_mode(uint8_t id)
{
    if(id==0x00){
        return "Disable the connected NFCEE";
    }else if(id==0x01){
        return "Enable the connected NFCEE";
    }else{
        return "RFU";
    }
}
static int32_t parse_nfcee_mode_set_cmd(uint32_t indent, const uint8_t *p, int32_t len)
{
    CXD224X_CHECK_NULL(p,-1)
    CXD224X_PRINT_1BN(indent, p,len,"NFCEE ID")
    CXD224X_PRINT_1BF(indent, p,len,"NFCEE Mode", get_nfcee_mode);
    return 0;
}

//---------------------------------------------
// TLV=Tag Len Value
//---------------------------------------------
static int32_t print_tlv_by_len(uint32_t indent, const uint8_t *p, uint32_t len, 
    const CXD224X_NCI_ID_NAME_REC *p_rec, uint32_t rec_cnt)
{
    CXD224X_CHECK_NULL(p,-1)

    while(1){
        uint8_t want=2;
        uint8_t tag;
        uint8_t data_len;
        const char *tag_name;
        int32_t index;
        
        if(len < want){
            break;
        }
        tag = p[0];
        data_len = p[1];
        p += want;
        len -= want;

        if(len < data_len){
            break;
        }

        index=get_index(tag, p_rec, rec_cnt);
        if(index >= 0){
            tag_name=p_rec[index].m_pName;
            cxd224x_debug(indent, "+Tag=0x%02X(%s) Len=0x%02X", tag, tag_name, data_len);
            cxd224x_dump2(indent, "|Value", p, data_len);
            if(p_rec[index].m_func != NULL){
                (*p_rec[index].m_func)(indent+1, p, len);
            }
        }else{
            tag_name="XXX";
            cxd224x_debug(indent, "+Tag=0x%02X(%s) Len=0x%02X", tag, tag_name, data_len);
            cxd224x_dump2(indent, "|Value", p, data_len);
        }
        p += data_len;
        len -= data_len;
    }
    return 0;
}

static int32_t print_tlv_by_cnt(uint32_t indent, const uint8_t *p, uint32_t len,
    uint32_t tag_cnt, 
    const CXD224X_NCI_ID_NAME_REC *p_rec, uint32_t rec_cnt)
{
    CXD224X_CHECK_NULL(p,-1)

    while(1){
        uint8_t want=2;
        uint8_t tag;
        uint8_t data_len;
        const char *tag_name;
        int32_t index;
        
        if(len < want){
            break;
        }
        tag = p[0];
        data_len = p[1];
        p += want;
        len -= want;

        if(len < data_len){
            break;
        }

        index=get_index(tag, p_rec, rec_cnt);
        if(index >= 0){
            tag_name=p_rec[index].m_pName;
            cxd224x_debug(indent, "+Tag=0x%02X(%s) Len=0x%02X", tag, tag_name, data_len);
            cxd224x_dump2(indent, "|Value", p, data_len);
            if(p_rec[index].m_func != NULL){
                (*p_rec[index].m_func)(indent+1, p, data_len);
            }
        }else{
            tag_name="XXX";
            cxd224x_debug(indent, "+Tag=0x%02X(%s) Len=0x%02X", tag, tag_name, data_len);
            cxd224x_dump2(indent, "|Value", p, data_len);
        }
        p += data_len;
        len -= data_len;
    }
    return 0;
}

static void print_tlv_tag(uint32_t indent, const uint8_t *p, uint32_t len, 
    const CXD224X_NCI_ID_NAME_REC *p_rec, uint32_t rec_cnt)
{
uint32_t i;

    if(p == NULL || len == 0){
        return;
    }

    for(i=0;i<len;i++){
        const char *tag_name;
        tag_name=get_name(p[i], p_rec, rec_cnt);
        cxd224x_debug(indent, "+TAG=0x%02X:%s", p[i], tag_name);
    }
}

//-----------------------------------------------------------
// utility API
//----------------------------------------------------------
static const char* get_name(uint8_t id, const CXD224X_NCI_ID_NAME_REC *p, uint32_t cnt)
{
uint32_t i;

    if(p == NULL || cnt == 0){
        return "";
    }
    for(i=0;i<cnt;i++){
        if(id >= p[i].m_start && id <= p[i].m_end){
            return p[i].m_pName;
        }
    }
    return "XXX";
}

static int32_t get_index(uint8_t id, const CXD224X_NCI_ID_NAME_REC *p, uint32_t cnt)
{
uint32_t i;

    if(p == NULL || cnt == 0){
        return -1;
    }
    for(i=0;i<cnt;i++){
        if(id >= p[i].m_start && id <= p[i].m_end){
            return i;
        }
    }
    return -1;
}

//-----------------------------------------------------------
// Tables
//----------------------------------------------------------
static const char* get_nci_r100_tbl7(uint8_t id)
{
    if(id==0x00){
        return "kept";
    }else if(id==0x01){
        return "reset";
    }else{
        return "RFU";
    }
}

static void print_nci_r100_tbl9_obt0(uint32_t indent, uint8_t id)
{
uint8_t n;

    cxd224x_debug(indent, "NFCC Features:OCT0=0x%02X", id);

    n = (id>>1)&0x03;
    if(n == 0){
        cxd224x_debug(indent+1, "b2b1:the DH is the only entity that configures the NFCC");
    }else if(n == 1){
        cxd224x_debug(indent+1, "b2b1:the NFCC can receive configurations from the DH and other NFCEEs");
    }else{
        cxd224x_debug(indent+1, "RFU");
    }
    n = id&0x01;
    if(n == 1){
        cxd224x_debug(indent+1, "b0:Discovery Frequency configuration in RF_DISCOVER_CMD supported");
    }else{
        cxd224x_debug(indent+1, "b0:Discovery Frequency value is ignored");
    }
}

static void print_nci_r100_tbl9_obt1(uint32_t indent, uint8_t id)
{
uint8_t n;

    cxd224x_debug(indent, "NFCC Features:OCT1=0x%02X", id);

    n = (id>>3)&0x01;
    cxd224x_debug(indent+1, "b3:AID based routing=%s", (n?"YES":"NO"));

    n = (id>>2)&0x01;
    cxd224x_debug(indent+1, "b2:Protocol based routing=%s", (n?"YES":"NO"));

    n = (id>>1)&0x01;
    cxd224x_debug(indent+1, "b1:Technology based Routing=%s", (n?"YES":"NO"));

}

static void print_nci_r100_tbl9_obt2(uint32_t indent, uint8_t id)
{
uint8_t n;

    cxd224x_debug(indent, "NFCC Features:OCT2=0x%02X", id);

    n = (id>>1)&0x01;
    cxd224x_debug(indent+1, "b1:Switched Off state=%s", (n?"YES":"NO"));

    n = (id>>0)&0x01;
    cxd224x_debug(indent+1, "b0:Battery Off state=%s", (n?"YES":"NO"));
}

static void print_nci_r100_tbl9_obt3(uint32_t indent, uint8_t id)
{
    cxd224x_debug(indent, "NFCC Features:OCT3=0x%02X", id);
    cxd224x_debug(indent+1, "reserved for proprietary capabilities");
}

static const char* get_nci_r100_tbl12(uint8_t id)
{
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00, RFU)
    CXD224X_DEF1N(0x01, NFCC_Loop_back)
    CXD224X_DEF1N(0x02, Remote_NFC_Endpoint)
    CXD224X_DEF1N(0x03, NFCEE)
    CXD224X_DEF2N(0x04, 0xC0, RFU)
    CXD224X_DEF2N(0xC1, 0xFE, Proprietary)
    CXD224X_DEF1N(0xFF, RFU)
    CXD224X_DEF_END
}

static const char* get_nci_r100_tbl21(uint8_t id)
{
uint8_t n=id&0x01;
    if(n == 1){
        return "Operating field generated by Remote NFC Endpoint";
    }else{
        return "No Operating field generated by Remote NFC Endpoint.";
    }
}

static const char* get_nci_r100_tbl45(uint8_t id)
{
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00, Last_Message)
    CXD224X_DEF1N(0x01, More_Messages_to_follow)
    CXD224X_DEF2N(0x02, 0xFF, RFU)
    CXD224X_DEF_END
}

static const char* get_nci_r100_tbl50(uint8_t id)
{
const char *p;
id &= 0x07;

    switch(id){
    case 0:
        p="BA_OFF:SW_OFF:SW_ON=000";
        break;
    case 1:
        p="BA_OFF:SW_OFF:SW_ON=001";
        break;
    case 2:
        p="BA_OFF:SW_OFF:SW_ON=010";
        break;
    case 3:
        p="BA_OFF:SW_OFF:SW_ON=011";
        break;
    case 4:
        p="BA_OFF:SW_OFF:SW_ON=100";
        break;
    case 5:
        p="BA_OFF:SW_OFF:SW_ON=101";
        break;
    case 6:
        p="BA_OFF:SW_OFF:SW_ON=110";
        break;
    default: //case 7:
        p="BA_OFF:SW_OFF:SW_ON=111";
        break;
    }
    return p;
}

static const char* get_nci_r100_tbl63(uint8_t id)
{
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00, Idle_Mode)
    CXD224X_DEF1N(0x01, Sleep_Mode)
    CXD224X_DEF1N(0x02, Sleep_AF_Mode)
    CXD224X_DEF1N(0x03, Discovery)
    CXD224X_DEF2N(0x04, 0xFF, RFU)
    CXD224X_DEF_END
}

static const char* get_nci_r100_tbl64(uint8_t id)
{
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00, DH_Request)
    CXD224X_DEF1N(0x01, Endpoint_Request)
    CXD224X_DEF1N(0x02, RF_Link_Loss)
    CXD224X_DEF1N(0x03, NFC-B_Bad_AFI)
    CXD224X_DEF2N(0x04, 0xFF, RFU)
    CXD224X_DEF_END
}

static const char* get_nci_r100_tbl69(uint8_t id)
{
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00, ISO_IEC_7816_4_SELECT_command_with_an_AID)
    CXD224X_DEF1N(0x01, RF_Protocol_based_routing_decision)
    CXD224X_DEF1N(0x02, RF_Technology_based_routing_decision)
    CXD224X_DEF2N(0x03, 0x0F, RFU)
    CXD224X_DEF1N(0x10, Application_initiation)
    CXD224X_DEF2N(0x11, 0xFF, RFU)
    CXD224X_DEF_END
}

static const char* get_nci_r100_tbl94(uint8_t id){
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00,STATUS_OK)
    CXD224X_DEF1N(0x01,STATUS_REJECTED)
    CXD224X_DEF1N(0x02,STATUS_RF_FRAME_CORRUPTED)
    CXD224X_DEF1N(0x03,STATUS_FAILED)
    CXD224X_DEF1N(0x04,STATUS_NOT_INITIALIZED)
    CXD224X_DEF1N(0x05,STATUS_SYNTAX_ERROR)
    CXD224X_DEF1N(0x06,STATUS_SEMANTIC_ERROR)
    CXD224X_DEF2N(0x07,0x08,RFU)
    CXD224X_DEF1N(0x09,STATUS_INVALID_PARAM)
    CXD224X_DEF1N(0x0A,STATUS_MESSAGE_SIZE_EXCEEDED)
    CXD224X_DEF2N(0x0B,0x9F,RFU)
// RF Discovery Specific Status Codes
    CXD224X_DEF1N(0xA0,DISCOVERY_ALREADY_STARTED)
    CXD224X_DEF1N(0xA1,DISCOVERY_TARGET_ACTIVATION_FAILED)
    CXD224X_DEF1N(0xA2,DISCOVERY_TEAR_DOWN)
    CXD224X_DEF2N(0xA3,0xAF,RFU)
// RF Interface Specific Status Codes
    CXD224X_DEF1N(0xB0,RF_TRANSMISSION_ERROR)
    CXD224X_DEF1N(0xB1,RF_PROTOCOL_ERROR)
    CXD224X_DEF1N(0xB2,RF_TIMEOUT_ERROR)
    CXD224X_DEF2N(0xB3,0xBF,RFU)
// NFCEE Interface Specific Status Codes
    CXD224X_DEF1N(0xC0,NFCEE_INTERFACE_ACTIVATION_FAILED)
    CXD224X_DEF1N(0xC1,NFCEE_TRANSMISSION_ERROR)
    CXD224X_DEF1N(0xC2,NFCEE_PROTOCOL_ERROR)
    CXD224X_DEF1N(0xC3,NFCEE_TIMEOUT_ERROR)
    CXD224X_DEF2N(0xC4,0xDF,RFU)
// Proprietary Status Codes
// CXD224X proprietary StatusCode 0xE0 <= xxx <= 0xF1
    CXD224X_DEF1N(0xE0,CXD_STS_INVALD_POWER_ERROR)
    CXD224X_DEF1N(0xE1,CXD_STS_INVALD_STATE)
    CXD224X_DEF1N(0xE2,CXD_STS_NFCEE_STATUS_ERROR)
    CXD224X_DEF1N(0xE3,CXD_STS_MULTI_CARD_INVALD_UID_ERROR)
    CXD224X_DEF1N(0xE4,CXD_STS_NO_PROCESSING_TARGET)
    CXD224X_DEF1N(0xE5,CXD_STS_DATA_EXCHANGE_INVALID_SIZE_ERROR)
    CXD224X_DEF1N(0xE6,CXD_STS_EXCHANGE_CREDIT_OVER_ERROR)
    CXD224X_DEF1N(0xE7,CXD_STS_NVM_ECC_FAIL)
    CXD224X_DEF1N(0xE8,CXD_STS_NVM_ECC_USED)
    CXD224X_DEF1N(0xE9,CXD_STS_NVM_HW_FAIL)
    CXD224X_DEF1N(0xEA,CXD_STS_CONN_ID_ERROR)
    CXD224X_DEF1N(0xEB,CXD_STS_FOR_DCLB_HOSTCOMM_STATE)
    CXD224X_DEF1N(0xEC,CXD_STS_HCI_GATE_ID_ERROR)
    CXD224X_DEF1N(0xED,CXD_STS_PROP_MODE_STATE)
    CXD224X_DEF1N(0xEE,CXD_STS_CLKUNDET)
    CXD224X_DEF1N(0xEF,CXD_STS_TX_SIZE_ERROR)
    CXD224X_DEF1N(0xF0,CXD_STS_FW_INTERNAL_ERROR)
    CXD224X_DEF1N(0xF1,CXD_STS_BEINGPROCESS_ERROR)

    CXD224X_DEF2N(0xF2,0xFF,For_proprietary_use)
    CXD224X_DEF_END
}

static const char* get_nci_r100_tbl95(uint8_t id)
{
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00, NFC_RF_TECHNOLOGY_A)
    CXD224X_DEF1N(0x01, NFC_RF_TECHNOLOGY_B)
    CXD224X_DEF1N(0x02, NFC_RF_TECHNOLOGY_F)
    CXD224X_DEF1N(0x03, NFC_RF_TECHNOLOGY_15693)
    CXD224X_DEF2N(0x04, 0x7F, RFU)
    CXD224X_DEF2N(0x80, 0xFE, For_proprietary_use)
    CXD224X_DEF1N(0xFF, RFU)
    CXD224X_DEF_END
}

static const char* get_nci_r100_tbl96(uint8_t id)
{
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00, NFC_A_PASSIVE_POLL_MODE)
    CXD224X_DEF1N(0x01, NFC_B_PASSIVE_POLL_MODE)
    CXD224X_DEF1N(0x02, NFC_F_PASSIVE_POLL_MODE)
    CXD224X_DEF1N(0x03, NFC_A_ACTIVE_POLL_MODE_RFU)
    CXD224X_DEF1N(0x04, RFU)
    CXD224X_DEF1N(0x05, NFC_F_ACTIVE_POLL_MODE_RFU)
    CXD224X_DEF1N(0x06, NFC_15693_PASSIVE_POLL_MODE_RFU)
    CXD224X_DEF2N(0x07, 0x6F, RFU)
    CXD224X_DEF2N(0x70, 0x7F, Reserved_for_Proprietary_Technologies_in_Poll_Mode)
    CXD224X_DEF1N(0x80, NFC_A_PASSIVE_LISTEN_MODE)
    CXD224X_DEF1N(0x81, NFC_B_PASSIVE_LISTEN_MODE)
    CXD224X_DEF1N(0x82, NFC_F_PASSIVE_LISTEN_MODE)
    CXD224X_DEF1N(0x83, NFC_A_ACTIVE_LISTEN_MODE_RFU)
    CXD224X_DEF1N(0x84, RFU)
    CXD224X_DEF1N(0x85, NFC_F_ACTIVE_LISTEN_MODE_RFU)
    CXD224X_DEF1N(0x86, NFC_15693_PASSIVE_LISTEN_MODE_RFU)
    CXD224X_DEF2N(0x87, 0xEF, RFU)
    CXD224X_DEF2N(0xF0, 0xFF, Reserved_for_Proprietary_Technologies_in_Listen_Mode)
    CXD224X_DEF_END
}

static const char* get_nci_r100_tbl97(uint8_t id)
{
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00, NFC_BIT_RATE_106)
    CXD224X_DEF1N(0x01, NFC_BIT_RATE_212)
    CXD224X_DEF1N(0x02, NFC_BIT_RATE_424)
    CXD224X_DEF1N(0x03, NFC_BIT_RATE_848)
    CXD224X_DEF1N(0x04, NFC_BIT_RATE_1695)
    CXD224X_DEF1N(0x05, NFC_BIT_RATE_3390)
    CXD224X_DEF1N(0x06, NFC_BIT_RATE_6780)
    CXD224X_DEF2N(0x07, 0x7F, RFU)
    CXD224X_DEF2N(0x80, 0xFE, For_proprietary_use)
    CXD224X_DEF1N(0xFF, RFU)
    CXD224X_DEF_END
}

static const char* get_nci_r100_tbl98(uint8_t id)
{
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00,PROTOCOL_UNDETERMINED)
    CXD224X_DEF1N(0x01,PROTOCOL_T1T)
    CXD224X_DEF1N(0x02,PROTOCOL_T2T)
    CXD224X_DEF1N(0x03,PROTOCOL_T3T)
    CXD224X_DEF1N(0x04,PROTOCOL_ISO_DEP)
    CXD224X_DEF1N(0x05,PROTOCOL_NFC_DEP)
    CXD224X_DEF2N(0x06,0x7F,RFU)
    CXD224X_DEF2N(0x80,0xFE,For_proprietary_use)
    CXD224X_DEF1N(0xFF,RFU)
    CXD224X_DEF_END
}

static const char* get_nci_r100_tbl99(uint8_t id)
{
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00,NFCEE_Direct_RF_Interface)
    CXD224X_DEF1N(0x01,Frame_RF_Interface)
    CXD224X_DEF1N(0x02,ISO_DEP_RF_Interface)
    CXD224X_DEF1N(0x03,NFC_DEP_RF_Interface)
    CXD224X_DEF2N(0x04,0x7F,RFU)
    CXD224X_DEF2N(0x80,0xFE,For_proprietary_use)
    CXD224X_DEF1N(0xFF,RFU)
    CXD224X_DEF_END
}

static const char* get_nci_r100_tbl100(uint8_t id)
{
    CXD224X_DEF_START
    CXD224X_DEF1N(0x00,APDU)
    CXD224X_DEF1N(0x01,HCI_Access)
    CXD224X_DEF1N(0x02,Type_3_Tag_Command_Set)
    CXD224X_DEF1N(0x03,Transparent)
    CXD224X_DEF2N(0x04,0x7F, RFU)
    CXD224X_DEF2N(0x80,0xFE, For_proprietary_use)
    CXD224X_DEF1N(0xFF,RFU)
    CXD224X_DEF_END
}

static const CXD224X_NCI_ID_NAME_REC cxd224x_llcp_tlv[] = {
    CXD224X_XDEF1(VERSION,      0x01, NULL)
    CXD224X_XDEF1(MIUX,         0x02, NULL)
    CXD224X_XDEF1(WKS,          0x03, NULL)
    CXD224X_XDEF1(LTO,          0x04, NULL)
    CXD224X_XDEF1(RW,           0x05, NULL)
    CXD224X_XDEF1(SN,           0x06, NULL)
    CXD224X_XDEF1(OPT,          0x07, NULL)
    CXD224X_XDEF1(SDREQ,        0x08, NULL)
    CXD224X_XDEF1(SDRES,        0x09, NULL)
};

typedef struct  {
    const char  *p_name;
    uint8_t        type;  // parse type
} CXD224X_LLCP_DEF;

static const CXD224X_LLCP_DEF llcp_def[] = {
    {"SYMM",     0},
    {"PAX",      1},    // 1=parameter
    {"AGF",      0},
    {"UI",       0},
    {"CONNECT",  1},
    {"DISC",     0},
    {"CC",       1},
    {"DM",       0},
    {"FRMR",     0},
    {"SNL",      1},
    {"reserved", 0},
    {"reserved", 0},
    {"I",        2},    // 2=info
    {"RR",       0},
    {"RNR",      0},
    {"reserved", 0},
};

static int32_t parse_llcp(uint32_t indent, const uint8_t *p, int32_t len)
{
uint8_t c0, c1;
uint8_t ptype;

    CXD224X_CHECK_NULL(p,-1)
    if(len < 2){
        return -1;
    }

    c0 = p[0]&0x03;
    c1 = (p[1] >> 6)&0x03;
    ptype = (c0 << 2) | c1;

    ptype &= 0x0F;

    if(ptype != 0){
        cxd224x_debug(indent, "LLCP=%s  DSAP=0x%02X:SSAP=0x%02X", 
            llcp_def[ptype].p_name, (p[0]>>2)&0x3F, p[1]&0x3F);
    }else{
        cxd224x_debug(indent, "LLCP=%s  DSAP=0x%02X:SSAP=0x%02X", 
            llcp_def[ptype].p_name, (p[0]>>2)&0x3F, p[1]&0x3F);
    }

    p += 2;
    len -= 2;
    if(llcp_def[ptype].type == 1){
        print_tlv_by_len(indent+1, p, len, cxd224x_llcp_tlv, CXD224X_CNT(cxd224x_llcp_tlv));
    }else if(llcp_def[ptype].type == 2){
        if(len >= 3){
            const char *snep_str;
            switch(p[2]){
                case 0x00:
                    snep_str="Continue";
                    break;
                case 0x01:
                    snep_str="Get";
                    break;
                case 0x02:
                    snep_str="Put";
                    break;
                case 0x7F:
                    snep_str="Reject";
                    break;
                case 0x80:
                    snep_str="Continue";
                    break;
                case 0x81:
                    snep_str="Success";
                    break;
                case 0xC0:
                    snep_str="Not Found";
                    break;
                case 0xC1:
                    snep_str="Excess Data";
                    break;
                case 0xC2:
                    snep_str="Bad Request";
                    break;
                case 0xE0:
                    snep_str="Not Implemented";
                    break;
                case 0xE1:
                    snep_str="Unsupported Version";
                    break;
                case 0xFF:
                    snep_str="Reject";
                    break;
                default:
                    snep_str="";
                    break;
            }
            if(strlen(snep_str) > 0){
                cxd224x_debug(indent+1, "SNEP=%s", snep_str); 
            }
        }
    }
    return 0;
}

typedef struct  {
    const char  *p_name;
    uint32_t     type;
    uint32_t     cmd;
    uint32_t     rsp;
} HCI_COMMAND;

#define HCI_COMMAND_DEF( name, cmd, rsp )  \
    { #name, NFA_HCI_## name, cmd, rsp }

enum {
    DATAP_TYPE_NONE=0,
    /* HCI */
    DATAP_TYPE_HCI_PTYPE_RSP,
    DATAP_TYPE_HCI_PTYPE_INDEX,
    DATAP_TYPE_HCI_PTYPE_NUM,
    DATAP_TYPE_HCI_PTYPE_PIPE_CCMD,
    DATAP_TYPE_HCI_PTYPE_PIPE_RSP,
    DATAP_TYPE_HCI_PTYPE_PIPE_NCMD,
    DATAP_TYPE_HCI_PID,
    DATAP_TYPE_HCI_REF_DATA,
    DATAP_TYPE_HCI_HID,
    
    /*T4T*/
    DATAP_TYPE_UNKNOWN
};
typedef int32_t tDATAP_TYPE;

static const HCI_COMMAND hci_cmds[] =
{
    HCI_COMMAND_DEF(ANY_SET_PARAMETER,          DATAP_TYPE_HCI_PTYPE_INDEX,     DATAP_TYPE_HCI_PTYPE_RSP),
    HCI_COMMAND_DEF(ANY_GET_PARAMETER,          DATAP_TYPE_HCI_PTYPE_INDEX,     DATAP_TYPE_HCI_PTYPE_RSP),
    HCI_COMMAND_DEF(ANY_OPEN_PIPE,              DATAP_TYPE_HCI_PTYPE_NUM,       DATAP_TYPE_HCI_PTYPE_RSP),
    HCI_COMMAND_DEF(ANY_CLOSE_PIPE,             DATAP_TYPE_NONE,                DATAP_TYPE_HCI_PTYPE_RSP),
    HCI_COMMAND_DEF(ADM_CREATE_PIPE,            DATAP_TYPE_HCI_PTYPE_PIPE_CCMD, DATAP_TYPE_HCI_PTYPE_PIPE_RSP),
    HCI_COMMAND_DEF(ADM_DELETE_PIPE,            DATAP_TYPE_HCI_PID,             DATAP_TYPE_HCI_PTYPE_RSP),
    HCI_COMMAND_DEF(ADM_NOTIFY_PIPE_CREATED,    DATAP_TYPE_HCI_PTYPE_PIPE_NCMD, DATAP_TYPE_HCI_PTYPE_RSP),
    HCI_COMMAND_DEF(ADM_NOTIFY_PIPE_DELETED,    DATAP_TYPE_HCI_PID,             DATAP_TYPE_HCI_PTYPE_RSP),
    HCI_COMMAND_DEF(ADM_CLEAR_ALL_PIPE,         DATAP_TYPE_HCI_HID,             DATAP_TYPE_HCI_PTYPE_RSP),
    HCI_COMMAND_DEF(ADM_NOTIFY_ALL_PIPE_CLEARED,DATAP_TYPE_HCI_HID,             DATAP_TYPE_HCI_PTYPE_RSP),
};

const char *hci_rsp_str( uint8_t rsp_code )
{
    switch( rsp_code ){
    case NFA_HCI_ANY_OK :
        return "ANY_OK";
    case NFA_HCI_ANY_E_NOT_CONNECTED         :
        return "ANY_E_NOT_CONNECTED";
    case NFA_HCI_ANY_E_CMD_PAR_UNKNOWN       :
        return "ANY_E_CMD_PAR_UNKNOWN";
    case NFA_HCI_ANY_E_NOK                   :
        return "ANY_E_NOK";
    case NFA_HCI_ADM_E_NO_PIPES_AVAILABLE    :
        return "ADM_E_NO_PIPES_AVAILABLE";
    case NFA_HCI_ANY_E_REG_PAR_UNKNOWN       :
        return "ANY_E_REG_PAR_UNKNOWN";
    case NFA_HCI_ANY_E_PIPE_NOT_OPENED       :
        return "ANY_E_PIPE_NOT_OPENED";
    case NFA_HCI_ANY_E_CMD_NOT_SUPPORTED     :
        return "ANY_E_CMD_NOT_SUPPORTED";
    case NFA_HCI_ANY_E_INHIBITED             :
        return "ANY_E_INHIBITED";
    case NFA_HCI_ANY_E_TIMEOUT               :
        return "ANY_E_TIMEOUT";
    case NFA_HCI_ANY_E_REG_ACCESS_DENIED     :
        return "ANY_E_REG_ACCESS_DENIED";
    case NFA_HCI_ANY_E_PIPE_ACCESS_DENIED    :
        return "ANY_E_PIPE_ACCESS_DENIED";
    }
    return "RFU";
}
const char *hci_host_str( uint8_t host_id )
{
    switch( host_id ){
    case NFA_HCI_HOST_CONTROLLER:
        return "HOST_CONTROLLER";
    case NFA_HCI_DH_HOST:
        return "DH_HOST";
    case NFA_HCI_UICC_HOST:
        return "UICC_HOST";
    default:
        ;
    }
    return (host_id < 0xBF) ? "RFU" : "Proprietary";
}
const char *hci_pid_str( uint8_t pid )
{
    switch( pid ){
    case NFA_HCI_LINK_MANAGEMENT_PIPE:
        return "LINK_MANAGEMENT_PIPE";
    case NFA_HCI_ADMIN_PIPE:
        return "ADMIN_PIPE";
    }
    return (pid < 0x6F) ? "Dynamic_GATE": "RFU";
}
const char *hci_admingate_parm_str( uint8_t idx )
{
    switch( idx ){
    case NFA_HCI_SESSION_IDENTITY_INDEX:
        return "SESSION_IDENTITY";
    case NFA_HCI_MAX_PIPE_INDEX:
        return "MAX_PIPE";
    case NFA_HCI_WHITELIST_INDEX:
        return "WHITELIST";
    case NFA_HCI_HOST_LIST_INDEX:
        return "HOST_LIST";
    default:
        ;
    }
    return "";
}


#define MAX_BUFLEN 300
static size_t get_hci_commandstrings( const uint8_t *p, uint8_t inst, uint8_t pid, uint8_t type, char *dst_buf, size_t dst_len)
{
    uint32_t i;
    const HCI_COMMAND *cmdp = &hci_cmds[0];
    char buf[MAX_BUFLEN]="";
    uint8_t instv=0, rspv=0xff;

    switch( type ){
    case 0: /* Cmd */
        instv = inst;
        break;
    case 1:  /* Event */
        instv = 0;
        break;
    case 2:  /* Resp */
        instv = g_last_cmd;
        rspv = inst;
        break;
    }
    for( i = 0; i < sizeof(hci_cmds)/sizeof(HCI_COMMAND); i++ )
    {
        if( instv == cmdp->type )
        {
            break;
        }
        cmdp++;
    }

    if( instv != cmdp->type )
    {
        return 0;
    }

    switch( type ){
    case 0:
        g_last_cmd = cmdp->type;
        switch(cmdp->cmd){
        case DATAP_TYPE_HCI_PTYPE_INDEX:
            if( pid == NFA_HCI_ADMIN_PIPE )
            {
                snprintf( buf, MAX_BUFLEN, "%s: idx=%s(%d)", cmdp->p_name, hci_admingate_parm_str(*p), *p );
                p++;
            }
            else
                snprintf( buf, MAX_BUFLEN, "%s: idx=%d", cmdp->p_name, *p++);
            break;
        case DATAP_TYPE_HCI_PTYPE_NUM:
            snprintf( buf, MAX_BUFLEN, "%s: num=%d", cmdp->p_name, *p++ );
            break;
        case DATAP_TYPE_HCI_PTYPE_PIPE_CCMD:
            snprintf( buf, MAX_BUFLEN, "%s: srcGid=0x%x dstHid=0x%x dstGid=0x%x", cmdp->p_name, p[0], p[1], p[2]);
            p+=3;
            break;
        case DATAP_TYPE_HCI_PTYPE_PIPE_NCMD:
            snprintf( buf, MAX_BUFLEN, "%s: srcHid=0x%x srcGid=0x%x dstHid=0x%x dstGid=0x%x Pid=0x%x", 
                      cmdp->p_name, p[0], p[1], p[2], p[3], p[4]);
            p+=5;
            break;
        case DATAP_TYPE_HCI_PID:
            snprintf( buf, MAX_BUFLEN, "%s: Pid=0x%x", cmdp->p_name, *p++ );
            break;
        case DATAP_TYPE_HCI_HID:
            snprintf( buf, MAX_BUFLEN, "%s: Hid=%s(0x%x)", cmdp->p_name, hci_host_str(*p), *p );
            p++;
            break;
        default:
            g_last_cmd = 0;
        }
        break;
    case 2:
        switch(cmdp->rsp){
        case DATAP_TYPE_HCI_PTYPE_RSP:
            snprintf( buf, MAX_BUFLEN, "%s: %s", cmdp->p_name, hci_rsp_str(rspv));
            break;
        case DATAP_TYPE_HCI_PTYPE_PIPE_RSP:
            snprintf( buf, MAX_BUFLEN, "%s: %ssrcHid=0x%x srcGid=0x%x dstHid=0x%x dstGid=0x%x Pid=0x%x", cmdp->p_name, 
                      hci_rsp_str(rspv),  p[0], p[1], p[2], p[3], p[4]);
            p+=5;
            break;
        }
        g_last_cmd = 0;
    default:
        ;
    }
    size_t b_len = strlen(buf);
    b_len =  b_len < dst_len ? b_len : dst_len;

    strncpy( dst_buf, buf, b_len );
    return b_len;
}


static int32_t parse_hci(uint32_t indent, const uint8_t *p, int32_t len)
{
    uint8_t cid, plen, cb, pid;
    uint8_t inst, type;
    char buf[MAX_BUFLEN]="";
    const char *tstr;

    if(len < 2){
        return -1;
    }
    cb = (*p) >> 6;
    pid = *p++ & 0x3f;
    
    type = (*p) >> 6;
    inst = *p++ & 0x3f;
    len--;

    switch( type ){
    case 0:
        tstr = "CMD";
        break;
    case 1:
        tstr = "EVT";
        break;
    case 2:
        tstr = "RSP";
        break;
    default:
        tstr = "ukn";
    }

    get_hci_commandstrings( p, inst, pid, type, buf, MAX_BUFLEN );
    cxd224x_debug(indent, "{HCP_%s: Pid=%s(%d) %s}", 
                  tstr, hci_pid_str(pid), pid, buf );
    return 0;
}
static int check_cla( uint8_t cla )
{
    switch( (cla >> 4) & 0xf ){
    case 0:
    case 8:
    case 9:
    case 0xA:
    case 0xb:
    case 0xc:
    case 0xd:
    case 0xe:
    case 0xf:
        return 0;
    }
    return 1;
}

/*
  http://www.cardwerk.com/smartcards/smartcard_standard_ISO7816-4_5_basic_organizations.aspx#table8
  Table 8 - Coding and meaning of CLA
  Table 9 - Coding and meaning of nibble 'X' when CLA='0X','8X','9X' or 'AX'
*/
static char *get_isodep_cla_str( uint8_t cla )
{
    static char cla_buf[20];
    char sec, ap, lc;
    switch( (cla >> 4) & 0xf ){
    case 0:
    case 8:
    case 9:
    case 0xA:
        if( cla & (1<<3) ){
            sec = 'S';
            if( cla & (1<<2) ){
                ap = 'A';
            }else{
                ap = 'a';
            }
        }else{
            sec = 's';
            if( cla & (1<<2) ){
                ap = 'P';
            }else{
                ap = 'p';
            }
        }
        lc = '0' + (cla & 0x3);
        break;
    case 0xb:
    case 0xc:
        sec ='c';
        ap = 'p';
        lc = '-';
        break;
    case 0xd:
    case 0xe:
    case 0xf:
        sec ='c';
        ap = 'P';
        lc = '-';
        break;
    default:
        sec = 'R';
        ap = 'F';
        lc = 'U';
        break;
    }
    sprintf( cla_buf, "%02x:%c%c%c", cla, sec, ap, lc );
    return cla_buf;
}
/* http://www.cardwerk.com/smartcards/smartcard_standard_ISO7816-4_5_basic_organizations.aspx#table11
 * Table 11 - INS codes defined in this part of ISO/IEC 7816
 */
static char *get_isodep_ins_str(uint8_t ins)
{
    const char *icmd;
    static char ins_buf[32];
    if( (ins & 0x1) || (ins & 0xf0)==0x60 || (ins & 0xf0)==0x90 )
    {
        sprintf(ins_buf, "%02x:Invalid INS", ins);
        return ins_buf;
    }
    switch( ins ){
    case 0x0e: icmd="ERASE BINARY"; break;
    case 0x20: icmd="VERIFY"; break;
    case 0x70: icmd="MANAGE CHANNEL"; break;
    case 0x82: icmd="EXTERNAL AUTHENTICATE"; break;
    case 0x84: icmd="GET CHALLENGE"; break;
    case 0x88: icmd="INTERNAL AUTHENTICATE"; break;
    case 0xA4: icmd="SELECT FILE"; break;
    case 0xB0: icmd="READ BINARY"; break;
    case 0xB2: icmd="READ RECORD(S)"; break;
    case 0xC0: icmd="GET RESPONSE"; break;
    case 0xC2: icmd="ENVELOPE"; break;
    case 0xCA: icmd="GET DATA"; break;
    case 0xD0: icmd="WRITE BINARY"; break;
    case 0xD2: icmd="WRITE RECORD"; break;
    case 0xD6: icmd="UPDATE BINARY"; break;
    case 0xDA: icmd="PUT DATA"; break;
    case 0xDC: icmd="UPDATE DATA"; break;
    case 0xE2: icmd="APPEND RECORD"; break; 
    default: icmd="unkonw";
    }
    sprintf(ins_buf, "%02x:%s", ins, icmd);
    return ins_buf;
}
/*
 * Table 12 - Coding of SW1-SW2
 */
static char *get_isodep_sw_str(uint8_t sw1, uint8_t sw2)
{
    static char sw_buf[200];
    const char *s1="unknown";
    char s2[100]="";
    switch( sw1 ){
    case 0x90:
        if( sw2 == 0x0 ){
            s1 = "SUCCESS";
        }
        break;
    case 0x61: 	s1 = "SW2 indicates the number of response bytes still available"; 
        sprintf( s2, "%02x", sw2);
        break;
/*	Warning processings */
    case 0x62: 	s1 = "W:State of non-volatile memory unchanged";
        switch( sw2 ){
        case 0x81: sprintf( s2, "Part of returned data may be corrupted"); break;
        case 0x82: sprintf( s2, "End of file/record reached before reading Le bytes"); break;
        case 0x83: sprintf( s2, "Selected file invalidated"); break;
        case 0x84: sprintf( s2, "FCI not formatted according to 1.1.5"); break;
        }
        break;
    case 0x63: 	s1 = "W:State of non-volatile memory changed";
        if( sw2 == 0x81 ){
            sprintf( s2, "File filled up by the last write");
        }else if( (sw2 & 0xf0) == 0xc0 ){
            sprintf( s2, "Counter provided by %d", sw2 & 0xf);
        }
        break;
/*	Execution errors */ 
    case 0x64: 	s1 = "E:State of non-volatile memory unchanged"; break;
    case 0x65: 	s1 = "E:State of non-volatile memory changed";
        if( sw2 == 0x81 ){
            sprintf( s2, "Memory failure");
        }
        break;
    case 0x66: 	s1 = "E:Reserved for security-related issues"; break;
        
/*	Checking errors */
    case 0x67: 	s1 = "E:Wrong length"; break;
    case 0x68: 	s1 = "E:Functions in CLA not supported"; 
        if( sw2 == 0x81 ){
            sprintf( s2, "Logical channel not supported");
        }else if( sw2 == 0x82 ){
            sprintf( s2, "Secure messaging not supported");
        }
        break;
    case 0x69: 	s1 = "E:Command not allowed";
        switch( sw2 ){
        }
        break;
    case 0x6A:        
        switch( sw2 ){
        case 0x81: 	sprintf( s2, "Command incompatible with file structure"); break;
        case 0x82: 	sprintf( s2, "Security status not satisfied"); break;
        case 0x83: 	sprintf( s2, "Authentication method blocked"); break;
        case 0x84: 	sprintf( s2, "Referenced data invalidated"); break;
        case 0x85: 	sprintf( s2, "Conditions of use not satisfied"); break;
        case 0x86: 	sprintf( s2, "Command not allowed (no current EF)"); break;
        case 0x87: 	sprintf( s2, "Expected SM data objects missing"); break;
        case 0x88: 	sprintf( s2, "SM data objects incorrect "); break;
        }
    case 0x6B: 	s1 = "E:Wrong parameter(s) P1-P2"; break;
    case 0x6C: 	s1 = "E:Wrong length Le: SW2 indicates the exact length";break;
    case 0x6D: 	s1 = "E:Instruction code not supported or invalid";break;
    case 0x6E: 	s1 = "E:Class not supported";break;
    case 0x6F:  s1 = "E:No precise diagnosis";break;
    default:
        ;
    }
    sprintf(sw_buf,"%02x:%02x %s:%s", sw1,sw2, s1,s2);
    return sw_buf;
}

struct ISO7816_STRUCT {
  uint8_t cla;
  uint8_t ins;
  uint8_t p1;
  uint8_t p2;
  uint8_t *body;
  uint32_t bodyBytes;
};
static int32_t parse_print_select_aid(uint32_t indent, const struct ISO7816_STRUCT iso7816Struct)
{
  uint8_t *pChar = (uint8_t *)iso7816Struct.body;
  uint32_t nc = 0;
  uint8_t shortOrExtended;
  enum uint8_t {SOE_short, SOE_ext};
  char aidStrings[255*3] = {0}; // 3 is jump in strings for print
  uint32_t i;

  if(!(iso7816Struct.ins == 0xA4 && iso7816Struct.p1 == 0x04)) return -1;
  if(iso7816Struct.bodyBytes <= 0) return -1;
  if(*pChar == 0){
    nc = (*(pChar+1) << 8) + (*(pChar+2));
    pChar+=2;
  }else{
    nc = *pChar++;
  }
  for(i=0; i < nc; i++){
//    cxd224x_debug(indent,"aid[%d]=%02X",i,*pChar);
    sprintf(&aidStrings[i*3], "%02X ", *pChar);
    pChar++;
  }
  aidStrings[i*3] = '\0';
  cxd224x_debug(indent, "SELECT_AID command: AID(%d bytes)=%s", nc, aidStrings);
  return 0;
}
#define ISO_IEC_7816_4_LEN_BYTES 4
static int32_t parse_isodep(uint32_t indent, const uint8_t *p, int32_t len)
{
    struct ISO7816_STRUCT iso7816Struct;

    if(len < ISO_IEC_7816_4_LEN_BYTES) return -1;
    if( !g_last_cmd || g_last_dir == g_cxd224x.data_write_flag){
        iso7816Struct.cla = *p++;
        iso7816Struct.ins = *p++;
        iso7816Struct.p1 = *p++;
        iso7816Struct.p2 = *p++;
        iso7816Struct.body = (uint8_t *)p;
        iso7816Struct.bodyBytes = len-ISO_IEC_7816_4_LEN_BYTES;
        if( !check_cla( iso7816Struct.cla ) ){
            g_last_cmd = iso7816Struct.ins;
            cxd224x_debug(indent, "ISODEP_CMD: CLA='%s' INS='%s' p='%02x:%02x'",
                          get_isodep_cla_str( iso7816Struct.cla ),
                          get_isodep_ins_str( iso7816Struct.ins ), iso7816Struct.p1, iso7816Struct.p2
            );
            parse_print_select_aid(indent+2, iso7816Struct);
        }else{
            g_last_cmd = -1;
            cxd224x_debug(indent, "ISODEP_UNKNOWN: %02x:%02x:%02x:%02x",
                          iso7816Struct.cla, iso7816Struct.ins, iso7816Struct.p1, iso7816Struct.p2 );
        }
    }else if( g_last_cmd == -1 ){
        g_last_cmd = 0;
        cxd224x_debug(indent, "ISODEP_UNKNOWN");
    }else{
        cxd224x_debug(indent, "ISODEP_RSP: INS='%s' sw='%s'",
                      get_isodep_ins_str( g_last_cmd ),
                      get_isodep_sw_str( p[len-2], p[len-1] )
            );
        g_last_cmd = 0;
    }
    g_last_dir = g_cxd224x.data_write_flag;
    return 0;
}
static int32_t parse_t2t(uint32_t indent, const uint8_t *p, int32_t len)
{
    const char *s1 = "Ukn";
    char s2[100] = "";

    if( g_last_cmd > 0xff && g_last_dir != g_cxd224x.data_write_flag){
        switch( g_last_cmd & 0xff )
        {
        case 0xA0:
            s1 = "COMPATIBILITY_WRITE_P2";
            sprintf(s2, "%02x:%02x:%02x:%02x", p[0], p[1], p[2], p[3]);
            break;
        case 0xC2:
            s1 = "SECTOR_SELECT_P2";
            sprintf(s2, "SecNo=%d", *p);
            break;
        default:
            s1 = "Unknown";
        }
        cxd224x_debug(indent, "T2T_CMD2: %s %s",
                      s1, s2
            );
        g_last_cmd &= 0xff;

    }else if( !g_last_cmd || g_last_dir == g_cxd224x.data_write_flag){
        uint8_t cmd = *p++;
        uint8_t p1,p2;
        g_return_len=0;
        switch( cmd )
        {
        case 0x30:
            g_last_cmd = cmd;
            p1 = *p++;
            s1 = "READ";
            g_return_len=16;
            sprintf(s2, "BLK=%d", p1);
            break;
        case 0x3a:
            g_last_cmd = cmd;
            p1 = *p++;
            p2 = *p++;
            s1 = "FIRST_READ";
            g_return_len=(p2>p1)? (p2-p1)*4 : 0;
            sprintf(s2, "start=%d end=%d", p1, p2 );
            break;
        case 0xA2:
            g_last_cmd = cmd;
            s1 = "WRITE";
            p1 = *p++;
            sprintf(s2, "BLK=%d D=%02x:%02x:%02x:%02x", p1, 
                    p[0], p[1], p[2], p[3]);
            break;
        case 0xA0:
            g_last_cmd = -cmd;
            s1 = "COMPATIBILITY_WRITE_P1";
            sprintf(s2, "BLK=%d", p[0] );
            break;
        case 0x1B:
            g_last_cmd = cmd;
            s1 = "PWD_AUTH";
            g_return_len=2;
            break;
        case 0x3c:
            g_last_cmd = cmd;
            s1 = "READ_SIG";
            p1 = *p++;
            g_return_len=32;
            sprintf(s2, "BLK=%d", p1);
            break;
        case 0xC2:
            g_last_cmd = -cmd;
            if( *p == 0xff ){
                s1 = "SECTOR_SELECT_P1";
            }else{
                s1 = "SECTOR_SELECT??_P1";
            }
            break;
        case 0x60:
            g_last_cmd = cmd;
            s1 = "GET_VERSION";
            break;
        default:
            s1 = "Unknown";
        }
        cxd224x_debug(indent, "T2T_CMD: %s %s",
                      s1, s2
            );
    }else{
        uint8_t stat = p[len-1];
        if( stat != 0 ){
            if( stat == 0x2 )
                s1 = "STATUS_RF_FRAME_CORRUPTED";
            else
                s1 = "STATUS_FAILED";
        }else if( len == g_return_len + 1 ){
            s1 = "STATUS_OK";
        }else{
            uint8_t rsp = *p;
            switch( rsp & 0xf ){
            case 0xa:
                s1 = "Ack";
                break;
            case 0:
                s1 = "Nak(invalid argument/any other errors)";
                break;
            case 1:
                s1 = "Nak(parity or CRC error)";
                break;
            case 5:
                s1 = "Nak(EEPROM write error)";
                break;
            case 4:
                s1 = "Nak(Unknown)";
                break;
            default:
                s1 = "Nak?Unknown";
            }
        }
        if( g_last_cmd < 0 ){
            g_last_cmd = 0x100 | -g_last_cmd;
        }else{
            g_last_cmd = 0;
        }
        cxd224x_debug(indent, "T2T_RSP: %s",
                      s1
            );
    }
    g_last_dir = g_cxd224x.data_write_flag;
    return 0;
}


static int32_t parse_t1(uint32_t indent, const uint8_t *p, int32_t len)
{
    char *name="";
    char buf[32];

    if(g_cxd224x.data_write_flag == 0 || g_cxd224x.data_write_flag == -1)
        return 0;

    switch(*p){
    case 0x78:
        name = "RID_CMD";
        break;
    case 0x00:
        name = "RALL_CMD";
        break;
    case 0x01:
        name = "READ_CMD";
        break;
    case 0x53:
        name = "WRITE_E_CMD";
        break;
    case 0x1A:
        name = "WRITE_NE_CMD";
        break;
    case 0x10:
        name = "RSEG_CMD";
        break;
    case 0x02:
        name = "READ8_CMD";
        break;
    case 0x54:
        name = "WRITE-E8_CMD";
        break;
    case 0x1B:
        name = "WRITE-NE8_CMD";
        break;
    default:
        name = buf;
        sprintf(buf, "TBD:cmd=0x%02X", *p);
        break;    
    }
    cxd224x_debug(indent+1, "T1T_%s", name);

    return 0;
}

static int32_t parse_t2(uint32_t indent, const uint8_t *p, int32_t len)
{
char *name="";
char buf[32];

    switch(*p){
    case 0x30:
        name = "READ_CMD";
        break;
    case 0xA2:
        name = "WRITE_CMD";
        break;
    default:
        name = buf;
        sprintf(buf, "TBD:cmd=0x%02X", *p);
        break;    
    }
    cxd224x_debug(indent+1, "T2T_%s", name);

    return 0;
}

static int32_t parse_t3(uint32_t indent, const uint8_t *p, int32_t len)
{
char *name="";
char buf[32];
int dump_flag=0;

    //cxd224x_debug(indent+1, "T3 len=%d p[0]=0x%02X(%d)<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n", len, p[0], p[0]);

    if(p[0] > len){
        cxd224x_debug(indent+1, "T3T len invalid2\n");
        return 0;
    }
    len--;
    p++;

    switch(*p){
    case 0x00:
        name = "POLL_CMD";
        break;
    case 0x01:
        name = "POLL_RSP";
        break;
    case 0x06:
        name = "CHECK_CMD";
        break;
    case 0x07:
        name = "CHECK_RSP";
        break;
    case 0x08:
        name = "UPDATE_CMD";
        break;
    case 0x09:
        name = "UPDATE_RSP";
        break;
    case 0x22:
        name = "GET_CONTAINER_ISSUE_INFO_CMD";
        break;
    case 0x23:
        name = "GET_CONTAINER_ISSUE_INFO_RSP";
        break;
    case 0xA4:
        name = "CHANGE_ACTIVE_INTERFACE_CMD";
        break;
    case 0xA5:
        name = "CHANGE_ACTIVE_INTERFACE_RSP";
        break;
    case 0xB0:
        name = "FELICA_PUSH_CMD";
        dump_flag = 1;
        break;
    case 0xB1:
        name = "FELICA_PUSH_RSP";
        break;
    default:
        name = buf;
        sprintf(buf, "TBD:cmd=0x%02X", *p);
        break;    
    }
    cxd224x_debug(indent+1, "T3T_%s", name);
    if(dump_flag){
         cxd224x_dump2(indent+1, "", p, len);
    }

    return 0;
}

#define MY_CHECK1(X)    if(memcmp(p,X,sizeof(X))==0){name=#X;goto do_end;}

static int32_t parse_t4(uint32_t indent, const uint8_t *p, int32_t len)
{
char *name="";
char buf[32];
unsigned char SEL_APP_CMD[]   = {0x00, 0xA4, 0x04, 0x00};
unsigned char SEL_FILE_CMD[]  = {0x00, 0xA4, 0x00, 0x0C};
unsigned char READ_CMD[]      = {0x00, 0xB0};
unsigned char UPDATE_CMD[]    = {0x00, 0xD6};

    MY_CHECK1(SEL_APP_CMD)
    MY_CHECK1(SEL_FILE_CMD)
    MY_CHECK1(READ_CMD)
    MY_CHECK1(UPDATE_CMD)

do_end:
    cxd224x_debug(indent+1, "T4T_%s", name);

    return 0;
}

static void uidtostring( char *dest, const uint8_t *p, int32_t len )
{
    *dest ='\0';
    int i;
    for( i=0; i<len; i++){
        int d = (*p & 0xf0)>>4;
        dest[2*i] = d <= 9 ? d + '0' : d - 10 + 'a';
        d = (*p++ & 0xf);
        dest[2*i+1] = d <= 9 ? d + '0' : d - 10 + 'a';
    }
    dest[len*2]='\0';
}

static int32_t parse_iso15693(uint32_t indent, const uint8_t *p, int32_t len)
{
    char uid[8*2+1]="";
    int status=0;
    uint8_t cmd,flag;
    char buf[100]="";
    int optlen,postlen;
    const char *optfmt, *postfmt;

    if( len <= 1 || p == NULL ){
        status=1;
    }
    if( status ){
        return status;
    }
    if( g_last_cmd ){
        if( !g_cxd224x.is_brcm ){
            uint8_t rsp = p[len-1];
            switch( rsp ){
            case NCI_STATUS_OK:
                sprintf(buf, "STATUS_OK");
                break;
            case NCI_STATUS_REJECTED:
                sprintf(buf, "STATUS_REJECTED(0x%x)", rsp);
                break;
            case NCI_STATUS_MESSAGE_CORRUPTED:
                sprintf(buf, "STATUS_MESSAGE_CORRUPTED(0x%x)", rsp);
                break;
            case NCI_STATUS_BUFFER_FULL:
                sprintf(buf, "STATUS_BUFFER_FULL(0x%x)", rsp);
                break;
            case NCI_STATUS_FAILED:
                sprintf(buf, "STATUS_FAILED(0x%x)", rsp);
                break;
            default:
                sprintf(buf, "STATUS_xx(0x%x)", rsp);
            }
        }else{
            sprintf(buf, "RSP");
        }
        cxd224x_debug(indent+1, "I93_%s", buf);
        g_last_cmd =0;
        return 0;
    }
    flag = *p++;
    cmd = *p++;
    len -=2;
    g_last_cmd =cmd;
    optlen=postlen=0;
    switch(cmd){
    case I93_CMD_INVENTORY                   :    /* Inventory  */
        sprintf(buf, "INVENTORY:");
        optlen=1;
        optfmt=" afi=0x%x";
        break;
    case I93_CMD_STAY_QUIET                  :    /* Stay Quiet */
        sprintf(buf, "STAY_QUIET:");
        break;
    case I93_CMD_READ_SINGLE_BLOCK           :    /* Read single block     */
        sprintf(buf, "READ_SINGLE_BLOCK:");
        if( flag & I93_FLAG_PROT_EXT_YES ){
            postlen=2;
            postfmt=" blk=%02x:%02x";
        }else{
            postlen=1;
            postfmt=" blk=%d";
        }
        break;
    case I93_CMD_WRITE_SINGLE_BLOCK          :    /* Write single block    */
        sprintf(buf, "WRITE_SINGLE_BLOCK:");
        if( flag & I93_FLAG_PROT_EXT_YES ){
            postlen=2;
            postfmt=" blk=%02x:%02x";
        }else{
            postlen=1;
            postfmt=" blk=%d";
        }
        break;
    case I93_CMD_LOCK_BLOCK                  :    /* Lock block            */
        sprintf(buf, "LOCK_BLOCK:");
        postlen=1;
        postfmt=" blks=%d";
        break;
    case I93_CMD_READ_MULTI_BLOCK            :    /* Read multiple blocks  */
        sprintf(buf, "READ_MULTI_BLOCK:");
        if( flag & I93_FLAG_PROT_EXT_YES ){
            postlen=3;
            postfmt=" fst=%02x:%02x blks=%d";
        }else{
            postlen=2;
            postfmt=" fst=%d blks=%d";
        }
        break;
    case I93_CMD_WRITE_MULTI_BLOCK           :    /* Write multiple blocks */
        sprintf(buf, "WRITE_MULTI_BLOCK:");
        postlen=2;
        postfmt=" fst=%d blks=%d";
        break;
    case I93_CMD_SELECT                      :    /* Select                */
        sprintf(buf, "SELECT:");
        break;
    case I93_CMD_RESET_TO_READY              :    /* Reset to ready        */
        sprintf(buf, "RESET_TO_READY:");
        break;
    case I93_CMD_WRITE_AFI                   :    /* Wreite AFI            */
        sprintf(buf, "WRITE_AFI:");
        postlen=1;
        postfmt=" afi=0x%x";
        break;
    case I93_CMD_LOCK_AFI                    :    /* Lock AFI              */
        sprintf(buf, "LOCK_AFI:");
        break;
    case I93_CMD_WRITE_DSFID                 :    /* Write DSFID           */
        sprintf(buf, "WRITE_DSFID:");
        postlen=1;
        postfmt=" dsfid=0x%x";
        break;
    case I93_CMD_LOCK_DSFID                  :    /* Lock DSFID            */
        sprintf(buf, "LOCK_DSFID:");
        break;
    case I93_CMD_GET_SYS_INFO                :    /* Get system information */
        sprintf(buf, "GET_SYS_INFO:");
        break;
    case I93_CMD_GET_MULTI_BLK_SEC           :
        sprintf(buf, "GET_MULTI_BLK_SEC:");
        if( flag & I93_FLAG_PROT_EXT_YES ){
            postlen=4;
            postfmt=" fst=%02x:%02x blks=%02x:%02x";
        }else{
            postlen=2;
            postfmt=" fst=%d blks=%d";
        }
        break;
    default:
        g_last_cmd=0;
    }
    if( flag ){
        if( flag & I93_FLAG_INVENTORY_MASK ){
            sprintf(&buf[strlen(buf)], " %c%c%c", 
                    flag & I93_FLAG_OPTION_MASK ? 'O' : 'o',
                    flag & I93_FLAG_SLOT_MASK ? 'S' : 's',
                    flag & I93_FLAG_AFI_MASK ? 'A' : '_'
                );
        }else{
            sprintf(&buf[strlen(buf)], " %c%c%c", 
                    flag & I93_FLAG_OPTION_MASK ? 'O' : 'o',
                    flag & I93_FLAG_ADDRESS_MASK ? 'A' : '_',
                    flag & I93_FLAG_SELECT_MASK ? 'S' : '_'
                );
        }
        sprintf(&buf[strlen(buf)], ":%c%c%c%c",
                flag & I93_FLAG_PROT_EXT_MASK ? 'P' : 'p',
                flag & I93_FLAG_INVENTORY_MASK ? 'I' : 'i',
                flag & I93_FLAG_DATA_RATE_MASK ? 'D' : 'd',
                flag & I93_FLAG_SUB_CARRIER_MASK ? 'C' : 'c'
            );
    }

    if( len >= optlen + I93_UID_BYTE_LEN ){
        uidtostring( uid, &p[optlen], I93_UID_BYTE_LEN );
        sprintf(&buf[strlen(buf)], " uid=%s", uid);
    }
    if( optlen ){
        sprintf(&buf[strlen(buf)], optfmt, p[0], p[1], p[2], p[3]);
    }
    if( postlen && len >= optlen + I93_UID_BYTE_LEN + postlen ){
        sprintf(&buf[strlen(buf)], postfmt, 
                p[optlen + I93_UID_BYTE_LEN], 
                p[optlen + I93_UID_BYTE_LEN + 1], 
                p[optlen + I93_UID_BYTE_LEN + 2], 
                p[optlen + I93_UID_BYTE_LEN + 3]);
    }
    cxd224x_debug(indent+1, "I93_%s", buf);

    return 0;
}


// parse NCI data packet
static int32_t parse_data(uint32_t indent, uint8_t conn_id, const uint8_t *p, int32_t len)
{

    if(p == NULL || len <= 0){
        return 0;
    }
    else  /* g_cxd224x.parse_dtype == 0 : auto detect */
    {
        tDATA_TYPE dtype;
        dtype = (g_cxd224x.parse_dtype != 0) ? g_cxd224x.parse_dtype : get_data_type();

        switch ( dtype ){
        case DATA_TYPE_T1T:
            parse_t1(indent, p, len);
            break;
        case DATA_TYPE_T2T:
//            parse_t2(indent, p, len);
            parse_t2t(indent, p, len);
            break;
        case DATA_TYPE_T3T:
            parse_t3(indent, p, len);
            break;
        case DATA_TYPE_ISODEP:
//            parse_t4(indent, p, len);
            parse_isodep(indent, p, len);
            break;
        case DATA_TYPE_NFCDEP:
            parse_llcp(indent, p, len);
            break;
        case DATA_TYPE_I93:
            parse_iso15693(indent, p, len);
            break;
        case DATA_TYPE_HCI:
            parse_hci(indent, p, len);
            break;
        }
    }
    return 0;
}
