/******************************************************************************
 *
 *  Copyright (C) 2013 Sony Corporation
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
#include "OverrideLog.h"
extern "C"
{
#include "nfc_hal_target.h"
}
#include <cutils/log.h>
#include "config.h"
#include <string.h>

#ifndef BTE_LOG_BUF_SIZE
    #define BTE_LOG_BUF_SIZE  1024
#endif
#define BTE_LOG_MAX_SIZE  (BTE_LOG_BUF_SIZE - 12)
#define MAX_NCI_PACKET_SIZE  259
#define MAX_LOGCAT_LINE     4096
static char log_line[MAX_LOGCAT_LINE];
static const char* sTable = "0123456789abcdef";

extern UINT32 ScrProtocolTraceFlag_Hal;         // = SCR_PROTO_TRACE_ALL; // 0x017F;
extern "C"
{
    void DispNci (UINT8 *p, UINT16 len, BOOLEAN is_recv);
    void DispHciCmd_Hal (BT_HDR *p_buf);
    void DispHciEvt_Hal (BT_HDR *p_buf);
}

void LogMsgH (UINT32 trace_set_mask, const char *fmt_str, ...)
{
    char buffer [BTE_LOG_BUF_SIZE]; /* remove static for multi thread safe */
    va_list ap;
    UINT32 trace_type = trace_set_mask & 0x07; //lower 3 bits contain trace type
    int android_log_type = ANDROID_LOG_INFO;

    va_start (ap, fmt_str);
    vsnprintf (buffer, BTE_LOG_MAX_SIZE, fmt_str, ap);
    va_end (ap);
    if (trace_type == TRACE_TYPE_ERROR)
        android_log_type = ANDROID_LOG_ERROR;

    __android_log_write (android_log_type, "NfcNciHal", buffer);
}

#define PRINT_CMD_LEN 40
static int print_cmd( UINT8 *data, char *line_buf );

void DispNci (UINT8 *data, UINT16 len, BOOLEAN is_recv)
{
        if (!(ScrProtocolTraceFlag_Hal & SCR_PROTO_TRACE_NCI))
            return;

        char line_buf[(MAX_NCI_PACKET_SIZE*2)+1 + PRINT_CMD_LEN];
        int i,j;
        j=print_cmd( data, line_buf );

        for(i = 0; i < len && j < (int )sizeof(line_buf)-3; i++)
        {
            line_buf[j++] = sTable[(*data >> 4) & 0xf];
            line_buf[j++] = sTable[*data & 0xf];
            data++;
        }
        line_buf[j] = '\0';

#ifndef SPZ_IMPL
        __android_log_write(ANDROID_LOG_DEBUG, (is_recv) ? "DbgNciR": "DbgNciX", line_buf);
#else
        __android_log_write(ANDROID_LOG_WARN, (is_recv) ? "DbgNciR": "DbgNciX", line_buf);
#endif /* SPZ_IMPL */
}


void DispHciCmd_Hal (BT_HDR *p_buf)
{
    int i,j;
    int nBytes = ((BT_HDR_SIZE + p_buf->offset + p_buf->len)*2)+1;
    UINT8 * data = (UINT8*) p_buf;
    int data_len = BT_HDR_SIZE + p_buf->offset + p_buf->len;

    if (h_appl_trace_level < BT_TRACE_LEVEL_DEBUG)
        return;

    if (nBytes > (int )sizeof(log_line))
        return;

    for(i = 0, j = 0; i < data_len && j < (int )sizeof(log_line)-3; i++)
    {
        log_line[j++] = sTable[(*data >> 4) & 0xf];
        log_line[j++] = sTable[*data & 0xf];
        data++;
    }
    log_line[j] = '\0';

        __android_log_write(ANDROID_LOG_DEBUG, "DbgHciX", log_line);

}


void DispHciEvt_Hal (BT_HDR *p_buf)
{
    int i,j;
    int nBytes = ((BT_HDR_SIZE + p_buf->offset + p_buf->len)*2)+1;
    UINT8 * data = (UINT8*) p_buf;
    int data_len = BT_HDR_SIZE + p_buf->offset + p_buf->len;

    if (h_appl_trace_level < BT_TRACE_LEVEL_DEBUG)
        return;

    if (nBytes > (int )sizeof(log_line))
        return;

    for(i = 0, j = 0; i < data_len && j < (int )sizeof(log_line)-3; i++)
    {
        log_line[j++] = sTable[(*data >> 4) & 0xf];
        log_line[j++] = sTable[*data & 0xf];
        data++;
    }
    log_line[j] = '\0';

        __android_log_write(ANDROID_LOG_DEBUG, "DbgHciR", log_line);
}

#include "nci_defs.h"
static const char *mt_str(UINT8 data)
{
    UINT8 mt = (NCI_MT_MASK & data)>>NCI_MT_SHIFT;
    switch( mt ){
    case NCI_MT_CMD:   /* (NCI_MT_CMD << NCI_MT_SHIFT) = 0x20 */
        return "CMD";
    case NCI_MT_RSP:   /* (NCI_MT_RSP << NCI_MT_SHIFT) = 0x40 */
        return "RSP";
    case NCI_MT_NTF:   /* (NCI_MT_NTF << NCI_MT_SHIFT) = 0x60 */
        return "NTF";
    case NCI_MT_CFG:
        return "CFC";
    default:
        ;
    }
    return "?";
}

static const char *cmd_core_str(UINT8 oid)
{
    switch( oid & NCI_OID_MASK ){
    case NCI_MSG_CORE_RESET              :
        return "RESET";
    case NCI_MSG_CORE_INIT               :
        return "INIT";
    case NCI_MSG_CORE_SET_CONFIG         :
        return "SET_CONFIG";
    case NCI_MSG_CORE_GET_CONFIG         :
        return "GET_CONFIG";
    case NCI_MSG_CORE_CONN_CREATE        :
        return "CONN_CREATE";
    case NCI_MSG_CORE_CONN_CLOSE         :
        return "CONN_CLOSE";
    case NCI_MSG_CORE_CONN_CREDITS       :
        return "CONN_CREDITS";
    case NCI_MSG_CORE_GEN_ERR_STATUS     :
        return "GEN_ERR_STATUS";
    case NCI_MSG_CORE_INTF_ERR_STATUS    :
        return "INTF_ERR_STATUS";
    default:
        ;
    }
    return "?";
}
static const char *rf_str(UINT8 oid)
{
    switch( oid & NCI_OID_MASK ){
    case NCI_MSG_RF_DISCOVER_MAP         :
        return "DISCOVER_MAP";
    case NCI_MSG_RF_SET_ROUTING          :
        return "SET_ROUTING";
    case NCI_MSG_RF_GET_ROUTING          :
        return "GET_ROUTING";
    case NCI_MSG_RF_DISCOVER             :
        return "RF_DISCOVER";
    case NCI_MSG_RF_DISCOVER_SELECT      :
        return "DISCOVER_SELECT";
    case NCI_MSG_RF_INTF_ACTIVATED       :
        return "INTF_ACTIVATED";
    case NCI_MSG_RF_DEACTIVATE           :
        return "DEACTIVATE";
    case NCI_MSG_RF_FIELD                :
        return "FIELD";
    case NCI_MSG_RF_T3T_POLLING          :
        return "T3T_POLLING";
    case NCI_MSG_RF_EE_ACTION            :
        return "EE_ACTION";
    case NCI_MSG_RF_EE_DISCOVERY_REQ     :
        return "EE_DISCOVERY_REQ";
    case NCI_MSG_RF_PARAMETER_UPDATE     :
        return "PARAMETER_UPDATE";
    default:
        ;
    }
    return "?";
}
static const char *nfcee_str(UINT8 oid)
{
    switch( oid & NCI_OID_MASK ){
    case NCI_MSG_NFCEE_DISCOVER         :
        return "DISCOVER";
    case NCI_MSG_NFCEE_MODE_SET         :
        return "MODE_SET";
    default:
        ;
    }
    return "?";
}

static int print_cmd( UINT8 *data, char *line_buf )
{
    UINT8 mt = (NCI_MT_MASK & data[0])>>NCI_MT_SHIFT;
    int j;
    line_buf[0]='\0';
    switch( mt )
    {
    case NCI_MT_CMD:
    case NCI_MT_RSP:
    case NCI_MT_NTF:
    case NCI_MT_CFG:
        switch( data[0] & NCI_GID_MASK)
        {
        case NCI_GID_CORE:
            snprintf( line_buf, PRINT_CMD_LEN, "CORE:%s %s len=%d ", mt_str(data[0]), cmd_core_str(data[1]), data[2]);
            break;
        case NCI_GID_RF_MANAGE:
            snprintf( line_buf, PRINT_CMD_LEN, "RF:%s %s len=%d ", mt_str(data[0]), rf_str(data[1]), data[2]);
            break;
        case NCI_GID_EE_MANAGE:
            snprintf( line_buf, PRINT_CMD_LEN, "NFCEE:%s %s len=%d ", mt_str(data[0]), nfcee_str(data[1]), data[2]);
            break;
        case NCI_GID_PROP:
            snprintf( line_buf, PRINT_CMD_LEN, "PROP: ");
            break;
        default:
            ;
        }
        break;
    case NCI_MT_DATA:
        snprintf( line_buf, PRINT_CMD_LEN, "DATA:cid=%d len=%d ", (data[0] & NCI_GID_MASK), data[2]);
        break;
    default:
        ;
    }

    j = strlen(line_buf);
    j = PRINT_CMD_LEN < j ? PRINT_CMD_LEN : j ;
    return j;
}
