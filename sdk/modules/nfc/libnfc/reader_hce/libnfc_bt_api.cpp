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
#ifndef SPZ1_IMPL
#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#endif /* SPZ1_IMPL */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "config.h"

#ifdef SPZ1_IMPL
#include "syscall_wrapper.h"
#endif

#ifndef SPZ_IMPL
#include "libnfc_api.h"
#else
#include "nfc/libnfc_api.h"
#ifdef SPZ1_IMPL
#include "../sdk_include/libnfc_api.h"
#endif // SPZ1_IMPL
#endif /* SPZ_IMPL */

#ifdef SPZ1_IMPL
#include "spz_nfc_log.h"
DBG_DEFINE_MODULE(NF);
#define DBG_MODULE NF
#endif //SPZ1_IMPL

#define DEBUG_TRUE 1

#ifdef DEBUG_TRUE
#ifndef SPZ1_IMPL
#define DBG_BT_NDEF_PRINT(...) printf(__VA_ARGS__)
#else
#define DBG_BT_NDEF_PRINT(...) DBG_LOGF_WARN(__VA_ARGS__)
#endif /* SPZ1_IMPL */
#else
#define DBG_BT_NDEF_PRINT(...)
#endif /* DEBUG_TRUE */

#define NDEF_RECORD_HEADER_MASK 0xC0

#define N_RECORD_MB_MASK  0x80
#define N_RECORD_ME_MASK  0x40
#define N_RECORD_CF_MASK  0x20
#define N_RECORD_SR_MASK  0x10
#define N_RECORD_IL_MASK  0x08
#define N_RECORD_TNF_MASK 0x07

char record_type_bt[]  = "application/vnd.bluetooth.ep.oob";
char record_type_ble[] = "application/vnd.bluetooth.le.oob";

/*******************************************************************************
**
** Function:        NF_ParseNdefData
**
** Description:     ho_info:   Bluetooth handover pase information.
**                  ndef_data: Ndef data.
**                  ndef_size: Ndef data size.
**
** Returns:         None
**
*******************************************************************************/
int NF_ParseNdefData(HO_INFO_S *ho_info, unsigned char *ndef_data, unsigned short ndef_size)
{
    int   i;
    int   posi = 0;
    bool  id_flag;
    short oob_tmp;
    unsigned short count;
    unsigned short conf_len = 0;

    DBG_BT_NDEF_PRINT("%s start\n", __FUNCTION__);

    /*
     * Parameters Check
     */
    if(ndef_data == NULL) {
        DBG_BT_NDEF_PRINT("%s: ndef data addr is null\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    if(ndef_size == 0) {
        DBG_BT_NDEF_PRINT("%s: ndef data size is 0\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    /*
     * Decode NDEF
     */
    count = 0;
    if((ndef_data[count] & N_RECORD_MB_MASK) != N_RECORD_MB_MASK) {
        DBG_BT_NDEF_PRINT("%s: Header Error 1\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    if((ndef_data[count] & NDEF_RECORD_HEADER_MASK) == N_RECORD_MB_MASK) {

        /*
         * Handover Select Record parse
         */
        if(RECORD_TYPE_LENGTH < ndef_data[count + 1]) {
            DBG_BT_NDEF_PRINT("%s: hs Record Type Length Error\n", __FUNCTION__);
            return D_NF_PARAM_ERR;
        }

        ho_info->ho_sel_record.record_type_len = ndef_data[++count];
        ho_info->ho_sel_record.payload_len     = ndef_data[++count];

        for(i = 0; i < ho_info->ho_sel_record.record_type_len; i++) {
            ho_info->ho_sel_record.record_type[i] = ndef_data[++count];
        }

        ho_info->ho_sel_record.version_num = ndef_data[++count];

        if((ndef_data[++count] & N_RECORD_ME_MASK) != N_RECORD_ME_MASK) {
            DBG_BT_NDEF_PRINT("%s: Header Error 2 \n", __FUNCTION__);
            return D_NF_PARAM_ERR;
        }

        /*
         * Alternative Carrier Record parse
         */
        if(RECORD_TYPE_LENGTH < ndef_data[count + 1]) {
            DBG_BT_NDEF_PRINT("%s: ac Record Type Length Error\n", __FUNCTION__);
            return D_NF_PARAM_ERR;
        }

        ho_info->alt_car_record.record_type_len = ndef_data[++count];
        ho_info->alt_car_record.payload_len     = ndef_data[++count];

        for(i = 0; i < ho_info->alt_car_record.record_type_len; i++) {
            ho_info->alt_car_record.record_type[i] = ndef_data[++count];
        }

        ho_info->alt_car_record.carrier_flag         = ndef_data[++count];
        ho_info->alt_car_record.carrier_data_ref_len = ndef_data[++count];
        ho_info->alt_car_record.carrier_data_ref     = ndef_data[++count];
        ho_info->alt_car_record.aux_data_count       = ndef_data[++count];
        count++;
    }

    if((ndef_data[count] & N_RECORD_IL_MASK) == N_RECORD_IL_MASK) {
        id_flag = true;
    }
    else {
        id_flag = false;
    }

    /*
     * Bluetooth Carrier Configuration Record parse
     */
    if(RECORD_TYPE_NAME_LENGTH < ndef_data[count + 1]) {
        DBG_BT_NDEF_PRINT("%s: Record Type Name Size Error\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    ho_info->car_conf_record.record_type_len = ndef_data[++count];
    ho_info->car_conf_record.payload_len     = ndef_data[++count];

    if(id_flag) {
        ho_info->car_conf_record.payload_id_len  = ndef_data[++count];
    }

    for(i = 0; i < ho_info->car_conf_record.record_type_len; i++) {
        ho_info->car_conf_record.record_type[i] = ndef_data[++count];
    }

    if(strncmp(record_type_bt,
                ho_info->car_conf_record.record_type,
                ho_info->car_conf_record.record_type_len) == 0) {

        if(id_flag) {
            ho_info->car_conf_record.payload_id = ndef_data[++count];
        }

        ho_info->car_conf_record.bt_oob_data_len = ndef_data[++count];
        oob_tmp = ndef_data[++count];
        oob_tmp = oob_tmp << 8;
        ho_info->car_conf_record.bt_oob_data_len |= oob_tmp;

        for(i = (BT_DEV_ADDR_LENGTH - 1); i >= 0; i--) {
           ho_info->car_conf_record.bt_dev_addr[i] = ndef_data[++count];
        }
    }
    else if(strncmp(record_type_ble,
                   ho_info->car_conf_record.record_type,
                   ho_info->car_conf_record.record_type_len) != 0) {

        if(id_flag) {
            ho_info->car_conf_record.payload_id = ndef_data[++count];
        }
    }
    else {
        DBG_BT_NDEF_PRINT("%s: Type Name Mismatch\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    count++;
    posi = count;
    while((conf_len + count) < ndef_size) {
        conf_len += (ndef_data[posi] + 1);
        posi     += (ndef_data[posi] + 1);
    }

    ho_info->car_conf_record.conf_len = conf_len;

    if((ho_info->car_conf_record.conf_len + count) != ndef_size) {
        DBG_BT_NDEF_PRINT("%s: Size Error\n", __FUNCTION__);
        return D_NF_PROC_ERR;
    }

    ho_info->car_conf_record.conf_data = &ndef_data[count];

    DBG_BT_NDEF_PRINT("%s end\n", __FUNCTION__);

    return D_NF_SUCCESS;
}

/*******************************************************************************
**
** Function:        NF_EncodeNdefData
**
** Description:     encode_info:   Bluetooth handover encode information.
**                  buff_size: buffer size.
**                  buff: buffer pointer for ndef data.
**                  en_ndef_size:encode ndef data size
**
** Returns:         success / fail
**                  
*******************************************************************************/
int NF_EncodeNdefData(ENCODE_INFO *encode_info, unsigned short buff_size, unsigned char *buff, unsigned short *en_ndef_size)
{
    int            i = 0, j = 0;
    short          oob_temp;
    unsigned short count;
    unsigned char  data_len = 0;

    DBG_BT_NDEF_PRINT("%s start\n", __FUNCTION__);

    /*
     * Parameters Check
     */
    if(encode_info == NULL) {
        DBG_BT_NDEF_PRINT("%s: encode info is null\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    if((encode_info->encord_type != E_TYPE_BT) && (encode_info->encord_type != E_TYPE_BLE)) {
        DBG_BT_NDEF_PRINT("%s: encord type is invalid\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    if((encode_info->encord_type == E_TYPE_BT) && (encode_info->bt_dev_addr == NULL)) {
        DBG_BT_NDEF_PRINT("%s: bt dev addr is null\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    if(encode_info->conf_data == NULL) {
        DBG_BT_NDEF_PRINT("%s: config data addr is null\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    if(encode_info->encord_type == E_TYPE_BT) {
        data_len = encode_info->conf_len + BT_INFO_SIZE;
    }
    else if(encode_info->encord_type == E_TYPE_BLE) {
        data_len = encode_info->conf_len + BLE_INFO_SIZE;
    }
    else {
        /* nothing to do */
    }
    if(buff_size < data_len) {
        DBG_BT_NDEF_PRINT("%s: buff_size is smaller than ndef size\n", __FUNCTION__);
        return D_NF_PARAM_ERR;
    }

    /*
     * Encode NDEF
     */
    count = 0;
    buff[count++] = (unsigned char)0xD2; /* NDEF Record Header */
    buff[count++] = (unsigned char)0x20; /* Record Type */

    if(encode_info->conf_len > 0) {

        /*
         * Set bluetooth configuration parameter
         */
        if(encode_info->encord_type == E_TYPE_BT) {
            /* Payload Length */
            buff[count++] = (unsigned char)(encode_info->conf_len + BT_DEV_ADDR_LENGTH + 2);

            /* Record Type Name */
            for(i = 0; i < BT_NAME_SIZE; i++) {
                buff[count++] = record_type_bt[i];
            }

            /* OOB Optional Data Length */
            oob_temp = (encode_info->conf_len + BT_DEV_ADDR_LENGTH + 2);
            for(i = 0; i < 2 ; i++) {
                buff[count++] = (unsigned char)(oob_temp >> j);
                j += 8;
            }

            /* Bluetooth Device Address */
            for(i = (BT_DEV_ADDR_LENGTH - 1); i >= 0; i--) {
               buff[count++] = encode_info->bt_dev_addr[i];
            }

            /* Config Data */
            for(i = 0; i < (encode_info->conf_len); i++) {
                buff[count++] = encode_info->conf_data[i];
            }

        }

        /*
         * Set bluetooth configuration parameter
         */
        else if(encode_info->encord_type == E_TYPE_BLE) {
            /* Payload Length */
            buff[count++] = encode_info->conf_len;

            /* Record Type Name */
            for(i = 0; i < BLE_NAME_SIZE; i++) {
                buff[count++] = record_type_ble[i];
            }

            /* Config Data */
            for(i = 0; i < encode_info->conf_len; i++) {
                buff[count++] = *(encode_info->conf_data + i);
            }
        }
        else {
            /* nothing to do */
        }
    }

    if(count != data_len) {
        DBG_BT_NDEF_PRINT("%s: Count Size Error\n", __FUNCTION__);
        return D_NF_PROC_ERR;
    }

    *en_ndef_size = count;

    DBG_BT_NDEF_PRINT("%s end\n", __FUNCTION__);

    return D_NF_SUCCESS;
}


void NF_ParsePrint(HO_INFO_S *ho_info)
{
    int i = 0;

    /* Print Parsed Ndef */
    DBG_BT_NDEF_PRINT("\nHandover Select Record\n");

    DBG_BT_NDEF_PRINT("Record Type Length:            0x%02x\n",
        ho_info->ho_sel_record.record_type_len);
    DBG_BT_NDEF_PRINT("Payload Length:                0x%02x\n",
        ho_info->ho_sel_record.payload_len);

    DBG_BT_NDEF_PRINT("Record Type:                   0x%02x 0x%02x ",
        ho_info->ho_sel_record.record_type[0],
        ho_info->ho_sel_record.record_type[1]);

    for(i = 0; i < ho_info->alt_car_record.record_type_len; i++) {
        DBG_BT_NDEF_PRINT("%c", ho_info->ho_sel_record.record_type[i]);
    }

    DBG_BT_NDEF_PRINT("\nVersion Number:                0x%02x\n",
        ho_info->ho_sel_record.version_num);

    DBG_BT_NDEF_PRINT("\nAltemative Carrier Record\n");

    DBG_BT_NDEF_PRINT("Record Type Length:            0x%02x\n",
        ho_info->alt_car_record.record_type_len);

    DBG_BT_NDEF_PRINT("Payload Length:                0x%02x\n",
        ho_info->alt_car_record.payload_len);

    DBG_BT_NDEF_PRINT("Record Type:                   0x%02x 0x%02x ",
        ho_info->alt_car_record.record_type[0],
        ho_info->alt_car_record.record_type[1]);

    for(i = 0; i < ho_info->alt_car_record.record_type_len; i++) {
       DBG_BT_NDEF_PRINT("%c", ho_info->alt_car_record.record_type[i]);
    }

    DBG_BT_NDEF_PRINT("\nCarrier Flags:                 0x%02x\n",
        ho_info->alt_car_record.carrier_flag);

    DBG_BT_NDEF_PRINT("Carrier Data Reference Length: 0x%02x\n",
        ho_info->alt_car_record.carrier_data_ref_len);

    DBG_BT_NDEF_PRINT("Carrier Data Reference:        0x%02x\n",
        ho_info->alt_car_record.carrier_data_ref);

    DBG_BT_NDEF_PRINT("Auxiliary Data Reference Count:0x%02x\n",
        ho_info->alt_car_record.aux_data_count);

    DBG_BT_NDEF_PRINT("\nBluetooth Carrier Configuration Record\n");

    DBG_BT_NDEF_PRINT("Record Type Length:            0x%02x\n",
        ho_info->car_conf_record.record_type_len);

    DBG_BT_NDEF_PRINT("Pyload Length:                 0x%02x\n",
        ho_info->car_conf_record.payload_len);

    DBG_BT_NDEF_PRINT("Payload ID Length:             0x%02x\n",
        ho_info->car_conf_record.payload_id_len);

    DBG_BT_NDEF_PRINT("Record Type Name:              ");

    for(i = 0; i < ho_info->car_conf_record.record_type_len; i++) {
        DBG_BT_NDEF_PRINT("%c", ho_info->car_conf_record.record_type[i]);
    }

    DBG_BT_NDEF_PRINT("\nPayload ID:                    0x%02x\n",
        ho_info->car_conf_record.payload_id);

    DBG_BT_NDEF_PRINT("Bluetooth OOB Data Length:     0x%02x\n",
        ho_info->car_conf_record.bt_oob_data_len);

    DBG_BT_NDEF_PRINT("Bluetooth Device Address:      ");

    for(i = 0; i < BT_DEV_ADDR_LENGTH; i++) {
        DBG_BT_NDEF_PRINT("%02x", ho_info->car_conf_record.bt_dev_addr[i]);
        if(i < (BT_DEV_ADDR_LENGTH - 1))
            DBG_BT_NDEF_PRINT(":");
    }

    DBG_BT_NDEF_PRINT("\nConfig Total Length:           0x%02x\n",
        ho_info->car_conf_record.conf_len);

    DBG_BT_NDEF_PRINT("Config Data:                   %p 0x%02x\n\n",
        ho_info->car_conf_record.conf_data,
        *ho_info->car_conf_record.conf_data);
}
