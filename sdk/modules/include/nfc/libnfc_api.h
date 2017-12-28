/*
 * Copyright 2015 Sony Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/**
 * @file libnfc_api.h
 * @brief NFC(Near Field Communication) API.
 *
 * To use NFC features,  nf_api.h needs to be included in source code as following.
 * <pre>#include	<nfc/nf_api.h></pre>
 */


#ifndef _LIBNFC_API_H_
#define _LIBNFC_API_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @defgroup nfc_libnfc NFC Library(libnfc)
 * @brief Applicaton Interface provided by NFC Library(libnfc)
 *
 * To use NFC features,  nf_api.h needs to be included in source code as following.
 * <pre>#include	<nfc/nf_api.h></pre>
 * @{
 */


typedef char           SCHAR;
typedef short          SSHORT;
typedef int            SINT;
typedef long           SLONG;
typedef unsigned char  UCHAR;
typedef unsigned short USHORT;
typedef unsigned int   UINT;
typedef unsigned long  ULONG;


#define READ_AND_HCE 0  /**< Operation mode(reader_hce_mode in NF_start) for HCE and Reader */
#define READ_ONLY    1  /**< Operation mode(reader_hce_mode in NF_start) for Reader */
#define HCE_ONLY     2  /**< Operation mode(reader_hce_mode in NF_start) for HCE */

#define D_NF_SUCCESS    0 /**< success in NF_xxx return code */
#define D_NF_PARAM_ERR -1 /**< parameter error in NF_xxx return code */
#define D_NF_DUPL_ERR  -2 /**< duplicate error in NF_xxx return code */
#define D_NF_PROC_ERR  -3 /**< proccess error in NF_xxx return code */

#define D_EVENT_TAG_READ 0 /**< Event of "peer tag is detected" used in RESULT_INFO */
#define D_EVENT_HCE      1 /**< Event of "peer NFC Reader is detected" used in RESULT_INFO */
#define D_EVENT_ERROR    2 /**< Event of "error is detected" used in RESULT_INFO */

#define D_NFC_SUCCESS        0x00 /**< Success status in RESULT_INFO */
#define D_NFC_FAILED         0xFF /**< Failed status in RESULT_INFO */
#define D_NFC_TIMEOUT        0xF0 /**< "Timeout occur" status in RESULT_INFO */
#define D_NFC_NDEF_SIZE_OVER 0xE0 /**< "Size of received ndef is too big" status in RESULT_INFO */

#define LOCAL_NDEF_MAX 255

#define RECORD_TYPE_LENGTH       2
#define RECORD_TYPE_NAME_LENGTH 32

#define BT_DEV_ADDR_LENGTH       6
#define BLE_DEV_ADDR_LENGTH      7

#define BT_NAME_SIZE            32
#define BLE_NAME_SIZE           32

#define NDEF_RECORD_HEADER_SIZE  1
#define RECORD_TYPE_LENGTH_SIZE  1
#define PAYLOAD_LENGTH_SIZE      1
#define BT_DATA_LENGTH_SIZE      2

#define BT_INFO_SIZE           ( NDEF_RECORD_HEADER_SIZE \
                               + RECORD_TYPE_LENGTH_SIZE \
                               + PAYLOAD_LENGTH_SIZE     \
                               + BT_DATA_LENGTH_SIZE     \
                               + BT_DEV_ADDR_LENGTH      \
                               + BT_NAME_SIZE )

#define BLE_INFO_SIZE          ( NDEF_RECORD_HEADER_SIZE \
                               + RECORD_TYPE_LENGTH_SIZE \
                               + PAYLOAD_LENGTH_SIZE     \
                               + BLE_NAME_SIZE )

/**
 * Specify type of NDEF for Bluetooth (Classic) or BLE 
 *
 */
typedef enum {
    E_TYPE_BT,   ///< Bluetooth (Classic)
    E_TYPE_BLE   ///< BLE
} E_ENCORD_TYPE; 

/**
 *
 * Bluetooth Handover Select Record
 *
 */
typedef struct {
    unsigned char   record_type_len;                 /**<[out] Record Type Length */
    unsigned char   payload_len;                     /**<[out] Payload Length */
    char            record_type[RECORD_TYPE_LENGTH]; /**<[out] Record Type */
    unsigned char   version_num;                     /**<[out] Version Number */
} HO_SEL_RECD;

/**
 *
 * Bluetooth Alternative Carrier Record
 *
 */
typedef struct {
    unsigned char   record_type_len;                 /**<[out] Record Type Length */
    unsigned char   payload_len;                     /**<[out] Payload Length */
    char            record_type[RECORD_TYPE_LENGTH]; /**<[out] Record Type */
    unsigned char   carrier_flag;                    /**<[out] Carrier Flags */
    unsigned char   carrier_data_ref_len;            /**<[out] Carrier Data Reference Length */
    unsigned char   carrier_data_ref;                /**<[out] Carrier Data Reference */
    unsigned char   aux_data_count;                  /**<[out] Auxiliary Data Reference Count */
} ALT_CAR_RECD;

/**
 *
 * Bluetooth Carrier Configuration Record
 *
 */
typedef struct {
    unsigned char   record_type_len;                 /**<[out] Record Type Length */
    unsigned char   payload_len;                     /**<[out] Payload Length */
    unsigned char   payload_id_len;                  /**<[out] Payload ID Length */
    char            record_type[RECORD_TYPE_NAME_LENGTH]; /**<[out] Record Type */
    unsigned char   payload_id;                      /**<[out] Payload ID */
    unsigned short  bt_oob_data_len;                 /**<[out] Bluetooth OOB(Out-Of-Band) Data Length */
    unsigned char   bt_dev_addr[BT_DEV_ADDR_LENGTH]; /**<[out] Bluetooth Device Address */
    unsigned char   conf_len;                        /**<[out] Configuration Data Length */
    unsigned char   *conf_data;                      /**<[out] Configuration Data */
} CAR_CONF_RECD;


/**
 *
 * Bluetooth Handover related information
 *
 */
typedef struct {
    HO_SEL_RECD      ho_sel_record;                  /**<[out] Handover Select Record */
    ALT_CAR_RECD     alt_car_record;                 /**<[out] Alternative Carrier Record */
    CAR_CONF_RECD    car_conf_record;                /**<[out] Carrier Configuration Record */
} HO_INFO_S;

/**
 *
 * Result and event of libnfc operation
 *
 */
typedef struct {
    int event;                     /**<[out] Event type */
    unsigned char  *read_ndef;     /**<[out] Pointer to recieved NDEF data */
    unsigned short read_ndef_size; /**<[out] Actual size of recieved NDEF data */
    int status;                    /**<[out] Status of operation */
} RESULT_INFO;


/**
 *
 * Handover related information used in NF_EncodeNdefData
 *
 */
typedef struct {
    E_ENCORD_TYPE   encord_type; /**<[in] specify type of NDEF for Bluetooth (Classic) or BLE */
    unsigned char   bt_dev_addr[BT_DEV_ADDR_LENGTH]; /**<[in] Device Address */
    unsigned char   conf_len;                        /**<[in] Length of conf_data */
    unsigned char   *conf_data;                      /**<[in] Pointer to optional Configuration Data */
} ENCODE_INFO;


/**
*@name Callback Functions
*@{
*/

/**
 * @brief Callback function passes status and event of libnfc operation
 *
 
 * @param[out]  data: Pointer to the RESULT_INFO buffer given by applicaton when calling NF_Start
 *
 * @retval unused
 */
typedef int (*FUNC_CB)(void *data);

/** @} */


/**
 * @brief Start NFC discovery operarion, and wait for peer NFC device in background task until calling NF_End
 *
 *
 * @param[in]   ndef_data: Pointer to ndef data to be sent to peer NFC device
 * @param[in]   ndef_size: Size of ndef data
 * @param[in]   reader_hce_mode: Mode of operation, which specify the HCE(Host Card Emulation) mode and / or Reader mode
 * @param[out]  buff: Pointer to buffer for recieved ndef data
 * @param[in]   buff_size: Size of buff
 * @param[in]   funcCb: Address of callback function
 * @param[out]  result_info: Pointer to results returned by funcCb
 *
 * @retval D_NF_SUCCESS : success
 * @retval D_NF_PARAM_ERR : parameter error
 * @retval D_NF_DUPL_ERR : duplicate error
 * @retval D_NF_PROC_ERR : proccess error
 */

extern int  NF_Start(UCHAR *ndef_data, USHORT ndef_size, SINT reader_hce_mode, UCHAR *buff, USHORT buff_size, FUNC_CB funcCb, RESULT_INFO *result_info);


/**
 * @brief Finish NFC discovery operation
 *
 *
 * @param   none
 *
 * @retval D_NF_SUCCESS : success
 */

extern int  NF_End(void);


/**
 * @brief Parse NDEF data received from peer NFC device and retrieve OOB(Out-Of-Band) data.
 * @brief The utility function helps Bluetooth static handover to get BD_ADDR of peer Bluetooth device.
 *
 *
 * @param[out]  ho_info: Pointer to buffer for parsed OOB data
 * @param[in]   ndef_data: Pointer to ndef data to be parsed
 * @param[in]   ndef_size: Size of ndef data
 *
 * @retval D_NF_SUCCESS : success
 * @retval D_NF_PARAM_ERR : parameter error
 * @retval D_NF_PROC_ERR : proccess error
 */

extern int  NF_ParseNdefData(HO_INFO_S *ho_info, UCHAR *ndef_data, USHORT ndef_size);


/**
 * @brief Generate NDEF data from ENCODE_INFO
 * @brief The utility function helps Bluetooth static handover to pass own BD_ADDR
 *
 *
 * @param[in]  encode_info: Pointer to handover related information
 * @param[in]   buff_size: Size of buff
 * @param[out]   buff: Pointer to buffer for generated NDEF data
 * @param[out]   en_ndef_size: Size of generated NDEF data
 *
 * @retval D_NF_SUCCESS : success
 * @retval D_NF_PARAM_ERR : parameter error
 * @retval D_NF_PROC_ERR : proccess error
 */

extern int  NF_EncodeNdefData(ENCODE_INFO *encode_info, USHORT buff_size, UCHAR *buff, USHORT *en_ndef_size);

extern void NF_ParsePrint(HO_INFO_S *ho_info);

#ifdef __cplusplus
}
#endif /* __cplusplus */

/** @} NFC */

#endif /* _LIBNFC_API_H_ */
