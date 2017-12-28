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
 * @file nfc_fe_api.h
 * @brief NFC(Near Field Communication) Sequencer API.
 *
 * To use NFC Sequencer features,  nf_api.h needs to be included in source code as following.
 * <pre>#include	<nfc/nf_api.h></pre>
 */


#ifndef _NFC_FE_API_H_
#define _NFC_FE_API_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @defgroup nfc_sequencer NFC Sequencer
 * @brief Applicaton Interface provided by NFC Sequencer Library
 *
 * To use NFC Sequencer features,  nf_api.h needs to be included in source code as following.
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

/* for felica */
#define NF_START_MODE_RF   0x01 /**< Operation mode(RF in NF_Listen_Start) for Felica */
#define NF_START_MODE_WIRE 0x02 /**< Operation mode(WIRE in NF_Listen_Start) for Felica */
#define D_TECH_MASK_TYPE_A 0x01 /**< Operation type(Listen type A in NF_Listen_Start) for Felica */
#define D_TECH_MASK_TYPE_B 0x02 /**< Operation type(Listen type B in NF_Listen_Start) for Felica */
#define D_TECH_MASK_TYPE_F 0x04 /**< Operation type(Listen type F in NF_Listen_Start) for Felica */

/* api return code */
#define D_NF_SUCCESS    0  /**< success in NF_xxx return code */
#define D_NF_PARAM_ERR -1  /**< parameter error in NF_xxx return code */
#define D_NF_DUPL_ERR  -2  /**< duplicate error in NF_xxx return code */
#define D_NF_PROC_ERR  -3  /**< proccess error in NF_xxx return code */
/* for felica */
#define D_NF_NOT_START -4  /**< not start error in NF_xxx return code */
#define D_NF_UNSUPPORT -10 /**< unsupport error in NF_xxx return code */

#define D_NFC_SUCCESS        0x00 /**< Success status in RECV_INFO */
#define D_NFC_FAILED         0xFF /**< Failed status in RECV_INFO */

/**
 *
 * Result and event of libnfc operation
 *
 */
typedef struct {
    int status;                    /**<[out] Status of operation */
    int type;                      /**<[out] type of listen */
    unsigned char  *recv_data;     /**<[out] Pointer to recieved data */
    unsigned short  recvsize;      /**<[out] Actual size of recieved data */
} RECV_INFO;

/**
*@name Callback Functions
*@{
*/

/**
 * @brief Callback function passes status and event of nfc felica operation
 *
 
 * @param[in]  status: Status of operation
 * @param[in]  type: type of technology
 * @param[in]  recv_data: Pointer to recieved data
 * @param[in]  recv_size: Size of recieved data
 *
 * @retval unused
 */
typedef int (*FUNC_FE_CB)(int status, int type, UCHAR *recv_data, USHORT recv_size);

/** @} */

/**
 * @brief Start NFC Felica operation
 *
 *
 * @param[in]   mode: Mode of operation, which specify wired and rf communication mode
 * @param[in]   funcCb: Address of callback function
 * @param[in]   type: type of technology
 *
 * @retval D_NF_SUCCESS : success
 * @retval D_NF_PARAM_ERR : parameter error
 * @retval D_NF_DUPL_ERR : duplicate error
 * @retval D_NF_PROC_ERR : proccess error
 * @retval D_NF_UNSUPPORT : unsupport error
 */

extern int NF_Listen_Start(SINT mode, FUNC_FE_CB funcCb, SINT type);

/**
 * @brief Finish NFC Felica operation
 *
 *
 * @param   none
 *
 * @retval D_NF_SUCCESS : success
 * @retval D_NF_PROC_ERR : proccess error
 */

extern int NF_Listen_End(void);

/**
 * @brief Open wire connection operation
 *
 *
 * @param   none
 *
 * @retval D_NF_SUCCESS : success
 * @retval D_NF_PROC_ERR : proccess error
 * @retval D_NF_NOT_START : not NF_Listen_Start error
 */

extern int NF_Felica_Open(void);

/**
 * @brief Close wire connection operation
 *
 *
 * @param   none
 *
 * @retval D_NF_SUCCESS : success
 * @retval D_NF_PROC_ERR : proccess error
 * @retval D_NF_NOT_START : not NF_Listen_Start error
 */

extern int NF_Felica_Close(void);

/**
 * @brief Send wire connection operation
 *
 *
 * @param[in]   send_data: Pointer to send data
 * @param[in]   send_size: Size of send data
 *
 * @retval D_NF_SUCCESS : success
 * @retval D_NF_PARAM_ERR : parameter error
 * @retval D_NF_PROC_ERR : proccess error
 * @retval D_NF_NOT_START : not NF_Listen_Start error
 */

extern int NF_Felica_Send(UCHAR *send_data, USHORT send_size);

/**
 * @brief Start NFC FW Update operation
 *
 * @param   none
 *
 * @retval D_NF_SUCCESS : success
 * @retval D_NF_DUPL_ERR : duplicate error
 * @retval D_NF_PROC_ERR : proccess error
 */

extern int NF_Fw_Update(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

/** @} NFC */

#endif /* _NFC_FE_API_H_ */
