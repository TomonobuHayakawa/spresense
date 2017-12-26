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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
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
 * @file       lte_if_sms.h
 * @brief      LTE interface for SMS definition header file for SDK API
 * @author     Masatoshi.Ueno@jp.sony.com
 */

#ifndef LTE_IF_SMS_H__
#define LTE_IF_SMS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "lte_if_system.h"

/* Definition */

/**
 * @defgroup lteiftop Telephony
 * LTE interface for SDK API
 */

/**
 * @ingroup lteiftop
 * @defgroup lteifsms LTE SMS Manager
 * LTE SMS Manager interface for SDK API
 * @{
 */

/** 
 * @defgroup ltesmssize Buffer size
 * Buffer size for each structure
 * @{
 */
/* Buffer size */
#define LTE_SIZE_COMMAND_SMS_STATUS	16	/**< Buffer size of LTECommand_SMSRecv (status) */
#define LTE_SIZE_COMMAND_SMS_NUMBER	48	/**< Buffer size of LTECommand_SMSRecv (number) */
#define LTE_SIZE_COMMAND_SMS_TEXT	16	/**< Buffer size of LTECommand_SMSRecv (text) */
#define LTE_SIZE_COMMAND_SMS_TIME	24	/**< Buffer size of LTECommand_SMSRecv (time) */
#define LTE_SIZE_COMMAND_SMS_MESSAGE	160	/**< Buffer size of LTECommand_SMSRecv (message) */
#define LTE_SIZE_COMMAND_SMS_MEMORY	4	/**< Buffer size of LTECommand_SMSReport */
/**@}*/


/** 
 * @defgroup ltesmscomcode Command code
 * Command code for code value of LTECommandHeader
 * @{
 */
/* SMS I/F command */
#define LTE_COMMAND_SMS_CONFIG		0x2001	/**< Configure modem device for SMS */
#define LTE_COMMAND_SMS_GET_MEMORY	0x2002	/**< Get use memory select for SMS */
#define LTE_COMMAND_SMS_SET_MEMORY	0x2003	/**< Set use memory select for SMS */

#define LTE_COMMAND_SMS_LIST		0x2011	/**< Get message list from SIM */
#define LTE_COMMAND_SMS_READ		0x2012	/**< Get one message from SIM */
#define LTE_COMMAND_SMS_WRITE		0x2013	/**< Put one message to SIM */
#define LTE_COMMAND_SMS_SEND		0x2014	/**< Send one message from SIM */
#define LTE_COMMAND_SMS_DIRECT		0x2015	/**< Send one message */
#define LTE_COMMAND_SMS_DELETE		0x2016	/**< Delete one or more messages */

#define LTE_COMMAND_SMS_RECV		0x2021	/**< Receive one message (unsolicited) */
#define LTE_COMMAND_SMS_SEND_ACK	0x2022	/**< Receive send ack (unsolicited) */
/**@}*/


/** 
 * @defgroup ltesmscomtype Manager type of command code
 * Manager type bitmask for @ref ltesmscomcode "command code"
 * @{
 */
#define LTE_COMMAND_TYPE_SMS		0x2000	/**< Bitmask for SMS Manager */
/**@}*/


/**
 * @defgroup ltesmsstrtype Structure type of command header
 * - Structure type for type value of LTECommandHeader
 * - Details of each structure shows at @ref ltesmsstr "Strutures for SMS"
 * @{
 */
#define LTE_STRUCT_TYPE_SMS_MEMORY	0x2001	/**< Structure for SMS memory select */
#define LTE_STRUCT_TYPE_SMS_RECV	0x2011	/**< Structure for SMS receive */
#define LTE_STRUCT_TYPE_SMS_SEND	0x2012	/**< Structure for SMS send */
#define LTE_STRUCT_TYPE_SMS_DELETE	0x2013	/**< Structure for SMS message delete */
#define LTE_STRUCT_TYPE_SMS_REPORT	0x2014	/**< Structure for SMS report message */
/**@}*/


/**
 * @defgroup ltesmsflgsms SMS message encoding
 * SMS encoding command for ucs2_flag value of LTECommand_SMSSend
 * @{
 */
#define LTE_FLAG_SMS_DEFAULT		0x0000	/**< Default (not use) */
#define LTE_FLAG_SMS_8859		0x0000	/**< Use 8859-1 encoding for SMS message */
#define LTE_FLAG_SMS_UCS2		0x0001	/**< Use UCS2 encoding for SMS message */
/**@}*/


/**
 * @defgroup ltesmstxt SMS text group 
 * Text message for SMS
 * @{
 */
#define LTE_SMS_TEXT_NO_MESSAGE		"NO MESSAGE"	/**< No message is existed */
/**@}*/


/**
 * @defgroup ltesmscalldef Typedef for callback function
 * Typedef for common callback function
 * @{
 */
/* Data structure */
typedef void (* LTECallbackFunctionSms)(void * data);	/**< Typedef for callback function */
/**@}*/


/**
 * @defgroup ltesmsstr Structure for SMS Manager
 * Data structure for SMS Manager
 * @{
 */
/**
 * LTE SMS memory select information structure for SMS Manager
 */
typedef struct {
    int memory_length;				/**< [in,out] Length of memory */
    char memory[LTE_SIZE_COMMAND_SMS_MEMORY];	/**< [in,out] Memory name ("ME" or "SM") */
    int used;					/**< [out] Used memory storage count */
    int total;					/**< [out] Total memory storage count */
} LTECommand_SmsMemory;

/**
 * LTE SMS read structure for SMS Manager
 */
typedef struct {
    int index;					/**< [in,out] Index in memory */
    int status_length;				/**< [out] Length of status */
    char status[LTE_SIZE_COMMAND_SMS_STATUS];	/**< [out] Send/receive status */
    int deliver;				/**< [out] Deliver/submit flag (3GPP TS 23.040) */
    int reference;				/**< [out] Message reference (3GPP TS 23.040) */
    int number_length;				/**< [out] Length of number */
    char number[LTE_SIZE_COMMAND_SMS_NUMBER];	/**< [out] SMS address number */
    int type;					/**< [out] Number type (3GPP TS 24.008) */
    int text_length;				/**< [out] Length of text */
    char text[LTE_SIZE_COMMAND_SMS_TEXT];	/**< [out] Number text in address book */
    int ctime_length;				/**< [out] Length of ctime */
    char ctime[LTE_SIZE_COMMAND_SMS_TIME];	/**< [out] Center time */
                                                /**<       (yy/MM/dd,hh:mm:ss+zz) */
    int dtime_length;				/**< [out] Length of dtime */
    char dtime[LTE_SIZE_COMMAND_SMS_TIME];	/**< [out] Deliver time */
                                                /**<       (yy/MM/dd,hh:mm:ss+zz) */
    int report;					/**< [out] Status report (3GPP TS 23.040) */
    int message_length;				/**< [out] Length of message */
    char message[LTE_SIZE_COMMAND_SMS_MESSAGE];	/**< [out] SMS message */
} LTECommand_SmsRecv;

/**
 * LTE SMS write structure for SMS Manager
 */
typedef struct {
    int index;					/**< [in] Index in memory */
    int ucs2_flag;				/**< [in] @ref ltesmsflgsms "Encoding flag" */
    int number_length;				/**< [in] Length of number */
    char number[LTE_SIZE_COMMAND_SMS_NUMBER];	/**< [in] SMS address number */
    int message_length;				/**< [in] Length of message */
    char message[LTE_SIZE_COMMAND_SMS_MESSAGE];	/**< [in] SMS message */
    int time_length;				/**< [out] Length of time */
    char time[LTE_SIZE_COMMAND_SMS_TIME];	/**< [out] Send time */
} LTECommand_SmsSend;

/**
 * LTE SMS delete structure for SMS Manager
 */
typedef struct {
    int index;					/**< [in] Index in memory */
    int flag;					/**< [in] Index flag (3GPP TS 27.005) */
} LTECommand_SmsDelete;

/**
 * LTE SMS report information structure for SMS Manager
 */
typedef struct {
    int index;					/**< [out] Index in memory */
    int memory_length;				/**< [out] Length of memory */
    char memory[LTE_SIZE_COMMAND_SMS_MEMORY];	/**< [out] Memory name ("ME" or "SM") */
} LTECommand_SmsReport;
/**@}*/


/**
 * @defgroup ltesmscomstr Structure for common interface
 * Data structure for common interface
 * @{
 */
/**
 * LTE command structure for all commands 
 * @warning "message" value is defined as union and is needed to switch structures for each purpose.
 * @warning The kind of union should be defined in type value of LTECommandHeader.
 */
typedef struct {
    LTECommandHeader			header;		/**< [in,out] command header structure */
    union {
        /* for SMS manager */
        LTECommand_Error		error;		/**< [out] Error message structure */
        LTECommand_Message		text;		/**< [out] text message structure */
        LTECommand_SmsMemory		memory;		/**< [in,out] memory select structure */
        LTECommand_SmsRecv		recv;		/**< [in,out] SMS receive structure */
        LTECommand_SmsSend		send;		/**< [in,out] SMS send structure */
        LTECommand_SmsDelete		del;		/**< [in] SMS delete structure */
        LTECommand_SmsReport		report;		/**< [out] SMS report structure */
    } message;						/**< [in,out] Command message union */
} LTECommand_Sms;
/**@}*/


/**
 * @defgroup ltesmscomapi Application Progrming Interface
 * Application programing interface for LTE manager
 * @{
 */
/**
 * LTE SMS command send API for SMS manager
 * @warning This command sends packets to LTE modem device and don't wait to receive response.
 * @warning Response from LTE modem device should be received by using callback interface.
 * @param command : Command structure
 * @retval command.header.result : Result code
 */
extern void LT_SendSms( LTECommand_Sms* command );

/**
 * LTE callback setup command for SMS Manager
 * @warning To call this command is needed if use SMS Manager
 * @param callback : Callback function
 * @param command : Command structure buffer for response (LTECommand_Sms)
 */
extern void LT_SetCallbackSms( LTECallbackFunctionSms callback, void* command );
/**@}*/

/**@}*/

#ifdef __cplusplus
}
#endif

#endif
