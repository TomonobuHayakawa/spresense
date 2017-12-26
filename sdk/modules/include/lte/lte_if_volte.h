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
 * @file       lte_if_volte.h
 * @brief      LTE interface for VoLTE definition header file for SDK API
 * @author     Masatoshi.Ueno@jp.sony.com
 */

#ifndef LTE_IF_VOLTE_H__
#define LTE_IF_VOLTE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "ril/lte_if_system.h"

/* Definition */

/**
 * @defgroup lteiftop Telephony
 * LTE interface for SDK API
 */

/**
 * @ingroup lteiftop
 * @defgroup lteifvol LTE VoLTE Manager
 * LTE VoLTE Manager interface for SDK API
 * @{
 */

/** 
 * @defgroup ltevoltesize Buffer size
 * Buffer size for each structure
 * @{
 */
/* Buffer size */
#define LTE_SIZE_COMMAND_VOLTE_STATUS	24	/**< Buffer size of LTECommand_VolteResponse */
#define LTE_SIZE_COMMAND_VOLTE_NUMBER	16	/**< Buffer size of LTECommand_VolteCurrent */
#define LTE_SIZE_COMMAND_VOLTE_SIP	64	/**< Buffer size of LTECommand_VolteCall (sip) */
#define LTE_SIZE_COMMAND_VOLTE_TEL	16	/**< Buffer size of LTECommand_VolteCall (tel) */
/**@}*/


/** 
 * @defgroup ltevoltecomcode Command code
 * Command code for code value of LTECommandHeader
 * @{
 */
/* VoLTE I/F command */
#define LTE_COMMAND_VOLTE_CONFIG	0x3001	/**< Configure modem device for VoLTE */
#define LTE_COMMAND_VOLTE_CALL		0x3002	/**< Call number */
#define LTE_COMMAND_VOLTE_ANSWER	0x3003	/**< Answer to incoming call */
#define LTE_COMMAND_VOLTE_HOOK		0x3004	/**< Hook active call */
#define LTE_COMMAND_VOLTE_SERVICE	0x3005	/**< Control party call */
#define LTE_COMMAND_VOLTE_STATUS	0x3006	/**< Get call information */

#define LTE_COMMAND_VOLTE_RING		0x3011	/**< Incoming call information (unsolicited) */
#define LTE_COMMAND_VOLTE_NO_CARRIER	0x3012	/**< Hooked call information (unsolicited) */
#define LTE_COMMAND_VOLTE_UPDATE	0x3013	/**< Updated call information (unsolicited) */
/**@}*/


/** 
 * @defgroup ltevoltecomtype Manager type of command code
 * Manager type bitmask for @ref ltevoltecomcode "command code"
 * @{
 */
#define LTE_COMMAND_TYPE_VOLTE		0x3000	/**< Bitmask for VoLTE Manager */
/**@}*/


/**
 * @defgroup ltevoltestrtype Structure type of command header
 * - Structure type for type value of LTECommandHeader
 * - Details of each structure shows at @ref ltevoltestr "Strutures for VoLTE"
 * @{
 */
#define LTE_STRUCT_TYPE_VOLTE_CALL	0x3001	/**< Structure for VoLTE call */
#define LTE_STRUCT_TYPE_VOLTE_RESPONSE	0x3002	/**< Structure for VoLTE response */
#define LTE_STRUCT_TYPE_VOLTE_SERVICE	0x3003	/**< Structure for VoLTE party call */
#define LTE_STRUCT_TYPE_VOLTE_CURRENT	0x3004	/**< Structure for VoLTE information */
#define LTE_STRUCT_TYPE_VOLTE_NUMBER	0x3005	/**< Structure for VoLTE number information */
/**@}*/


/**
 * @defgroup ltevolteflgvolte VoLTE number select
 * VoLTE number select (TEL or SIP) for flag value of LTECommand_VolteCall
 * @{
 */
#define LTE_FLAG_VOLTE_DEFAULT		0x0000	/**< Default (not use) */
#define LTE_FLAG_VOLTE_TEL		0x0000	/**< Use TELephone number */
#define LTE_FLAG_VOLTE_SIP		0x0001	/**< Use Session Initiation Protocol address */
/**@}*/


/**
 * @defgroup ltevoltecalldef Typedef for callback function
 * Typedef for common callback function
 * @{
 */
/* Data structure */
typedef void (* LTECallbackFunctionVolte)(void * data);	/**< Typedef for callback function */
/**@}*/


/**
 * @defgroup ltevoltestr Structure for VoLTE Manager
 * Data structure for VoLTE Manager
 * @{
 */
/**
 * LTE VoLTE call structure for VoLTE Manager
 */
typedef struct {
    int flag;					/**< [in] @ref ltevolteflgvolte "Dial flag" */
    int tel_length;				/**< [in] Length of phone number */
    char tel[LTE_SIZE_COMMAND_VOLTE_TEL];	/**< [in] Phone number */
    int sip_length;				/**< [in] Length of sip address */
    char sip[LTE_SIZE_COMMAND_VOLTE_SIP];	/**< [in] SIP address */
    int index;					/**< [out] Call index */
} LTECommand_VolteCall;

/**
 * LTE VoLTE call response structure for VoLTE Manager
 */
typedef struct {
    int index;					/**< [out] Call index */
    int length;					/**< [out] Length of call status */
    char status[LTE_SIZE_COMMAND_VOLTE_STATUS];	/**< [out] Call status */
} LTECommand_VolteResponse;

/**
 * LTE VoLTE phone number structure for VoLTE Manager
 */
typedef struct {
    int length;					/**< [out] Length of number */
    char number[LTE_SIZE_COMMAND_VOLTE_NUMBER];	/**< [out] Phone number */
    int type;					/**< [out] Number type (3GPP TS 24.008) */
} LTECommand_VolteNumber;

/**
 * LTE VoLTE call control structure for VoLTE Manager
 */
typedef struct {
    int code;					/**< [in] Call control code (3GPP TS 22.030) */
} LTECommand_VolteService;

/**
 * LTE VoLTE call status structure for VoLTE Manager
 */
typedef struct {
    int index;					/**< [in] Call index */
    int dir;					/**< [out] Call direction */
    int status;					/**< [out] Call status */
    int mode;					/**< [out] Call mode */
    int multiparty;				/**< [out] Multi-party flag */
    int length;					/**< [out] Length of phone number */
    char number[LTE_SIZE_COMMAND_VOLTE_NUMBER];	/**< [out] Phone number */
    int type;					/**< [out] Number type (3GPP TS 24.008) */
} LTECommand_VolteCurrent;
/**@}*/


/**
 * @defgroup ltevoltecomstr Structure for common interface
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
        /* for VoLTE manager */
        LTECommand_Error		error;		/**< [out] Error message structure */
        LTECommand_Message              text;           /**< [out] text message structure */
        LTECommand_VolteCall		call;		/**< [in,out] VoLTE call structure */
        LTECommand_VolteResponse	response;	/**< [out] VoLTE response structure */
        LTECommand_VolteNumber          number;         /**< [out] Phone number structure */
        LTECommand_VolteService		service;	/**< [in] VoLTE party call structure */
        LTECommand_VolteCurrent		current;	/**< [in,out] VoLTE status structure */
    } message;						/**< [in,out] Command message union */
} LTECommand_Volte;
/**@}*/


/**
 * @defgroup ltevoltecomapi Application Progrming Interface
 * Application programing interface for LTE manager
 * @{
 */
/**
 * LTE command send API for VoLTE manager
 * @warning This command sends packets to LTE modem device and don't wait to receive response.
 * @warning Response from LTE modem device should be received by using callback interface.
 * @param command : Command structure
 * @retval command.header.result : Result code
 */
extern void LT_SendVolte( LTECommand_Volte* command );

/**
 * LTE callback setup command for VoLTE Manager
 * @warning To call this command is needed if use VoLTE Manager
 * @param callback : Callback function
 * @param command : Command structure buffer for response (LTECommand_Volte)
 */
extern void LT_SetCallbackVolte( LTECallbackFunctionVolte callback, void* command );
/**@}*/

/**@}*/

#ifdef __cplusplus
}
#endif

#endif
