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
 * @file       lte_if_util.h
 * @brief      LTE interface for LTE utility definition header file for SDK API
 * @author     Masatoshi.Ueno@jp.sony.com
 */

#ifndef LTE_IF_UTIL_H__
#define LTE_IF_UTIL_H__

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
 * @defgroup lteifutl LTE System Utility
 * LTE System Utility interface for SDK API
 * @{
 */

/** 
 * @defgroup lteutlsize Buffer size
 * Buffer size for each structure
 * @{
 */
/* Buffer size */
#define LTE_SIZE_COMMAND_UTIL_NUMBER	16	/**< Buffer size of LTECommand_UtilNumber */
#define LTE_SIZE_COMMAND_UTIL_CLOCK	24	/**< Buffer size of LTECommand_UtilClock */
#define LTE_SIZE_COMMAND_UTIL_OPERATE	32	/**< Buffer size of LTECommand_UtilRegister */
/**@}*/


/** 
 * @defgroup lteutlcomcode Command code
 * Command code for code value of LTECommandHeader
 * @{
 */
/* Utility I/F command */
#define LTE_COMMAND_UTIL_VERSION	0x5001	/**< Get version information */
#define LTE_COMMAND_UTIL_NUMBER		0x5002	/**< Get phone number */
#define LTE_COMMAND_UTIL_REPORT		0x5003	/**< Get error report */
#define LTE_COMMAND_UTIL_QUALITY	0x5004	/**< Get signal quality */
#define LTE_COMMAND_UTIL_REGISTER	0x5005	/**< Get registration information */
#define LTE_COMMAND_UTIL_CLOCK		0x5006	/**< Get real time clock information */

#define LTE_COMMAND_UTIL_GET_DATA	0x5011	/**< Get data configuration information */
#define LTE_COMMAND_UTIL_SET_DATA	0x5012	/**< Set data configuration information */

#define LTE_COMMAND_UTIL_ATCOMMAND	0x50FF	/**< Send and receive AT command */
/**@}*/


/** 
 * @defgroup lteUtlcomtype Manager type of command code
 * Manager type bitmask for @ref lteutlcomcode "command code"
 * @{
 */
#define LTE_COMMAND_TYPE_UTIL		0x5000	/**< Bitmask for System Utility */
/**@}*/


/**
 * @defgroup lteutlstrtype Structure type of command header
 * - Structure type for type value of LTECommandHeader
 * - Details of each structure shows at @ref lteutlstr "Strutures for System Utility"
 * @{
 */
#define LTE_STRUCT_TYPE_UTIL_NUMBER	0x5001	/**< Structure for phone number */
#define LTE_STRUCT_TYPE_UTIL_QUALITY	0x5002	/**< Structure for signal quality */
#define LTE_STRUCT_TYPE_UTIL_REGISTER	0x5003	/**< Structure for registration information */
#define LTE_STRUCT_TYPE_UTIL_CLOCK	0x5004	/**< Structure for real time clock */
#define LTE_STRUCT_TYPE_UTIL_DATA	0x5011	/**< Structure for data configuration */
/**@}*/


/**
 * @defgroup lteutlflgsig Flag of signal quality
 * Information flag of signal quality of LTECommand_UtilQuality 
 * @{
 */
#define LTE_FLAG_QUALITY_DEFAULT	0x0000	/**< Default information (RSSI/BER) */
#define LTE_FLAG_QUALITY_ALL		0x0001	/**< All information (RSSI/BER/RSRP/RSRQ/SINR/RSSNR/CQI) */
/**@}*/


/**
 * @defgroup lteutlcalldef Typedef for callback function
 * Typedef for common callback function
 * @{
 */
/* Data structure */
typedef void (* LTECallbackFunctionUtil)(void * data);	/**< Typedef for callback function */
/**@}*/


/**
 * @defgroup lteutlstr Structure for System Utility
 * Data structure for System Utility
 * @{
 */
/**
 * LTE phone number structure for System Utility
 */
typedef struct {
    int length;					/**< [out] Length of number */
    char number[LTE_SIZE_COMMAND_UTIL_NUMBER];	/**< [out] Phone number */
    int type;					/**< [out] Number type (3GPP TS 24.008) */
} LTECommand_UtilNumber;

/**
 * LTE RF signal quality structure for System Utility
 */
typedef struct {
    int flag;			/**< [in] @ref lteutlflgsig "Detailed information flag" */
    int rssi;			/**< [out] Received Signal Strength Indicator (0-31,99) */
    int ber;			/**< [out] Bit Error Rate (0-7,99) */
    int rsrp;			/**< [out] Reference Signal Received Power (-140-0) */
    int rsrq;			/**< [out] Reference Signal Received Quality (-64-0) */
    int sinr;			/**< [out] Signal to Interface plus Noise Ratio (-12-40) */
    int rssnr;			/**< [out] Reference signal Signal to Noise Ratio (-120-400) */
    int cqi;			/**< [out] Channel Quality Indicator (0-15) */
} LTECommand_UtilQuality;

/**
 * LTE register information structure for System Utility
 */
typedef struct {
    int reg;			/**< [out] Network registration status */
    int area;			/**< [out] Location/Tracking area code (2bytes hexadecimal) */
    int cell;			/**< [out] GERAN/UTRAN/E-UTRAN cell ID (4bytes hexadecimal) */
    int technology;		/**< [out] Access technology (3GPP TS 27.007) (7:E-UTRAN) */
    int mode;			/**< [out] Network registration mode (0:automatic, 1:manual) */
    int format;			/**< [out] Network operator format (0:long, 1:short) */
    int operate_length;					/**< [out] Length of operate */
    char operate[LTE_SIZE_COMMAND_UTIL_OPERATE];	/**< [out] Network operator name */
    int ims;			/**< [out] IMS registeration status (0:not register, 1:register) */
} LTECommand_UtilRegister;

/**
 * LTE real time clock information structure for System Utility
 */
typedef struct {
    int time_length;				/**< [out] Length of time */
    char time[LTE_SIZE_COMMAND_UTIL_CLOCK];	/**< [out] Real time clock (yy/MM/dd,hh:mm:ss+zz) */
} LTECommand_UtilClock;

/**
 * LTE data configuration structure for System Utility
 */
typedef struct {
    int disable_data;		/**< [in,out] Disable data traffic on home and roaming*/
    int disable_data_roaming;	/**< [in,out] Disable data traffic on roaming */
    int time_sync;		/**< [in,out] Auto time synchronization */
} LTECommand_UtilData;
/**@}*/


/**
 * @defgroup lteutlcomstr Structure for common interface
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
        /* for System utility */
        LTECommand_Error		error;		/**< [out] Error message structure */
        LTECommand_Message		text;		/**< [out] text message structure */
        LTECommand_Message		version;	/**< [out] Version message structure */
        LTECommand_UtilNumber		number;		/**< [out] Phone number structure */
        LTECommand_UtilQuality		quality;	/**< [out] Signal quality structure */
        LTECommand_UtilRegister		reg;		/**< [out] Registration information structure */
        LTECommand_UtilClock		clock;		/**< [out] Real time clock structure */
        LTECommand_UtilData		data;		/**< [in,out] Data configuration structure */
        LTECommand_Message		ATcommand;	/**< [in,out] AT command structure */
    } message;						/**< [in,out] Command message union */
} LTECommand_Util;
/**@}*/


/**
 * @defgroup lteutlcomapi Application Progrming Interface
 * Application programing interface for LTE manager
 * @{
 */
/**
 * LTE command send API for system utility
 * @warning This command sends packets to LTE modem device and don't wait to receive response.
 * @warning Response from LTE modem device should be received by using callback interface.
 * @param command : Command structure
 * @retval command.header.result : Result code
 */
extern void LT_SendUtil( LTECommand_Util* command );

/**
 * LTE callback setup command for System Utility
 * @warning To call this command is needed if use System Utility
 * @param callback : Callback function
 * @param command : Command structure buffer for response (LTECommand_Util)
 */
extern void LT_SetCallbackUtil( LTECallbackFunctionUtil callback, void* command );
/**@}*/

/**@}*/

#ifdef __cplusplus
}
#endif

#endif
