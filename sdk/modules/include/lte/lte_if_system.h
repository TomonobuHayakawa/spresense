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
 * @file       lte_if_system.h
 * @brief      LTE interface for LTE manager definition header file for SDK API
 * @author     Masatoshi.Ueno@jp.sony.com
 */

#ifndef LTE_IF_SYSTEM_H__
#define LTE_IF_SYSTEM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Definition */

/**
 * @defgroup lteiftop Telephony
 * LTE interface for SDK API
 */

/**
 * @ingroup lteiftop
 * @defgroup lteifsys LTE System Manager
 * LTE System Manager interface for SDK API
 * @{
 */

/** 
 * @defgroup ltesyssize Buffer size
 * Buffer size for each structure
 * @{
 */
/* Buffer size */
#define LTE_SIZE_COMMAND_BUFFER		540	/**< Buffer size of LTECommand_Message */
#define LTE_SIZE_COMMAND_ERROR_GROUP	16	/**< Buffer size of LTECommand_Error */
#define LTE_SIZE_COMMAND_PIN		20	/**< Buffer size of LTECommand_PinAuth */
#define LTE_SIZE_COMMAND_PIN_STATUS	16	/**< Buffer size of LTECommand_PinInfo */
#define LTE_SIZE_COMMAND_EVENT_NAME	16	/**< Buffer size of LTECommand_EventInfo (name) */
#define LTE_SIZE_COMMAND_EVENT_PARAM	64	/**< Buffer size of LTECommand_EventInfo (param) */
#define LTE_SIZE_COMMAND_AID		32	/**< Buffer size of LTECommand_ANotify (id) */
#define LTE_SIZE_COMMAND_ANAME		32	/**< Buffer size of LTECommand_ANotify (name) */
/**@}*/


/** 
 * @defgroup ltesyscomcode Command code
 * Command code for code value of LTECommandHeader
 * @{
 */
/* System I/F command */
#define LTE_COMMAND_SYSTEM_CONNECT	0x1001	/**< Connect to LTE network */
#define LTE_COMMAND_SYSTEM_CONFIRM	0x1002	/**< Confirm LTE connection */
#define LTE_COMMAND_SYSTEM_DISCONNECT	0x1003	/**< Disconnect from LTE network */
#define LTE_COMMAND_SYSTEM_SLEEP	0x1004	/**< Sleep LTE modem */
#define LTE_COMMAND_SYSTEM_ACQUIRE_WAKELOCK	0x1005	/**< Acquire LTE modem wakelock */
#define LTE_COMMAND_SYSTEM_RELEASE_WAKELOCK	0x1006	/**< Release LTE modem wakelock */
#define LTE_COMMAND_SYSTEM_WAKELOCK_STATE	0x1007	/**< Get LTE modem wakelock state */

#define LTE_COMMAND_SYSTEM_INIT		0x1011	/**< Init LTE connection (before AUTH) */
#define LTE_COMMAND_SYSTEM_PIN		0x1012	/**< Get PIN information from SIM */
#define LTE_COMMAND_SYSTEM_AUTH		0x1013	/**< PIN authentication with SIM */
#define LTE_COMMAND_SYSTEM_ESTABLISH	0x1014	/**< Establish LTE connection (atfer AUTH) */
#define LTE_COMMAND_SYSTEM_RESET	0x1015	/**< Reset modem and status */

#define LTE_COMMAND_SYSTEM_STATE	0x1021	/**< Get modem connection state */
#define LTE_COMMAND_SYSTEM_POWER_STATE	0x1022	/**< Get modem power state */

#define LTE_COMMAND_SYSTEM_GET_EVENT	0x1111	/**< Get event information */
#define LTE_COMMAND_SYSTEM_SET_EVENT	0x1112	/**< Set event information */
#define LTE_COMMAND_SYSTEM_EVENT	0x1113	/**< Unsolicited event information */

#define LTE_COMMAND_SYSTEM_GPSNOTIFY	0x1201	/**< A-GPS location request notification */
#define LTE_COMMAND_SYSTEM_GPSALLOW	0x1202	/**< A-GPS location request allowance */
#define LTE_COMMAND_SYSTEM_GET_GPSAUTO	0x1203	/**< Get A-GPS location request auto allowance */
#define LTE_COMMAND_SYSTEM_SET_GPSAUTO	0x1204	/**< Set A-GPS location request auto allowance */
/**@}*/


/** 
 * @defgroup ltesyscomtype Manager type of command code
 * Manager type bitmask for @ref ltesyscomcode "command code"
 * @{
 */
#define LTE_COMMAND_TYPE_MASK		0xF000	/**< Bitmask for Interface type */
#define LTE_COMMAND_TYPE_SYSTEM		0x1000	/**< Bitmask for System Manager */
/**@}*/


/**
 * @defgroup ltesysstrtype Structure type of command header
 * - Structure type for type value of LTECommandHeader
 * - Details of each structure shows at @ref ltesysstr "Strutures for System"
 * @{
 */
#define LTE_STRUCT_TYPE_NONE		0x0000	/**< No message */
#define LTE_STRUCT_TYPE_MESSAGE		0x1001	/**< Structure for message */
#define LTE_STRUCT_TYPE_ERROR		0x1002	/**< Structure for error code */
#define LTE_STRUCT_TYPE_PIN_INFO	0x1011	/**< Structure for PIN information */
#define LTE_STRUCT_TYPE_PIN_AUTH	0x1012	/**< Structure for PIN authentication */
#define LTE_STRUCT_TYPE_STATE		0x1021	/**< Structure for modem connection state */
#define LTE_STRUCT_TYPE_CONFIG		0x1022	/**< Structure for connection configuration */
#define LTE_STRUCT_TYPE_POWER_STATE	0x1023	/**< Structure for modem power state */
#define LTE_STRUCT_TYPE_WAKELOCK_STATE	0x1024	/**< Structure for modem wakelock state */
#define LTE_STRUCT_TYPE_EVENT		0x1111	/**< Structure for event information */
#define LTE_STRUCT_TYPE_GPSNOTIFY	0x1201	/**< Structure for A-GPS notification */
#define LTE_STRUCT_TYPE_GPSALLOW	0x1202	/**< Structure for A-GPS allowance */
#define LTE_STRUCT_TYPE_GPSAUTO		0x1203	/**< Structure for A-GPS request auto allowance */
/**@}*/


/**
 * @defgroup ltesysflgevent Event flag
 * Event bitmask for flag value of LTECommand_EventInfo
 * @{
 */
#define LTE_FLAG_EVENT_DEFAULT		0x0000	/**< Default (not use) */
#define LTE_FLAG_EVENT_MASK		0x003F	/**< Available bit mask */
#define LTE_FLAG_EVENT_SYS_STATE	0x0001	/**< Enable system stat event information */
#define LTE_FLAG_EVENT_NET_STATE	0x0002	/**< Enable NET state event information */
#define LTE_FLAG_EVENT_IMS_STATE	0x0004	/**< Enable IMS state event information */
#define LTE_FLAG_EVENT_PDN_STATE	0x0008	/**< Enable PDN state event information */
#define LTE_FLAG_EVENT_EPS_STATE	0x0010	/**< Enable EPS state event information */
#define LTE_FLAG_EVENT_POWER_STATE	0x0020	/**< Enable modem power state event information */
#define LTE_FLAG_EVENT_GET_VALUE	0xFFFF	/**< Get curent mask value */
/**@}*/


/**
 * @defgroup ltesysflgpin PIN authentication
 * PIN authentication command for flag value of LTECommand_PinAuth
 * @{
 */
#define LTE_FLAG_PINAUTH_DEFAULT	0x0000	/**< Default (not use) */
#define LTE_FLAG_PINAUTH_DISABLE	0x0001	/**< Disable SIM lock function */
#define LTE_FLAG_PINAUTH_ENABLE		0x0002	/**< Enable SIM lock function */
#define LTE_FLAG_PINAUTH_PIN		0x0003	/**< Unlock PIN1/PIN2 */
#define LTE_FLAG_PINAUTH_CHANGE		0x0004	/**< Change PIN1/PIN2 with old PIN */
#define LTE_FLAG_PINAUTH_UNLOCK		0x0005	/**< Change PIN1/PIN2 with PUK/PUK2 */
/**@}*/


/**
 * @defgroup ltesysflgmdm Modem status
 * Initialization status for status value of LTECommand_State
 * @{
 */
#define LTE_FLAG_MODEM_DISCONNECT	0x0000	/**< Disconnected */
#define LTE_FLAG_MODEM_INITIALIZE	0x0001	/**< Initialized */
#define LTE_FLAG_MODEM_AUTHENTICATE	0x0002	/**< Pin authenticated */
#define LTE_FLAG_MODEM_ESTABLISH	0x0003	/**< Connected, configuring internally */
#define LTE_FLAG_MODEM_CONNECT		0x0004	/**< Connected */
#define LTE_FLAG_MODEM_LIMITED		0x0005	/**< Not authenticated */
/**@}*/


/**
 * @defgroup ltesysflgpwr Modem power status
 * Initialization status for power status value of LTECommand_PowerState
 * @{
 */
#define LTE_FLAG_MODEM_SLEEP		0x0000	/**< Slept */
#define LTE_FLAG_MODEM_WAKEUP		0x0001	/**< Wake up */
/**@}*/


/**
 * @defgroup ltesysgrperror Error group
 * Error group for group value of LTECommand_Error 
 * @{
 */
#define LTE_GROUP_ERROR_SYSTEM		"CME ERROR"	/**< CME error (3GPP TS 27.007) */
#define LTE_GROUP_ERROR_SMS		"CMS ERROR"	/**< CMS error (3GPP TS 27.005) */
#define LTE_GROUP_ERROR_UNSOLICITED	"UNSOLICITED"	/**< Command interrupted by unsolicited command */
/**@}*/


/**
 * @defgroup ltesysgrpevent Event group
 * Event name for event value of LTECommand_EventInfo 
 * @{
 */
#define LTE_GROUP_EVENT_SET_VALUE	"SETVALUE"	/**< Set event value event */
#define LTE_GROUP_EVENT_GET_VALUE	"GETVALUE"	/**< Get event value event */
#define LTE_GROUP_EVENT_SIM_STATE	"SIMSTATE"	/**< SIM state change event */
#define LTE_GROUP_EVENT_NET_STATE	"NETSTATE"	/**< Network state change event */
#define LTE_GROUP_EVENT_IMS_STATE	"IMSSTATE"	/**< IMS state change event */
#define LTE_GROUP_EVENT_PDN_STATE	"PDNSTATE"	/**< PDN state change event */
#define LTE_GROUP_EVENT_EPS_STATE	"EPSSTATE"	/**< EPS state change event */
#define LTE_GROUP_EVENT_PWR_STATE	"PWRSTATE"	/**< Modem power state change event */
/**@}*/


/**
 * @defgroup ltesyscalldef Typedef for callback function
 * Typedef for common callback function
 * @{
 */
/* Data structure */
typedef void (* LTECallbackFunctionSystem)(void * data);	/**< Typedef for callback function */
/**@}*/


/**
 * @defgroup ltesysstr Structure for System Manager
 * Data structure for System Manager
 * @{
 */
/**
 * LTE general message structure for System Manager
 */
typedef struct {
    int length;					/**< [in,out] Length of message */
    char data[LTE_SIZE_COMMAND_BUFFER];		/**< [in,out] Message */
} LTECommand_Message;

/**
 * LTE error message structure for System Manager
 */
typedef struct {
    int length;					/**< [out] Length of group */
    char group[LTE_SIZE_COMMAND_ERROR_GROUP];	/**< [out] @ref ltesysgrperror "Error group" */
    int code;					/**< [out] Error result code */
} LTECommand_Error;

/**
 * LTE modem connect information structure for System Manager
 */
typedef struct {
    int state;			/**< [out] @ref ltesysflgmdm "Status of LTE modem" */
} LTECommand_State;

/**
 * LTE modem power information structure for System Manager
 */
typedef struct {
    int state;			/**< [out] @ref ltesysflgpwr "Status of LTE modem power" */
} LTECommand_PowerState;

/**
 * LTE modem wakelock information structure for System Manager
 */
typedef struct {
    int state;			/**< [out] Status of LTE modem wakelock */
				/**< (0:unlocked, 0 < :locked) */
} LTECommand_WakelockState;

/**
 * LTE modem connection configuration structure for System Manager
 */
typedef struct {
    int IMSregist;		/**< [in] IMS registration flag (0:disable, 1:enable) */
} LTECommand_Config;

/**
 * LTE event information structure for System Manager
 */
typedef struct {
    int flag;					/**< [in,out] @ref ltesysflgevent "Bitmask flag" */
                                                /**< (-1:get current value, else:set value) */
    int event_length;				/**< [out] Length of event */
    char event[LTE_SIZE_COMMAND_EVENT_NAME];	/**< [out] @ref ltesysgrpevent "Event name" */
    int value;					/**< [out] Event parameter (value) */
    int param_length;				/**< [out] Length of parameter */
    char param[LTE_SIZE_COMMAND_EVENT_PARAM];	/**< [out] Event parameter (string) */
} LTECommand_Event;

/**
 * LTE pin authentication structure for System Manager
 */
typedef struct {
    int flag;					/**< [in] @ref ltesysflgpin "Command flag" */
    int pin_length;				/**< [in] Length of pin */
    char pin[LTE_SIZE_COMMAND_PIN];		/**< [in] PIN data */
    int newpin_length;				/**< [in] Length of new pin */
    char newpin[LTE_SIZE_COMMAND_PIN];		/**< [in] Update PIN data */
    int bclass;					/**< [in] Enable/disable class (bitmask) */
} LTECommand_PinAuth;

/**
 * LTE pin information structure for System Manager
 */
typedef struct {
    int pin_left;				/**< [out] PIN attempts left */
    int puk_left;				/**< [out] PUK attempts left */
    int pin2_left;				/**< [out] PIN2 attempts left */
    int puk2_left;				/**< [out] PUK2 attempts left */
    int length;					/**< [out] Length of status */
    char status[LTE_SIZE_COMMAND_PIN_STATUS];	/**< [out] PIN authentication status */
} LTECommand_PinInfo;

/**
 * LTE A-GPS location request notification structure for System Manager
 */
typedef struct {
    int flag;					/**< [in] subscribe flag for notification */
    int handle;					/**< [out] Handle ID to distinguish request */
    int notify;					/**< [out] User's privacy information */
    int location;				/**< [out] Location type */
    int id_length;				/**< [out] Length of External client ID */
    char id[LTE_SIZE_COMMAND_AID];		/**< [out] External client ID (3GPP TS 32.299) */
    int name_length;				/**< [out] Length of External client name */
    char name[LTE_SIZE_COMMAND_ANAME];		/**< [out] External client name (3GPP TS 32.299) */
    int plane;					/**< [out] Control plane */
} LTECommand_GpsNotify;

/**
 * LTE A-GPS location request disclosure allowance structure for System Manager
 */
typedef struct {
    int allow;					/**< [in] Allowance flag */
    int handle;					/**< [in] Handle ID to distinguish request */
} LTECommand_GpsAllow;

/**
 * LTE A-GPS location request auto allowance structure for System Manager
 */
typedef struct {
    int mode;					/**< [in,out] Auto allowance flag */
                                                /**< (0:manual, 1:all OK, 2:all NG) */
} LTECommand_GpsAuto;
/**@}*/


/**
 * @defgroup ltesyscomstr Structure for common interface
 * Data structure for common interface
 * @{
 */
/**
 * LTE command header structure for all commands 
 * @warning "id" value is used to distinguish LT_Send command and suitable callback command.
 * @warning The value "-1" will be set for unsolicited command.
 */
typedef struct {
    int code;			/**< [in,out] @ref ltesyscomcode "Command code for System" or */
                                /**<          @ref lteutlcomcode "Command code for Utility" or */
                                /**<          @ref ltesmscomcode "Command code for SMS" or */
                                /**<          @ref ltevoltecomcode "Command code for VoLTE" or */
                                /**<          @ref lteagpscomcode "Command code for A-GPS" */
    int id;			/**< [in,out] unique id for command matching */
                                /**<          (-1:unsolicited, other:same as request command) */
    int result;			/**< [out] command result */
                                /**<          (0, -errno) */
    int type;			/**< [in,out] @ref ltesysstrtype "Structure type for System" or */
                                /**<          @ref lteutlstrtype "Structure type for Utility" or */
                                /**<          @ref ltesmsstrtype "Structure type for SMS" or */
                                /**<          @ref ltevoltestrtype "Structure type for VoLTE" or */
                                /**<          @ref lteagpsstrtype "Structure type for A-GPS" */
} LTECommandHeader;

/**
 * LTE command structure for all commands 
 * @warning "message" value is defined as union and is needed to switch structures for each purpose.
 * @warning The kind of union should be defined in type value of LTECommandHeader.
 */
typedef struct {
    LTECommandHeader			header;		/**< [in,out] command header structure */
    union {
        /* for System manager */
        LTECommand_Error		error;		/**< [out] Error message structure */
        LTECommand_Message		text;		/**< [out] text message structure */
        LTECommand_State		state;		/**< [out] Modem information structure */
        LTECommand_PowerState		pwrstate;	/**< [out] Modem power state information structure */
        LTECommand_WakelockState	lockstate;	/**< [out] Modem wakelock state information structure */
        LTECommand_Config		config;		/**< [in] Connection configuration structure */
        LTECommand_Event		event;		/**< [in,out] Event information structure */
        LTECommand_PinInfo		pin;		/**< [out] PIN information structure */
        LTECommand_PinAuth		auth;		/**< [in] PIN authentication structure */
        LTECommand_GpsNotify		gpsnotify;	/**< [out] A-GPS notification structure */
        LTECommand_GpsAllow		gpsallow;	/**< [in] A-GPS allowance structure */
        LTECommand_GpsAuto		gpsauto;	/**< [in,out] A-GPS auto allowance structure */
    } message;						/**< [in,out] Command message union */
} LTECommand_System;
/**@}*/


/**
 * @defgroup ltesyscomapi Application Programing Interface
 * Application programing interface for LTE manager
 * @{
 */
/**
 * LTE command send API for system manager
 * @warning This command sends packets to LTE modem device and don't wait to receive response.
 * @warning Response from LTE modem device should be received by using callback interface.
 * @param command : Command structure
 * @retval command.header.result : Result code
 */
extern void LT_SendSystem( LTECommand_System* command );

/**
 * LTE callback setup command for System Manager
 * @warning To call this command is needed if use System Manager
 * @param callback : Callback function
 * @param command : Command structure buffer for response (LTECommand_System)
 */
extern void LT_SetCallbackSystem( LTECallbackFunctionSystem callback, void* command );
/**@}*/

/**@}*/

#ifdef __cplusplus
}
#endif

#endif
