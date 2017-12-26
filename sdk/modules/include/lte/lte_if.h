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
 * @file       lte_if.h
 * @brief      LTE interface definition header file for SDK API
 * @author     Masatoshi.Ueno@jp.sony.com
 */

#ifndef LTE_IF_H__
#define LTE_IF_H__

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
 * @defgroup lteifcom LTE Common
 * LTE Common interface for SDK API
 * @{
 */

/**
 * @defgroup ltedbglevel Debug level
 * Debug level value of LTECommand
 * @{
 */
#define LTE_DEBUG_MUTE		0	/**< Output no debug information */
#define LTE_DEBUG_FATAL		1	/**< Output fatal information */
#define LTE_DEBUG_ERROR		2	/**< Output error information */
#define LTE_DEBUG_WARN		3	/**< Output warning information */
#define LTE_DEBUG_INFO		4	/**< Output general debug information */
#define LTE_DEBUG_DEBUG		5	/**< Output debug information */
#define LTE_DEBUG_VERBOSE	6	/**< Output all debug information */
/**@}*/

/**
 * @defgroup ltecomstr Structure for common interface
 * Data structure for common interface
 * @{
 */
/**
 * LTE command header structure for start/stop commands 
 */
typedef struct {
    int debug_level;		/**< [in] debug information level */
                                /**< (0:none, 1:fatal, 2:error, 3:warn, 4:info, 5:debug, 6:all) */
    int result;			/**< [out] command result (0, -errno) */
} LTECommand;
/**@}*/


/**
 * @defgroup ltecomapi Application Progrming Interface
 * Application programing interface for LTE manager
 * @{
 */
/**
 * LTE function start up command
 * @warning To call this command is needed before using LTE modem device (including after resume).
 * @warning If "command.result" is returned "-EAGAIN", 
 *          need to re-call this function for waiting LINK-UP.
 * @param command : Command structure
 * @retval command.result : Result code
 */
extern void LT_Start( LTECommand* command );

/**
 * LTE function stop command
 * @warning To call this command is needed after using LTE modem device (including before suspend).
 * @param command : Command structure
 * @retval command.result : Result code
 */
extern void LT_Stop( LTECommand* command );
/**@}*/

/**@}*/

#ifdef __cplusplus
}
#endif

#endif
