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
 * @file       lte_if_agps.h
 * @brief      LTE interface for A-GPS definition header file for SDK API
 * @author     Masatoshi.Ueno@jp.sony.com
 */

#ifndef LTE_IF_AGPS_H__
#define LTE_IF_AGPS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "lte_if_system.h"
#include <arch/chip/gnss_type.h>
#include <arch/chip/gnss.h>

/* Definition */

/**
 * @defgroup lteiftop Telephony
 * LTE interface for SDK API
 */

/**
 * @ingroup lteiftop
 * @defgroup lteifgps LTE A-GPS Manager
 * LTE A-GPS Manager interface for SDK API
 * @{
 */

/** 
 * @defgroup lteagpscomcode Command code
 * Command code for code value of LTECommandHeader
 * @{
 */
/* A-GPS I/F command */
#define LTE_COMMAND_AGPS_QUALITY       0x4001 /**< Configure A-GPS quality of position */
#define LTE_COMMAND_AGPS_SATELLITE     0x4002 /**< Configure A-GPS satellite system usage */
#define LTE_COMMAND_AGPS_SETCAP        0x4003 /**< Set GPS capability */
#define LTE_COMMAND_AGPS_SETREP        0x4004 /**< Set reporting configuration */
#define LTE_COMMAND_AGPS_START         0x4011 /**< A-GPS function start */
#define LTE_COMMAND_AGPS_STOP          0x4012 /**< A-GPS function stop */
#define LTE_COMMAND_AGPS_GNSS_CTL_REQ  0x4101 /**< A-GPS GNSS control request */
#define LTE_COMMAND_AGPS_GNSS_CTL_CNF  0x4102 /**< A-GPS GNSS control confirm */
#define LTE_COMMAND_AGPS_GNSS_MEAS_IND 0x4103 /**< A-GPS GNSS mesurement indication */
#define LTE_COMMAND_AGPS_NET_POS_IND   0x4104 /**< A-GPS netowok positioning indication */
/**@}*/


/** 
 * @defgroup lteagpscomtype Manager type of command code
 * Manager type bitmask for @ref lteagpscomcode "command code"
 * @{
 */
#define LTE_COMMAND_TYPE_AGPS		0x4000	/**< Bitmask for A-GPS Manager */
/**@}*/


/**
 * @defgroup lteagpsstrtype structure type of command header
 * - Structure type for type value of LTECommandHeader
 * - Details of each structure shows at @ref lteagpsstr "Strutures for A-GPS"
 * @{
 */
#define LTE_STRUCT_TYPE_AGPS_QUALITY        0x4001 /**< Structure for A-GPS quality of position */
#define LTE_STRUCT_TYPE_AGPS_SATELLITE      0x4002 /**< Structure for A-GPS satellite system usage */
#define LTE_STRUCT_TYPE_AGPS_SETCAP         0x4003 /**< Structure for A-GPS set GPS capability */
#define LTE_STRUCT_TYPE_AGPS_SETREP         0x4004 /**< Structure for A-GPS set reporting configuration */
#define LTE_STRUCT_TYPE_AGPS_GNSS_CTL_REQ   0x4101 /**< Structure for A-GPS GNSS control request */
#define LTE_STRUCT_TYPE_AGPS_GNSS_CTL_CNF   0x4102 /**< Structure for A-GPS GNSS control confirm */
#define LTE_STRUCT_TYPE_AGPS_GNSS_MEAS_IND  0x4103 /**< Structure for A-GPS GNSS mesurement indication usage */
#define LTE_STRUCT_TYPE_AGPS_NET_POS_IND    0x4104 /**< Structure for A-GPS netowok positioning indication usage */
/**@}*/


/**
 * @defgroup lteagpsflgsat Usage flag of satellite system
 * Satellite system usage flag of LTECommand_AgpsSatellite
 * @{
 */
#define LTE_FLAG_AGPS_SATELLITE_NOT_USE	0x0000	/**< NOT use satellite system */
#define LTE_FLAG_AGPS_SATELLITE_USE	0x0001	/**< Use satellite system */
/**@}*/

/**
 * @defgroup lteagpsflgsetcap support flag of A-GPS capability
 * Supported capability flag of LTECommand_AgpsCapability
 * @{
 */
#define LTE_FLAG_AGPS_CAPABILITY_NOT_SUPPORT   0x0000  /* NOT support A-GPS capability */
#define LTE_FLAG_AGPS_CAPABILITY_SUPPORT       0x0001  /* Support A-GPS capability */
/**@}*/

/**
 * @defgroup lteagpssetrep location reporting recurrence
 * Location reporting recurrence of LTECommand_AgpsSetReport
 * @{
 */
#define LTE_AGPS_REPORTING_RECURRENCE_INFINITY 0   /**< Periodic infinite recurrence */
#define LTE_AGPS_REPORTING_RECURRENCE_MAX      999 /**< Maximum reporting recurrence */
/**@}*/

/**
 * @defgroup lteagpsgnssctrlid GNSS control command
 * GNSS control command usage of LTECommand_AgpsGnssCtlReq and LTECommand_AgpsGnssCtlCnf
 * @{
 */
#define LTE_AGPS_GNSS_CTL_SELECT_SATELLITE_SYSTEM CXD56_GNSS_IOCTL_SELECT_SATELLITE_SYSTEM           /**< Select the satellite systems */
#define LTE_AGPS_GNSS_CTL_START                   CXD56_GNSS_IOCTL_START                             /**< Start the positioning */
#define LTE_AGPS_GNSS_CTL_STOP                    CXD56_GNSS_IOCTL_STOP                              /**< Stop the positioning */
#define LTE_AGPS_GNSS_CTL_SET_TIME_GPS            CXD56_GNSS_IOCTL_AGPS_SET_TIME_GPS                 /**< Set high precision receiver time */
#define LTE_AGPS_GNSS_CTL_SET_RECEIVER_POSITION   CXD56_GNSS_IOCTL_SET_RECEIVER_POSITION_ELLIPSOIDAL /**< Set the receiver approximate position */
#define LTE_AGPS_GNSS_CTL_SET_EPHEMERIS           CXD56_GNSS_IOCTL_SET_EPHEMERIS                     /**< Set ephemeris data */
#define LTE_AGPS_GNSS_CTL_SET_ACQUIST             CXD56_GNSS_IOCTL_AGPS_SET_ACQUIST                  /**< Set acquist data */
#define LTE_AGPS_GNSS_CTL_SAVE_BACKUP_DATA        CXD56_GNSS_IOCTL_SAVE_BACKUP_DATA                  /**< Save the backup data */
#define LTE_AGPS_GNSS_CTL_SET_FRAMETIME           CXD56_GNSS_IOCTL_AGPS_SET_FRAMETIME                /**< Set frame time */
#define LTE_AGPS_GNSS_CTL_SET_TAU_GPS             CXD56_GNSS_IOCTL_AGPS_SET_TAU_GPS                  /**< Set tau_GPS */
#define LTE_AGPS_GNSS_CTL_CLEAR_RECEIVER_INFO     CXD56_GNSS_IOCTL_AGPS_CLEAR_RECEIVER_INFO          /**< Clear info(s) for hot start */
#define LTE_AGPS_GNSS_CTL_GET_TCXO_OFFSET         CXD56_GNSS_IOCTL_GET_TCXO_OFFSET                   /**< Get receiver TCXO offset value */
#define LTE_AGPS_GNSS_CTL_SET_TOW_ASSIST          CXD56_GNSS_IOCTL_AGPS_SET_TOW_ASSIST               /**< Set TOW assist data */
#define LTE_AGPS_GNSS_CTL_SET_UTC_MODEL           CXD56_GNSS_IOCTL_AGPS_SET_UTC_MODEL                /**< Set UTC model data */
/**@}*/
  
/** 
 * @defgroup lteagpscap GPS location
 * GPS location of LTECommand_AgpsSetCap
 */
#define LTE_AGPS_LOCATION_STANDALONE     0 /* GPS location without assistance from server*/
#define LTE_AGPS_LOCATION_AGPS_MSB       1 /* GPS location with assistance from server. Location calculated at the device */
#define LTE_AGPS_LOCATION_AGPS_MSA       2 /* GPS location with assistance from server. Location calculated at the host */
#define LTE_AGPS_LOCATION_ECID           3 /* Cell based location (without GPS). Location calculated at the host */
#define LTE_AGPS_LOCATION_OTDOA          4 /* Advanced Cell based location (without GPS). Location calculated at the host */
/**@}*/

/**
 * @defgroup lteagpscalldef Typedef for callback function
 * Typedef for common callback function
 * @{
 */
/* Data structure */
typedef void (* LTECallbackFunctionAgps)(void * data);	/**< Typedef for callback function */
/**@}*/


/**
 * @defgroup lteagpsstr Structure for A-GPS Manager
 * Data structure for A-GPS Manager
 * @{
 */
/**
 * LTE A-GPS quality of position setup structure for A-GPS Manager
 */
typedef struct {
    int horizontal;				/**< [in] Horizontal accuracy */
    int vertical;				/**< [in] Vertical accuracy */
    int time;					/**< [in] Preferred time to first fix */
    int age;					/**< [in] Maximum location age */
} LTECommand_AgpsQuality;

/**
 * LTE A-GPS satellite setting structure for A-GPS Manager.
 * @warning All values should use @ref lteagpsflgsat "Usage flag" patameter (NOT use or use).
 */
typedef struct {
    int GPS;				/**< [in] Usage of satellite system (GPS) */
    int GLONASS;			/**< [in] Usage of satellite system (GLONASS) */
    int SBAS;				/**< [in] Usage of satellite system (SBAS) */
    int QZSS_L1_CA;			/**< [in] Usage of satellite system (QZSS L1-CA) */
    int IMES;				/**< [in] Usage of satellite system (IMES) */
    int QZSS_L1_SAIF;			/**< [in] Usage of satellite system (QZSS L1-SAIF) */
} LTECommand_AgpsSatellite;

/**
 * LTE A-GPS location reporting recurrence setting structure for A-GPS Manager.
 */
typedef struct {
    int recurrence;        /**< [in] Location reporting recurrence */
    int interval;          /**< [in] Represents the time between fixes reports in milliseconds */
}LTECommand_AgpsSetReport;

/**
 * LTE A-GPS GPS capability setting structure for A-GPS Manager.
 * @ref lteagpsflgsetcap "Supported capability flag" patameter (NOT support or support).
 */
typedef struct {
    int AGPS_MSB;          /**< [in] Capability of AGPS MSB */
    int AGPS_MSA;          /**< [in] Capability of AGPS MSA */
    int ECID;              /**< [in] Capability of ECID */
    int OTDOA;             /**< [in] Capability of OTDOA */
}LTECommand_AgpsCapability;

/**
 * LTE A-GPS GPS capability setting structure for A-GPS Manager
 */
typedef struct {
    LTECommand_AgpsCapability capability; /* [in] A-GPS capability Structure variables */
    int                       gps_location; /* [in] GPS location */
}LTECommand_AgpsSetCap;

/**
 * LTE A-GPS GNSS control request structure for A-GPS Manager.
 */
typedef struct
{
  int ctl_id;                                                     /**< [out] Request control ID */
  struct
  {
    union 
    {
      int                                      use_sat;              /**< [out] Set parametor when "LTE_AGPS_GNSS_CTL_SELECT_SATELLITE_SYSTEM" in request control ID */
      struct cxd56_gnss_agps_time_gps_s        agps_time_gps;        /**< [out] Set parametor when "LTE_AGPS_GNSS_CTL_SET_TIME_GPS" in request control ID */
      struct cxd56_gnss_ellipsoidal_position_s ellipsoidal_position; /**< [out] Set parametor when "LTE_AGPS_GNSS_CTL_SET_RECEIVER_POSITION" in request control ID */
      struct cxd56_gnss_orbital_param_s        orbital_param;        /**< [out] Set parametor when "LTE_AGPS_GNSS_CTL_SET_EPHEMERIS" in request control ID */
      struct cxd56_gnss_agps_acquist_s         agps_acquist;         /**< [out] Set parametor when "LTE_AGPS_GNSS_CTL_SET_ACQUIST" in request control ID */
      struct cxd56_gnss_agps_frametime_s       agps_frametime;       /**< [out] Set parametor when "LTE_AGPS_GNSS_CTL_SET_FRAMETIME" in request control ID */
      double                                   tau_gps;              /**< [out] Set parametor when "LTE_AGPS_GNSS_CTL_SET_TAU_GPS" in request control ID */
      unsigned int                             clear_inf;            /**< [out] Set parametor when "LTE_AGPS_GNSS_CTL_CLEAR_RECEIVER_INFO" in request control ID */
      struct cxd56_gnss_agps_tow_assist_s      tow_assist;           /**< [out] Set parametor when "LTE_AGPS_GNSS_CTL_SET_TOW_ASSIST" in request control ID */
      struct cxd56_gnss_agps_utc_model_s       utc_model;            /**< [out] Set parametor when "LTE_AGPS_GNSS_CTL_SET_UTC_MODEL" in request control ID */
    } u;
  } param;
}  LTECommand_AgpsGnssCtlReq;

/**
 * LTE A-GPS GNSS control confirm structure for A-GPS Manager.
 */
typedef struct
{
  int ctl_id;         /**< [in] confirm control ID */
  int ctl_result;     /**< [in] control result */
  struct
  {
    union 
    {
      int tcxo_offset; /**< [out] Set parametor when "LTE_AGPS_GNSS_CTL_GET_TCXO_OFFSET" in confirm control ID */
    } u;
  } param;
}  LTECommand_AgpsGnssCtlCnf;

/**
 * LTE A-GPS GNSS measurement indication structure for A-GPS Manager.
 */
typedef struct
{
  struct cxd56_gnss_positiondata_s   gnss_pos_data;  /**< [in] Positioning data with SV data */
  struct cxd56_supl_mesurementdata_s supl_meas_data; /**< [in] SUPL positioning data */
}  LTECommand_AgpsGnssMeasInd;

/**
 * LTE A-GPS netowok positioning indication structure for A-GPS Manager.
 */
typedef struct
{
  uint32_t      tow;                /**< [out] TOW (UTC) */
  double        latitude;           /**< [out] Latitude [degree] */
  double        longitude;          /**< [out] Longitude [degree] */
  unsigned char is_altitude;        /**< [out] Altitude fields are valid */
  double        altitude;           /**< [out] Altitude [degree] */
  unsigned char unc_major;          /**< [out] Uncertainty Major */
  unsigned char unc_minor;          /**< [out] Uncertainty Minor */
  unsigned char ori_major;          /**< [out] Oriental Major */
  unsigned char alt_unc;            /**< [out] Altitude Uncertainty */
  unsigned char confidence;         /**< [out] Confidence */
  unsigned char shape_code;         /**< [out] Shape Code */
  struct 
    {
      unsigned is_verdirect    : 1; /**< [out] Vertical Speed direct fields are valid */
      unsigned is_bearing      : 1; /**< [out] Bearing fields are valid */
      unsigned is_hor_speed    : 1; /**< [out] Horizontal Speed fields are valid */
      unsigned is_ver_speed    : 1; /**< [out] Vertical Speed fields are valid */
      unsigned is_hor_unc_speed: 1; /**< [out] Horizontal Uncertainty Speed fields are valid */
      unsigned is_ver_unc_speed: 1; /**< [out] Vertical Uncertainty Speed fields are valid */
      
      unsigned short verdirect;     /**< [out] Vertical Speed direct */
      unsigned short bearing;       /**< [out] Bearing */
      unsigned short hor_speed;     /**< [out] Horizontal Speed */
      unsigned short ver_speed;     /**< [out] Vertical Speed */
      unsigned short hor_unc_speed; /**< [out] Horizontal Uncertainty Speed */
      unsigned short ver_unc_speed; /**< [out] Vertical Uncertainty Speed */
    } velocity;
}  LTECommand_AgpsNetPosInd;

/**@}*/


/**
 * @defgroup lteagpscomstr Structure for common interface
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
        /* for A-GPS manager */
        LTECommand_Error           error;         /**< [out] Error message structure */
        LTECommand_Message         text;          /**< [out] text message structure */
        LTECommand_AgpsQuality     quality;       /**< [in] A-GPS quality of position structure */
        LTECommand_AgpsSatellite   satellite;     /**< [in] A-GPS satellite usage structure */
        LTECommand_AgpsSetCap      set_cap;       /**< [in] A-GPS GPS capability setting structure */
        LTECommand_AgpsSetReport   set_report;    /**< [in] A-GPS location reporting recurrence structure */
        LTECommand_AgpsGnssCtlReq  gnss_ctl_req;  /**< [out] A-GPS GNSS control request structure */
        LTECommand_AgpsGnssCtlCnf  gnss_ctl_cnf;  /**< [in] A-GPS GNSS control confirm structure */
        LTECommand_AgpsGnssMeasInd gnss_meas_ind; /**< [in] A-GPS GNSS measurement indication structure */
        LTECommand_AgpsNetPosInd   nw_pos_ind;    /**< [out] A-GPS netowok positioning indication structure */
    } message;						/**< [in,out] Command message union */
} LTECommand_Agps;
/**@}*/


/**
 * @defgroup lteagpscomapi Application Progrming Interface
 * Application programing interface for LTE manager
 * @{
 */
/**
 * LTE command send API for A-GPS manager
 * @warning This command sends packets to LTE modem device and don't wait to receive response.
 * @warning Response from LTE modem device should be received by using callback interface.
 * @param command : Command structure
 * @retval command.header.result : Result code
 */
extern void LT_SendAgps( LTECommand_Agps* command );

/**
 * LTE callback setup command for A-GPS Manager
 * @warning To call this command is needed if use A-GPS Manager
 * @param callback : Callback function
 * @param command : Command structure buffer for response (LTECommand_Agps)
 */
extern void LT_SetCallbackAgps( LTECallbackFunctionAgps callback, void* command );
/**@}*/

/**@}*/

#ifdef __cplusplus
}
#endif

#endif
