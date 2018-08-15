/****************************************************************************
 * arch/arm/include/cxd56xx/cxd56_gnss.h
 *
 *   Copyright (C) 2016,2017 Sony Corporation. All rights reserved.
 *   Author: Tomoyuki Takahashi <Tomoyuki.A.Takahashi@sony.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_API_H
#define __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_API_H

#include <arch/chip/cxd56_gnss_type.h>

#ifndef TRUE
#define TRUE          1
#endif  /* ifndef TRUE */
#ifndef FALSE
#define FALSE         0
#endif  /* ifndef FALSE */


/* GD errno */
#define GD_OK         0
#define GD_ENOENT     2     /* No such file or directory */
#define GD_EIO        5
#define GD_ENOMEM     12
#define GD_EACCES     13
#define GD_EBUSY      16
#define GD_ENODEV     19
#define GD_EINVAL     22
#define GD_EPIPE      32
#define GD_EIDRM      36
#define GD_ECANCELED  47
#define GD_EBADRQC    54
#define GD_ENODATA    61
#define GD_ETIME      62
#define GD_EPROTO     71    /* Protocol error */
#define GD_ETIMEDOUT  145
#define GD_ESTALE     151

/* GD Start mode */
/*   GD_Start */
#define GD_STMOD_COLD       0
#define GD_STMOD_WARM       1
#define GD_STMOD_WARMACC2   2
#define GD_STMOD_HOT        3
#define GD_STMOD_HOTACC     4
#define GD_STMOD_HOTACC2    5
#define GD_STMOD_HOTACC3    6
#define GD_STMOD_XTC1       7
#define GD_STMOD_XTC2       8

/* GD operation mode */
/*   GD_SetOperationMode, GD_GetOperationMode */
#define GD_OPMOD_NORMAL     1
#define GD_OPMOD_LOWPOWER   2
#define GD_OPMOD_BALANCE    4
#define GD_OPMOD_1PSS       5

/* GNSS bit fields to posioning */
/*   this specifies Satellite systems */
/*   GD_GetAlmanac, GD_SetAlmanac, GD_GetEphemeris, GD_SetEphemeris */
/*   GD_SelectGnss, GD_GetGnss */
/*   GD_SAT_XXX are defined in cxd56_gnss_type.h */

/*
 * SF interface definition
 */
/*
 * GNSS driver require a publication the receiver data and tell a number of satellites to publish
 * Sensor client must prepare callback function and set it to the GNSS driver by GD_SetPublicationCallback.
 */
typedef int GD_INDEX;

/* SF receiver type */
typedef enum {
    GD_RECIVER_NMEA,
} GD_Receiver;

/* SF function type */
typedef void (*GD_CB)(int status, uint32_t value);
typedef void (*GD_CMD_CB)(int status, uint32_t value, uint32_t commandId);
typedef int (*GD_TellPositionFunc)(GD_INDEX *index);
typedef int (*GD_TellSpectrumFunc)(GD_INDEX *index);
typedef int (*GD_SendCommandFunc)(GD_Receiver type, void *data);
typedef int (*GD_NotifyRangeLatchFunc)(void* info, int svnum);
typedef int (*GD_NotifyUpdateEphFunc)(uint32_t gnss, int id);

/* GNSS driver require to change the own power to the power manager */
/* power management infomation for gnss */
typedef enum {
    GD_SLEEP_MODE_UNKONWN = 0,
    GD_SLEEP_MODE_INTERMITTENT,
    GD_SLEEP_MODE_INTERPOLATION
} GD_SLEEP_MODE;

#define GD_POWER_CHANGE_TYPE_GET_STATUS    0
#define GD_POWER_CHANGE_TYPE_POWER_OFF_BB  1
#define GD_POWER_CHANGE_TYPE_POWER_ON_BB   2
#define GD_POWER_CHANGE_TYPE_REQUIRE_SLEEP 5
#define GD_POWER_CHANGE_TYPE_REQUIRE_MODE  6
#define GD_POWER_CHANGE_TYPE_DEBUG         7

#define GD_POWER_USE_ACQCORE0  GNSP_ACQCORE0
#define GD_POWER_USE_ACQCORE1  GNSP_ACQCORE1
#define GD_POWER_USE_ACQCORE2  GNSP_ACQCORE2
#define GD_POWER_USE_ACQCORE3  GNSP_ACQCORE3
#define GD_POWER_USE_ACQCORE4  GNSP_ACQCORE4
#define GD_POWER_USE_ACQCORE5  GNSP_ACQCORE5

/* File Access Mode on Flash/eMMC */
#define GD_FILE_READ_MDOE     0
#define GD_FILE_WRITE_MDOE    1
#define GD_FILE_ADDPEND_MDOE  2


/* SF NMEA control */
typedef struct {
    enum {
        GD_NMEA_COMMAND_MASK = 0,
    } command;
    union {
        uint32_t mask;
    } data;
} GD_NMEA_CONTROL_INFO;

/*
 * return value of GD_GetPowerStatus()
 */
#define GD_POWER_STATUS_COP       (1U << 0)    /* COP Power ON */
#define GD_POWER_STATUS_ITP       (1U << 1)    /* ITP Power ON */
#define GD_POWER_STATUS_BB        (1U << 2)    /* Tycho and RF Power ON */

/*
 * Initialize GNSS Driver
 */
int GD_Initialize(void* data);

/*
 * Set a send SF command function to publish
 */
int GD_SetCommandTxCallback(GD_SendCommandFunc func);

/*
 * Set a callback function to publish
 */
int GD_SetPositioningCallback(GD_TellPositionFunc func);

/*
 * Set a callback function to Spectrum
 */
int GD_SetSpectrumCallback(GD_TellSpectrumFunc func);

/*
 * Set a callback function to publish carrier phase info.
 */
int GD_SetRangeLatchCallback(GD_NotifyRangeLatchFunc func);

/*
 * Set a callback function to publish ephemeris data
 */
int GD_SetUpdateEphemerisCallback(GD_NotifyUpdateEphFunc func);

/*
 * Get the publication data of the satellites by the iterator method
 */
int GD_GetReceiverData(GD_INDEX *ptr, GD_PVT_RECEIVER *receiver);

/*
 * Get the publication data of the satellites by the iterator method
 */
int GD_GetSatelliteData(GD_INDEX *ptr, GD_PVT_SV *satellite);

/*
 * Get AGPS Supl Measurement Position data
 */
int GD_GetSuplPositioningData(GD_INDEX *index, GD_SUPL_POS_DATA *pPosData);

/*
 * Get AGPS Supl Measurement Tracking data
 */
int GD_GetSuplTrackingData(GD_INDEX *index, GD_SUPL_TRK_DATA *pTrackingData, uint8_t *pSuplSatNum);

/*
 * Get carrier phase info. for publication
 */
void GD_GetCarrierPhaseData(void* info, GD_RTK_INFO* infoOut, GD_RTK_SV* svOut);

/*
 * Get ephemeris data for publication
 */
int GD_GetEphemerisData(uint32_t gnss, int id, void* ephOut);

/*
 * Get the Spectrum data  by the iterator method
 */
int GD_GetSpectrumStatusData(GD_INDEX *index, GD_SPECTRUM_DATA *dstData);

/* Get the Spectrum data  
 */
int GD_GetSpectrumData(GD_INDEX *index, GD_SPECTRUM_DATA *dstData , uint8_t nrdPacketCnt);

/*
 * Clear Spectrum data
 */
void GD_ClrSpectrumData(GD_INDEX *index);

/*
 * Start a positioning
 * begining to search the satellites and measure the receiver position
 */
int GD_Start(uint8_t startMode);

/*
 * Start a positioning asynchronously
 * begining to search the satellites and measure the receiver position
 */
int GD_StartAsync(GD_CMD_CB cb, uint32_t handle, uint32_t commandId, uint8_t startMode);

/*
 * Stop a positioning
 */
int GD_Stop(void);

/*
 * Stop a positioning asynchronously
 */
int GD_StopAsync(GD_CMD_CB cb, uint32_t handle, uint32_t commandId);

/*
 * Select GNSSs to positioning
 * These are able to specified by GD_B_SAT_XXX defines.
 */
int GD_SelectSatelliteSystem(uint32_t system);

/*
 * Get current using GNSSs to positioning
 * A argument 'satellite' indicates current GNSSs by bit fields defined by GD_B_SAT_XXX.
 */
int GD_GetSatelliteSystem(uint32_t *system);

/*
 * Set the rough receiver position
 * Three arguments are specifies by 10^6 times of degree.
 */
//int GD_SetReceiverPosition(int32_t latitude, int32_t longitude, int32_t altitude);

/* Set the rough receiver position
*/
int GD_SetReceiverPositionEllipsoidal(double *dLat, double *dLon, double *dHeight);

/* Set the rough receiver position as orgothonal
*/
int GD_SetReceiverPositionOrthogonal(int32_t dX, int32_t dY, int32_t dZ);

/*
 * Set enable or disable the 1PPS output.
 */
int GD_Set1ppsOutput(uint32_t enable);

/*
 * Get the current 1PPS output setting.
 */
int GD_Get1ppsOutput(uint32_t *enable);

/*
 * Set the receiver operation mode
 * 1st argument 'mode' is a operation mode defined by GD_OPMOD_XXX.
 * 2nd argument 'cycle' is a positioning period[ms], default is 1000[ms].
 */
int GD_SetOperationMode(uint32_t mode, uint32_t cycle);

/*
 * Get the receiver operation mode
 */
int GD_GetOperationMode(uint32_t *mode, uint32_t *cycle);

/*
 * Set the TCXO offset
 */
int GD_SetTcxoOffset(int32_t offset);

/*
 * Get the TCXO offset
 */
int GD_GetTcxoOffset(int32_t *offset);

/*
 * Set the estimated current time of the receiver.
 * 1st argument date & time are in UTC.
 */
int GD_SetTime(GD_DATE *date, GD_TIME *time);

/*
 * Set the network time
 */
int GD_SetFrameTime(uint16_t sec, uint32_t fracSec);

#if 0
/*
 * Save the backup data to a Flash memory.
 */
int GD_SaveBackupdata(void);

#endif

/*
 * Get the almanac data
 */
int GD_GetAlmanac(uint32_t satellite, uint32_t* almanac, uint32_t *almanacSize);

/*
 * Set the almanac data
 */
int GD_SetAlmanac(uint32_t satellite, uint32_t *almanac);

/*
 * Get the Ephemeris data
 */
int GD_GetEphemeris(uint32_t satellite, uint32_t* ephemeris, uint32_t *ephemerisSize);

/*
 * Set the Ephemeris data
 */
int GD_SetEphemeris(uint32_t satellite, uint32_t *ephemeris);

/*
 * Select to use or not use the initial position calculation supporting *
 * information of the QZSS L1-SAIF.
 */
int GD_SetQzssPosAssist(uint32_t enable);

/*
 * Get a setting of the initial position calculation supporting *
 * information of the QZSS L1-SAIF.
 */
int GD_GetQzssPosAssist(uint32_t *enable);

/*
 * Set IMES bitrates.
 */
int GD_SetImesBitrate(uint32_t bitrate);

/*
 * Get IMES bitrates.
 */
int GD_GetImesBitrate(uint32_t *bitrate);

/*
 * Set IMES center frequency offset.
 */
int GD_SetImesCenterFreqOffset(uint32_t offset);

/*
 * Set IMES preamble.
 */
int GD_SetImesPreamble(uint32_t preamble);

/*
 * Start GPS test
 */
void GD_StartGpsTest(uint32_t satellite, uint32_t reserve1, uint32_t reserve2, uint32_t reserve3);

/*
 * Stop GPS test
 */
int GD_StopGpsTest(void);

/*
 * Get GPS test result
 */
int GD_GetGpsTestResult(float* cn, float* doppler);

/*
 * Send SF command
 */
int GD_SendCommand(GD_Receiver receiver, void *data);

/*
 * Tell completion of positioning calculation
 */
int GD_TellPositioning(void* data);

/*
 * Tell completion of Spectrum calculation
 */
int GD_TellSpectrumOutput(GD_INDEX* data);

/*
 * (internal) Notify update of ephemeris
 */
int GD_NotifyLatchCarrierPhase( void* info );

/*
 * (internal) Notify update of ephemeris
 */
int GD_NotifyUpdateEphemeris( uint32_t gnss, int id ) ;

/*
 * Suspend
 */
int GD_Suspend(uint32_t level);

/*
 * Resume
 */
int GD_Resume(void);

/*
 * Control Spectrum output
 */
int GD_SpectrumControl(unsigned long time, unsigned int enable, 
                        unsigned char moniPoint1, unsigned char step1,
                        unsigned char moniPoint2, unsigned char step2);

/*
 * Get Power Status
 */
uint32_t GD_GetPowerStatus(void);

/*
 * GNSS core requires sleep.
 */
int GD_RequireSleep(GD_SLEEP_MODE sleepMode, uint32_t sleepTime);

/*
 * GNSS core requires to change power state.
 */
int GD_RequireChagePowerState(uint32_t acqcore);

/*
 * Power Control for GNSS debugging
 */
int GD_RequireDebug(uint8_t type, uint16_t value, uint8_t argSize, void *argv);

/*
 * Save the backup data to a Flash memory.
 */
int GD_SaveBackupdata(void);

/*
 * Load the backup data to a Flash memory.
 */
int GD_LoadBackupdata(void);

/*
 * Erase the backup data on a Flash memory.
 */
int GD_EraseBackup(void);

/*
 *  CEP  Write Assist Data to Flash 
 */
int GD_CepSetAssistData(void *wbuf, uint32_t size, uint32_t counter);

/*
 *  CEP  Erase Assist Data
 */
int GD_CepEraseAssistData(void);

/* 
 * CEP  Check Assist Data Valid 
 */
int GD_CepCheckAssistData(void);

/*
 *  CEP  Get Age Data 
 */
int GD_CepGetAgeData(float *age, float *cepi);

/*
 *  CEP  Reset Assist Data init flag & valid flag
 */
int GD_CepInitAssistData(void);

/*
 * AGPS Set tau
 */
int GD_SetTauGps(double *tau);

/*
 * AGPS Set Acquist
 */
int GD_SetAcquist(uint8_t *pAcquistData, uint16_t acquistSize);

/*
 * Set the estimated current time of the receiver.
 * 1st argument date & time are in GPS time.
 */
int GD_SetTimeGps(GD_DATE *date, GD_TIME *time);

/*
 * Clear Receiver Infomation
 */
int GD_ClearReceiverInfo(uint32_t type);

/*
 * AGPS Set Tow Assist
 */
int GD_SetTowAssist(uint8_t *pAssistData, uint16_t dataSize);

/*
 * AGPS Set UTC Model
 */
int GD_SetUtcModel(uint8_t *pModelData, uint16_t dataSize);

/*
 * Read GNSS data to specified buffer
 */
int GD_ReadBuffer(uint8_t type, int32_t offset, void *buf, uint32_t length);

/*
 * Write GNSS data from specified buffer
 */
int GD_WriteBuffer(uint8_t type, int32_t offset, void *buf, uint32_t length);

/*
 * Set notify mask, this mask flag is cleared when notified(poll/signal)
 */
int GD_SetNotifyMask(uint8_t type, uint8_t clear);

/*
 * Geofence Add Region
 */
int GD_GeoAddRegion(uint8_t id, long lat, long lon, uint16_t rad);

/*
 *  Geofence Modify Region
 */
int GD_GeoModifyRegion(uint8_t id, long lat, long lon, uint16_t rad);

/*
 *   Geofence Delete Region
 */
int GD_GeoDeleteRegione(uint8_t id);

/*
 *  Geofence All delete Region
 */
int GD_GeoDeleteAllRegion(void);

/*
 *  Geofence Region check
 */
int GD_GeoGetRegionData(uint8_t id, long *lat, long *lon, uint16_t *rad);

/*
 *  Geofence Get Used Region ID
 */
uint32_t GD_GeoGetUsedRegionId(void);

/*
 *  Geofence Set mode
 */
int GD_GeoSetOpMode(uint16_t deadzone, uint16_t dwell_detecttime);

/*
 *  Geofence Request All region notify
 */
int GD_GeoSetAllRgionNotifyRequest(void);

/*
 *  Geofence Register to gnss_provider
 */
int GD_RegisterGeofence(void);

/*
 *  Geofence Release from gnss_provider
 */
int GD_ReleaseGeofence(void);

/*
 *  Pvtlog Register to gnss_provider
 */
int GD_RegisterPvtlog(uint32_t cycle, uint32_t threshold);

/*
 *  Pvtlog Release
 */
int GD_ReleasePvtlog(void);

/*
 *  Pvtlog Delete log data
 */
int GD_PvtlogDeleteLog(void);

/*
 *  Pvtlog Get Log status
 */
int GD_PvtlogGetLogStatus(GD_PVTLOG_STATUS *pLogStatus);

/*
 * Start outputting carrier phase info.
 */
int GD_RtkStart( int interval, uint32_t gnss, int ephOut );

/*
 * Stop outputting carrier phase info.
 */
int GD_RtkStop( void );

/*
 * Set output interval of carrier phase info.
 *
 * interval : GD_RTK_INTERVAL_XXX (gd_type.h)
 */
int GD_RtkSetOutputInterval( int interval );

/*
 * Get output interval of carrier phase info. [ms]
 */
int GD_RtkGetOutputInterval( int* interval );

/*
 * Set GNSS of outputting carrier phase info.
 */
int GD_RtkSetGnss( uint32_t gnss );

/*
 * Get GNSS of outputting carrier phase info.
 */
int GD_RtkGetGnss( uint32_t* pGnss );

/*
 * Set enable/disable GD to notify updating ephemeris
 */
int GD_RtkSetEphNotify( int enable );

/*
 * Get enable/disable GD to notify updating ephemeris
 */
int GD_RtkGetEphNotify( int* enable );


#endif /* __ARCH_ARM_SRC_CXD56XX_CXD56_GNSS_API_H */
