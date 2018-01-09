/****************************************************************************
 * apps/include/gpsutils/location_manager/location_manager.h
 *
 *   Copyright (C) 2016 Sony Corporation. All rights reserved.
 *   Author: Takanori Yoshino <Takanori.x.Yoshino@sony.com>
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

#ifndef __APPS_INCLUDE_GPSUTILS_LOCATION_MANAGER_H
#define __APPS_INCLUDE_GPSUTILS_LOCATION_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <arch/chip/gnss.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define LOC_MNG_NUM_OF_FIX_INFNITE 0

/****************************************************************************
 * Public Data
 ****************************************************************************/
struct loc_mng_pos_data
{
  int                      ctl_id;    /**< [out] Control ID */
  struct cxd56_gnss_date_s date;      /**< [out] Current day (UTC) */
  struct cxd56_gnss_time_s time;      /**< [out] Current time (UTC) */
  double                   latitude;  /**< [out] Latitude [degree] */
  double                   longitude; /**< [out] Longitude [degree] */
  double                   altitude;  /**< [out] Altitude [degree] */
};

typedef void (FAR *loc_mng_pos_data_out_cbs)(struct loc_mng_pos_data *pos_data);

struct loc_mng_start_param
{
  int                      num_of_fixes; /**< [in] Number of fixes to notify */
  unsigned int             cycle;        /**< [in] Duty cycle of position data output(unit:sec) */
  loc_mng_pos_data_out_cbs cbs;          /**< [in] Position data output callback */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/* Initialize */
int LocMngInit(void);

/* Start */
int LocMngStart(struct loc_mng_start_param *sta_prm, int *ctl_id);

/* Stop */
int LocMngStop(int ctl_id);

/* Finalize */
int LocMngFin(void);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_GPSUTILS_LOCATION_MANAGER_H */