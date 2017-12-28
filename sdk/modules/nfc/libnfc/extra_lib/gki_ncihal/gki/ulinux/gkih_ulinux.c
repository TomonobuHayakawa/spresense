/******************************************************************************
 *
 *  Copyright (C) 1999-2012 Sony Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#ifndef SPZ2_IMPL
#include <sys/times.h>
#endif

/* Do not edit GKI_DEBUG in this file */
#ifndef GKI_DEBUG
#define GKI_DEBUG   FALSE
#endif

#include <pthread.h>  /* must be 1st header defined  */
#include <time.h>
#ifdef ANDROID
#include <hardware_legacy/power.h>  /* Android header */
#endif
#include "gki.h"
#include "gkie_target.h"

#ifdef SPZ1_IMPL
#include "pthread_wrapper.h"
#include "syscall_wrapper.h"
#endif /* SPZ1_IMPL */

/* Temp android logging...move to android tgt config file */

#ifndef LINUX_NATIVE
#include <cutils/log.h>
#else
#define LOGV(format, ...)  fprintf (stdout, LOG_TAG format, ## __VA_ARGS__)
#define LOGE(format, ...)  fprintf (stderr, LOG_TAG format, ## __VA_ARGS__)
#define LOGI(format, ...)  fprintf (stdout, LOG_TAG format, ## __VA_ARGS__)

#define SCHED_NORMAL 0
#define SCHED_FIFO 1
#define SCHED_RR 2
#define SCHED_BATCH 3

#endif
/*******************************************************************************
**
** Function         GKIH_run
**
** Description      This function does nothing
**
** Parameters:      p_task_id  - (input) pointer to task id
**
** Returns          void
**
*********************************************************************************/
void GKIH_run (void *p_task_id)
{
    GKI_TRACE_0("GKIH_run");
}

/*******************************************************************************
**
** Function         GKIH_shutdown
**
** Description      This function does nothing
**
** Returns          void
**
*******************************************************************************/
void GKIH_shutdown(void)
{
    GKI_TRACE_0("GKIH_shutdown");
}
