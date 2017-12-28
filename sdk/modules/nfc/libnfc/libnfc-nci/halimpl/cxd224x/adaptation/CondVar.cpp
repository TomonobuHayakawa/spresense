/******************************************************************************
 *
 *  Copyright (C) 2012 Broadcom Corporation
 *  Copyright (C) 2013 Sony Corporation
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

/******************************************************************************
 *
 *  Encapsulate a condition variable for thread synchronization.
 *
 ******************************************************************************/
#define LOG_TAG "NfcNciHal"
#include "OverrideLog.h"
#include "CondVar.h"
#include <errno.h>
#include <string.h>
#ifdef SPZ2_IMPL
//Added for CLOCK_MONOTONIC
#include <time.h>
#endif //SPZ2_IMPL

#ifdef SPZ1_IMPL
extern "C" {
#include "pthread_wrapper.h"
#include "syscall_wrapper.h"
}
#endif /* SPZ_IMPL */
/* build for android4.4 */
#if defined(HAVE_PTHREAD_COND_TIMEDWAIT_MONOTONIC) && (HAVE_PTHREAD_COND_TIMEDWAIT_MONOTONIC==1)
#define pthread_cond_timedwait pthread_cond_timedwait_monotonic
#define pthread_condattr_setclock(...) //
#endif

/*******************************************************************************
**
** Function:        CondVar
**
** Description:     Initialize member variables.
**
** Returns:         None.
**
*******************************************************************************/
CondVar::CondVar ()
{
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
#ifndef SPZ2_IMPL    
    // pthread_condattr_setclock is not supported by NuttX.
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
#endif    
    memset (&mCondition, 0, sizeof(mCondition));
    int const res = pthread_cond_init (&mCondition, &attr);
    if (res)
    {
        ALOGE ("CondVar::CondVar: fail init; error=0x%X", res);
    }
}


/*******************************************************************************
**
** Function:        ~CondVar
**
** Description:     Cleanup all resources.
**
** Returns:         None.
**
*******************************************************************************/
CondVar::~CondVar ()
{
    int const res = pthread_cond_destroy (&mCondition);
    if (res)
    {
        ALOGE ("CondVar::~CondVar: fail destroy; error=0x%X", res);
    }
}


/*******************************************************************************
**
** Function:        wait
**
** Description:     Block the caller and wait for a condition.
**
** Returns:         None.
**
*******************************************************************************/
void CondVar::wait (Mutex& mutex)
{
    int const res = pthread_cond_wait (&mCondition, mutex.nativeHandle());
    if (res)
    {
        ALOGE ("CondVar::wait: fail wait; error=0x%X", res);
    }
}


/*******************************************************************************
**
** Function:        wait
**
** Description:     Block the caller and wait for a condition.
**                  millisec: Timeout in milliseconds.
**
** Returns:         True if wait is successful; false if timeout occurs.
**
*******************************************************************************/
bool CondVar::wait (Mutex& mutex, long millisec)
{
    bool retVal = false;
    struct timespec absoluteTime;

    if (clock_gettime (CLOCK_MONOTONIC, &absoluteTime) == -1)
    {
        ALOGE ("CondVar::wait: fail get time; errno=0x%X", errno);
    }
    else
    {
        absoluteTime.tv_sec += millisec / 1000;
        long ns = absoluteTime.tv_nsec + ((millisec % 1000) * 1000000);
        if (ns > 1000000000)
        {
            absoluteTime.tv_sec++;
            absoluteTime.tv_nsec = ns - 1000000000;
        }
        else
            absoluteTime.tv_nsec = ns;
    }

    int waitResult = pthread_cond_timedwait (&mCondition, mutex.nativeHandle(), &absoluteTime);
    if ((waitResult != 0) && (waitResult != ETIMEDOUT))
        ALOGE ("CondVar::wait: fail timed wait; error=0x%X", waitResult);
    retVal = (waitResult == 0); //waited successfully
    return retVal;
}


/*******************************************************************************
**
** Function:        notifyOne
**
** Description:     Unblock the waiting thread.
**
** Returns:         None.
**
*******************************************************************************/
void CondVar::notifyOne ()
{
    int const res = pthread_cond_signal (&mCondition);
    if (res)
    {
        ALOGE ("CondVar::notifyOne: fail signal; error=0x%X", res);
    }
}
