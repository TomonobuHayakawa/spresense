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

#define LOG_TAG "NfcNciHal"
#include "OverrideLog.h"
#include "spdhelper.h"
#include "config.h"

SpdHelper SpdHelper::sSpdH;
static bool SpdHelperIsInit = false;

void SpdHelper::setPatchAsBad()
{
    getInstance().setPatchAsBadImpl();
}

void SpdHelper::incErrorCount()
{
    getInstance().incErrorCountImpl();
}

bool SpdHelper::isPatchBad(UINT8* prm, UINT32 len)
{
    return getInstance().isPatchBadImpl(prm, len);
}

bool SpdHelper::isSpdDebug()
{
    bool b = getInstance().isSpdDebugImpl();
    ALOGD("%s SpdDebug is %s", __func__, (b ? "TRUE" : "FALSE"));
    return b;
}

void SpdHelper::incErrorCountImpl()
{
    if (++mErrorCount >= mMaxErrorCount)
    {
        setPatchAsBadImpl();
    }
}

void SpdHelper::setPatchAsBadImpl()
{
    mIsPatchBad = true;
}

inline const char * toHex(UINT8 b)
{
    static char hex[] = "0123456789ABCDEF";
    static char c[3];
    c[0] = hex[((b >> 4) & 0x0F)];
    c[1] = hex[((b >> 0) & 0x0F)];
    c[2] = '\0';
    return &c[0];
}

bool SpdHelper::isPatchBadImpl(UINT8* prm, UINT32 len)
{
#ifdef SPZ2_IMPL
    // Need C++ Standard library
    return true;
#else
    string strNew;

    // Get the patch ID from the prm data.
    for (int i = 0; i < 8 && i < (int )len; ++i)
        strNew.append(toHex(*prm++));

    // If it is not the same patch as before, then reset things.
    if ( strNew != mPatchId )
    {
        mPatchId = strNew;
        mErrorCount = 0;
        mIsPatchBad = false;
    }

    // Otherwise the 'mIsPatchBad' will tell if its bad or not.
    ALOGD("%s '%s' (%d) is %sa known bad patch file", __func__, mPatchId.c_str(), mErrorCount, (mIsPatchBad ? "" : "not "));

    return mIsPatchBad;
#endif //SPZ2_IMPL
}

void SpdHelper::initialize()
{
    mErrorCount = 0;
#ifndef SPZ2_IMPL    
    // Need C++ Standard library
    mPatchId.erase();
#endif //SPZ2_IMPL
    
    mIsPatchBad = false;
#ifndef SPZ_IMPL
    if(!GetNumValue((char*)NAME_SPD_MAXRETRYCOUNT, &mMaxErrorCount, sizeof(mMaxErrorCount)))
        mMaxErrorCount = DEFAULT_SPD_MAXRETRYCOUNT;
    if (!GetNumValue((char*)NAME_SPD_DEBUG, &mSpdDebug, sizeof(mSpdDebug)))
        mSpdDebug = false;
#else /* SPZ_IMPL */
    mMaxErrorCount = DEFAULT_SPD_MAXRETRYCOUNT;
    mSpdDebug = false;
#endif /* SPZ_IMPL */
}

SpdHelper& SpdHelper::getInstance()
{
    if (SpdHelperIsInit == false)
    {
        sSpdH.initialize();
        SpdHelperIsInit = true;
    }
    return sSpdH;
}

SpdHelper::SpdHelper()
{
    SpdHelper::initialize();
}
