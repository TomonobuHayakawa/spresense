/*
 * Copyright (C) 2012 The Android Open Source Project
 *  Copyright (C) 2013 Sony Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
extern "C"
{
    #include "nfa_api.h"
}

extern bool nfcManager_doInitialize(void);
extern void nfcManager_enableDiscovery (tNFA_TECHNOLOGY_MASK t_mask, bool reader_mode, bool restart);
extern bool nfcManager_disableDiscovery (void);
extern bool nfcManager_doDeinitialize (void);

extern int nativeNfcTag_doCheckNdef (void);
extern int nativeNfcTag_doRead (unsigned char *buf, unsigned short buf_size, int *size);
extern bool nativeNfcTag_doPresenceCheck (void);

extern void nfcHce_settingTag(tNFA_TECHNOLOGY_MASK t_mask_listen, UINT8 *data, UINT16 cur_size, UINT16 max_size, bool read_only);

