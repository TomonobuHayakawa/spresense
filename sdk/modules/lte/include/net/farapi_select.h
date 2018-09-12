/****************************************************************************
 * modules/lte/include/net/farapi_select.h
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef __MODULES_LTE_INCLUDE_NET_FARAPI_SELECT_H
#define __MODULES_LTE_INCLUDE_NET_FARAPI_SELECT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <string.h>
#include "farapi_socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FARAPI_FD_SETSIZE       FARAPI_NSOCKET

#define FARAPI_FDSETSAFESET(s, code) \
  do { \
    if (((s) < FARAPI_FD_SETSIZE) && ((int)(s) >= 0)) \
      { \
        code; \
      } \
  } while(0)
#define FARAPI_FDSETSAFEGET(s, code) \
  (((s) < FARAPI_FD_SETSIZE) && ((int)(s) >= 0) ? (code) : 0)

#define FARAPI_FD_SET(s, set)   FARAPI_FDSETSAFESET(s, (set)->fd_bits[(s)/8] |=  (1 << ((s) & 7)))
#define FARAPI_FD_CLR(s, set)   FARAPI_FDSETSAFESET(s, (set)->fd_bits[(s)/8] &= ~(1 << ((s) & 7)))
#define FARAPI_FD_ISSET(s, set) FARAPI_FDSETSAFEGET(s, (set)->fd_bits[(s)/8] &   (1 << ((s) & 7)))
#define FARAPI_FD_ZERO(set)     memset((void*)(set), 0, sizeof(*(set)))

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct farapi_fd_set_s
{
  unsigned char fd_bits[(FARAPI_FD_SETSIZE+7)/8];
};

typedef struct farapi_fd_set_s farapi_fd_set;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_select
 *
 * Description:
 *   farapi_select() allows a program to monitor multiple file descriptors,
 *   waiting until one or more of the file descriptors become "ready" for
 *   some class of I/O operation (e.g., input possible).  A file descriptor
 *   is considered  ready if it is possible to perform the corresponding I/O
 *   operation (e.g., farapi_recv()) without blocking.
 *
 * Input parameters:
 *   maxfdp1 - the maximum socket file descriptor number (+1) of any
 *             descriptor in any of the three sets.
 *   readset - the set of descriptions to monitor for read-ready events
 *   writeset - the set of descriptions to monitor for write-ready events
 *   exceptset - the set of descriptions to monitor for error events
 *   timeout - Return at this time if none of these events of interest
 *             occur.
 *
 *  Return:
 *   0: Timer expired
 *  >0: The number of bits set in the three sets of descriptors
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int farapi_select(int maxfdp1, farapi_fd_set *readset,
                  farapi_fd_set *writeset, farapi_fd_set *exceptset,
                  struct farapi_timeval *timeout);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_LTE_INCLUDE_NET_FARAPI_SELECT_H */
