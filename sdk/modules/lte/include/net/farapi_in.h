/****************************************************************************
 * modules/lte/include/net/farapi_in.h
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

#ifndef __MODULES_LTE_INCLUDE_NET_FARAPI_IN_H
#define __MODULES_LTE_INCLUDE_NET_FARAPI_IN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "farapi_socket.h"
#include "farapi_inet.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct farapi_sockaddr_in
{
  uint8_t                sin_len;
  farapi_sa_family_t     sin_family;
  farapi_in_port_t       sin_port;
  struct farapi_in_addr  sin_addr;
#define FARAPI_SIN_ZERO_LEN 8
  char                   sin_zero[FARAPI_SIN_ZERO_LEN];
};

struct farapi_sockaddr_in6
{
  uint8_t                sin6_len;
  farapi_sa_family_t     sin6_family;
  farapi_in_port_t       sin6_port;
  uint32_t               sin6_flowinfo;
  struct farapi_in6_addr sin6_addr;
  uint32_t               sin6_scope_id;
};

struct farapi_ip_mreq
{
  struct farapi_in_addr imr_multiaddr;
  struct farapi_in_addr imr_interface;
};

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


#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_LTE_INCLUDE_NET_FARAPI_IN_H */
