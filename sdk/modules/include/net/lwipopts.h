/****************************************************************************
 * include/net/lwip/net_lwip/lwipopts.h
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Yutaka Miyajima <Yutaka.Miyajima@sony.com>
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

#ifndef __LWIPOPTS_H_
#define __LWIPOPTS_H_

#define LWIP_DEBUG 0

/* DHCP is ok, UDP is required with DHCP */
#define LWIP_RAW                        1
#define LWIP_DHCP                       0
#define LWIP_UDP                        1
#define LWIP_DNS                        1
#define LWIP_PPP                        1
#define PPP_SUPPORT                     1
#define PPPOE_SUPPORT                   0
#define PPPOS_SUPPORT                   1
#define LWIP_ARP                        0
#define LWIP_TELNET                     0
#define LWIP_COMPAT_MUTEX               0
#define ETHARP_STATS                    0
#define PPP_INPROC_OWNTHREAD            1
#define LWIP_SO_SNDRCVTIMEO_NONSTANDARD 0

#define LWIP_RAND()  1234

/* MSS should match the hardware packet size */
#define TCP_MSS                         1440
#define TCP_SND_BUF                     (2 * TCP_MSS)
#define TCP_WND                         (8 * TCP_MSS)

#define LWIP_SOCKET                     1
#define LWIP_NETCONN                    1
#define MEMP_NUM_SYS_TIMEOUT            300

/**
 * MBOX_MSG_SIZE 
 */
#ifndef MEMP_NUM_MBOX
#define MEMP_NUM_MBOX                  10
#endif
#ifndef MBOX_MSG_SIZE
#define MBOX_MSG_SIZE                  8
#endif

/* There are more *_DEBUG options that can be selected.
   See opts.h. Make sure that LWIP_DEBUG is defined when
   building the code to use debug. */
#define TCP_DEBUG                       LWIP_DBG_OFF
#define PBUF_DEBUG                      LWIP_DBG_OFF
#define IP_DEBUG                        LWIP_DBG_OFF
#define IP6_DEBUG                       LWIP_DBG_OFF
#define TCPIP_DEBUG                     LWIP_DBG_OFF
#define UDP_DEBUG                       LWIP_DBG_OFF
#define SOCKETS_DEBUG                   LWIP_DBG_OFF
#define PPP_DEBUG                       LWIP_DBG_OFF
#define ICMP_DEBUG                      LWIP_DBG_OFF
#define NETIF_DEBUG                     LWIP_DBG_OFF
#define WEBSOCK_DEBUG                   LWIP_DBG_OFF
#define DNS_DEBUG                       LWIP_DBG_OFF

#define USE_ALT1160                     1
#define USE_UART                        0
#define USE_BLE                         0
#define LWIP_NETIF_LOOPBACK             1
#define LWIP_HAVE_LOOPIF                1
#define LWIP_POSIX_SOCKETS_IO_NAMES     0

#define PPP_THREAD_STACKSIZE            2048
#define PPP_THREAD_PRIO                 100

#if USE_UART
#define PPP_MTU                         1016     /* Default MTU (size of Info field) */
#define PPP_MAXMTU                      1016     /* Largest MTU we allow */
#define PPP_MINMTU                      64
#define PPP_MRU                         1016     /* default MRU = max length of info field */
#define PPP_MAXMRU                      1016     /* Largest MRU we allow */
#else
#define PPP_MTU                         1500
#define PPP_MAXMTU                      1500
#define PPP_MINMTU                      64
#define PPP_MRU                         1500
#define PPP_MAXMRU                      1500
#endif

#define MEMP_NUM_NETCONN                10

#define NETIF_LINKUP_IPV6 2
#define NETIF_LINKUP_IPV4 1
#define NETIF_LINKUP 1
#define NETIF_LINKDOWN 0

#define LWIP_IPV6                   1
#define LWIP_ICMP6                  1
#define LWIP_IPV6_MLD               1
#define PPP_IPV6_SUPPORT            1

#define LWIP_IPV6_REASS             0
#define LWIP_ND6_TCP_REACHABILITY_HINTS 0
#define LWIP_STATS                  0
#define LWIP_DNS_API_DECLARE_STRUCTS 1
#define LWIP_PPP_API 1

#define LWIP_SO_RCVTIMEO 1
#define LWIP_SO_SNDTIMEO 1
#define LWIP_SOCKET_SET_ERRNO 1
#define SO_REUSE                        1
#define LWIP_STATS                      0
#define LWIP_IPV6_FRAG                  0

/* TCPIP thread must run at higher priority than MAC threads! */
#define TCPIP_THREAD_PRIO               (100)
#define TCPIP_THREAD_STACKSIZE          (2048)

/* Needed for malloc/free */
#include <stdlib.h>

#endif /* __LWIPOPTS_H_ */
