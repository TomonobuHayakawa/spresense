/****************************************************************************
 * modules/lte/farapi/include/api/socket/farapi_sock.h
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

#ifndef __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_FARAPI_SOCK_H
#define __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_FARAPI_SOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "farapi_socket.h"
#include "farapi_select.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FARAPI_SOCK_TIMEVAL2MS(ptv) \
  ((ptv->tv_sec * 1000L) + (ptv->tv_usec / 1000L))

#define FARAPI_CHECK_VALUELEN(len, type) \
  do { \
    if ((len) < sizeof(type)) \
      { \
        farapi_seterrno(FARAPI_EINVAL); \
        return -1; \
      } \
  } while(0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct farapi_socket_s
{
  uint8_t               flags;
  struct farapi_timeval sendtimeo;
  struct farapi_timeval recvtimeo;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: farapi_sockfd_socket
 *
 * Description:
 *   Given a socket descriptor, return the underlying socket structure.
 *
 * Input Parameters:
 *   sockfd - The socket descriptor index to use.
 *
 * Returned Value:
 *   On success, a reference to the socket structure associated with
 *   the socket descriptor is returned.  NULL is returned on any failure.
 *
 ****************************************************************************/

struct farapi_socket_s *farapi_sockfd_socket(int sockfd);

/****************************************************************************
 * Name: farapi_sockaddr_to_sockstorage
 *
 * Description:
 *   Convert from sockaddr structure to sockaddr_storage structure
 *
 * Parameters:
 *   addr     sockaddr structure.
 *   storage  sockstorage structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void farapi_sockaddr_to_sockstorage(const struct farapi_sockaddr *addr,
                                    struct farapi_sockaddr_storage *storage);

/****************************************************************************
 * Name: farapi_select_request_asyncsend
 *
 * Description:
 *   Send select request.
 *
 * Input parameters:
 *   maxfdp1 - the maximum socket file descriptor number (+1) of any
 *             descriptor in any of the three sets.
 *   readset - the set of descriptions to monitor for read-ready events
 *   writeset - the set of descriptions to monitor for write-ready events
 *   exceptset - the set of descriptions to monitor for error events
 *
 *  Return:
 *  >0: The select id use select cancel request
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int farapi_select_request_asyncsend(int maxfdp1, farapi_fd_set *readset,
                                    farapi_fd_set *writeset,
                                    farapi_fd_set *exceptset);

/****************************************************************************
 * Name: farapi_select_cancel_request_send
 *
 * Description:
 *   Send select cancel request.
 *
 * Input parameters:
 *   id - returned by farapi_select_request_asyncsend
 *
 *  Return:
 *   0: Send succeded
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int farapi_select_cancel_request_send(int id);

/****************************************************************************
 * Name: farapi_select_nonblock
 *
 * Description:
 *   farapi_select_nonblock() is the same as farapi_select(),
 *   but set command parameter to nonblock.
 *
 * Input parameters:
 *   maxfdp1 - the maximum socket file descriptor number (+1) of any
 *             descriptor in any of the three sets.
 *   readset - the set of descriptions to monitor for read-ready events
 *   writeset - the set of descriptions to monitor for write-ready events
 *   exceptset - the set of descriptions to monitor for error events
 *
 *  Return:
 *   0: All descriptors in the three sets are not ready
 *  >0: The number of bits set in the three sets of descriptors
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int farapi_select_nonblock(int maxfdp1, farapi_fd_set *readset,
                           farapi_fd_set *writeset,
                           farapi_fd_set *exceptset);

/****************************************************************************
 * Name: farapi_select_block
 *
 * Description:
 *   farapi_select_nonblock() is the same as farapi_select(),
 *   but set command parameter to block.
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
 *  >0: The number of bits set in the three sets of descriptors
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int farapi_select_block(int maxfdp1, farapi_fd_set *readset,
                        farapi_fd_set *writeset, farapi_fd_set *exceptset,
                        struct farapi_timeval *timeout);

#endif /* __MODULES_LTE_FARAPI_INCLUDE_API_SOCKET_FARAPI_SOCK_H */
