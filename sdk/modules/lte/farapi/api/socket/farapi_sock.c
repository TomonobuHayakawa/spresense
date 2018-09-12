/****************************************************************************
 * modules/lte/farapi/api/socket/farapi_sock.c
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
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>

#include "dbg_if.h"
#include "farapi_sock.h"
#include "farapi_socket.h"
#include "farapi_in.h"
#include "cc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct farapi_socket_s g_farapi_sockets[FARAPI_NSOCKET];

/****************************************************************************
 * Public Functions
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

FAR struct farapi_socket_s *farapi_sockfd_socket(int sockfd)
{
  if (sockfd >= 0 && sockfd < FARAPI_NSOCKET)
    {
      return &g_farapi_sockets[sockfd];
    }

  return NULL;
}

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

void farapi_sockaddr_to_sockstorage(FAR const struct farapi_sockaddr *addr,
                                    FAR struct farapi_sockaddr_storage *storage)
{
  FAR struct farapi_sockaddr_in  *inaddr;
  FAR struct farapi_sockaddr_in6 *in6addr;

  if (addr->sa_family == FARAPI_PF_INET)
    {
      inaddr = (FAR struct farapi_sockaddr_in*)addr;
      storage->s2_len    = inaddr->sin_len;
      storage->ss_family = inaddr->sin_family;

      /* These parameter are network byte order */

      memcpy(&storage->s2_data1[0], &inaddr->sin_port,
             sizeof(farapi_in_port_t));
      memcpy(&storage->s2_data2[0], &inaddr->sin_addr,
             sizeof(struct farapi_in_addr));
    }
  else if (addr->sa_family == FARAPI_PF_INET6)
    {
      in6addr = (FAR struct farapi_sockaddr_in6*)addr;
      storage->s2_len    = in6addr->sin6_len;
      storage->ss_family = in6addr->sin6_family;

      /* These parameter are network byte order */

      memcpy(&storage->s2_data1[0], &in6addr->sin6_port,
             sizeof(farapi_in_port_t));
      memcpy(&storage->s2_data2[1], &in6addr->sin6_addr,
             sizeof(struct farapi_in6_addr));
    }
}
