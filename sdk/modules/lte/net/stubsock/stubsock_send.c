/****************************************************************************
 * modules/lte/net/stubsock/stubsock_send.c
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

#include <nuttx/config.h>
#include <sdk/config.h>

#if defined(CONFIG_NET) && defined(CONFIG_NET_DEV_SPEC_SOCK)

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "devspecsock/devspecsock.h"
#include "stubsock.h"
#include "farapi_socket.h"
#include "farapi_errno.h"
#include "dbg_if.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stubsock_send
 *
 * Description:
 *   The stubsock_send() call may be used only when the socket is in
 *   a connected state  (so that the intended recipient is known).
 *
 * Parameters:
 *   psock    An instance of the internal socket structure.
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 ****************************************************************************/

ssize_t stubsock_send(FAR struct socket *psock, FAR const void *buf,
                      size_t len, int flags)
{
  FAR struct devspecsock_conn_s *ds_conn =
    (FAR struct devspecsock_conn_s*)psock->s_conn;
  FAR struct stubsock_conn_s    *conn = ds_conn->devspec_conn;
  int                            sockfd;
  int                            ret;
  int                            val = 0;
#ifdef CONFIG_NET_SOCKOPTS
  struct timeval                 tv_val;
#endif

  DBGIF_ASSERT(conn, "conn == NULL\n");

  sockfd = conn->stubsockid;

  if (_SS_ISNONBLOCK(psock->s_flags))
    {
      val |= FARAPI_O_NONBLOCK;
    }

  /* Whether it is blocking or not,
   * change the behavior of sokcet with fcntl */

  farapi_fcntl(sockfd, FARAPI_SETFL, val);

#ifdef CONFIG_NET_SOCKOPTS
  tv_val.tv_sec = psock->s_sndtimeo / DSEC_PER_SEC;
  tv_val.tv_usec = (psock->s_sndtimeo % DSEC_PER_SEC) * USEC_PER_DSEC;

  farapi_setsockopt(sockfd, FARAPI_SOL_SOCKET, FARAPI_SO_SNDTIMEO, &tv_val,
                    sizeof(tv_val));
#endif

  ret = farapi_send(sockfd, buf, len, stubsock_convflags_remote(flags));
  if (ret < 0)
    {
      ret = farapi_errno();
      ret = -ret;
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_DEV_SPEC_SOCK */
