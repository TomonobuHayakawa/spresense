/****************************************************************************
 * modules/lte/include/net/farapi_socket.h
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

#ifndef __MODULES_LTE_INCLUDE_NET_FARAPI_SOCKET_H
#define __MODULES_LTE_INCLUDE_NET_FARAPI_SOCKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of sockets */

#define FARAPI_NSOCKET             10

/* Address family */

#define FARAPI_AF_UNSPEC           0
#define FARAPI_AF_INET             2
#define FARAPI_AF_INET6            10
#define FARAPI_PF_INET             FARAPI_AF_INET
#define FARAPI_PF_INET6            FARAPI_AF_INET6
#define FARAPI_PF_UNSPEC           FARAPI_AF_UNSPEC

/* Socket protocol type */

#define FARAPI_SOCK_STREAM         1
#define FARAPI_SOCK_DGRAM          2
#define FARAPI_SOCK_RAW            3

/* Protocol */

#define FARAPI_IPPROTO_IP          0
#define FARAPI_IPPROTO_ICMP        1
#define FARAPI_IPPROTO_TCP         6
#define FARAPI_IPPROTO_UDP         17
#define FARAPI_IPPROTO_IPV6        41
#define FARAPI_IPPROTO_ICMPV6      58
#define FARAPI_IPPROTO_UDPLITE     136
#define FARAPI_IPPROTO_RAW         255

/* Flags */

#define FARAPI_MSG_PEEK            0x01
#define FARAPI_MSG_WAITALL         0x02
#define FARAPI_MSG_OOB             0x04
#define FARAPI_MSG_DONTWAIT        0x08
#define FARAPI_MSG_MORE            0x10

/* How */

#define FARAPI_SHUT_RD             0
#define FARAPI_SHUT_WR             1
#define FARAPI_SHUT_RDWR           2

/* Level */

#define FARAPI_SOL_SOCKET          0xfff

/* Option flags per-socket */

#define FARAPI_SO_REUSEADDR        0x0004
#define FARAPI_SO_KEEPALIVE        0x0008
#define FARAPI_SO_BROADCAST        0x0020

/* Additional options, not kept in so_options */

#define FARAPI_SO_ACCEPTCONN       0x0002
#define FARAPI_SO_LINGER           0x0080
#define FARAPI_SO_RCVBUF           0x1002
#define FARAPI_SO_SNDTIMEO         0x1005
#define FARAPI_SO_RCVTIMEO         0x1006
#define FARAPI_SO_ERROR            0x1007
#define FARAPI_SO_TYPE             0x1008
#define FARAPI_SO_NO_CHECK         0x100a

/* Options for level IPPROTO_IP */

#define FARAPI_IP_TOS              1
#define FARAPI_IP_TTL              2

/* Options and types related to multicast membership */

#define FARAPI_IP_ADD_MEMBERSHIP   3
#define FARAPI_IP_DROP_MEMBERSHIP  4

/* Options and types for UDP multicast traffic handling */

#define FARAPI_IP_MULTICAST_TTL    5
#define FARAPI_IP_MULTICAST_IF     6
#define FARAPI_IP_MULTICAST_LOOP   7

/* Options for level FARAPI_IPPROTO_TCP */

#define FARAPI_TCP_NODELAY         0x01
#define FARAPI_TCP_KEEPALIVE       0x02
#define FARAPI_TCP_KEEPIDLE        0x03
#define FARAPI_TCP_KEEPINTVL       0x04
#define FARAPI_TCP_KEEPCNT         0x05

/* Options for level FARAPI_IPPROTO_IPV6 */

#define FARAPI_IPV6_CHECKSUM       7
#define FARAPI_IPV6_V6ONLY         27

/* Options for level FARAPI_IPPROTO_UDPLITE */

#define FARAPI_UDPLITE_SEND_CSCOV  0x01
#define FARAPI_UDPLITE_RECV_CSCOV  0x02

/* Macros for fcntl */

#define FARAPI_GETFL               3
#define FARAPI_SETFL               4

#define FARAPI_O_NONBLOCK          0x01

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint8_t  farapi_sa_family_t;
typedef uint32_t farapi_socklen_t;
typedef uint16_t farapi_in_port_t;

struct farapi_sockaddr
{
  uint8_t            sa_len;
  farapi_sa_family_t sa_family;
  char               sa_data[14];
};

struct farapi_sockaddr_storage
{
  uint8_t            s2_len;
  farapi_sa_family_t ss_family;
  char               s2_data1[2];
  uint32_t           s2_data2[3];
  uint32_t           s2_data3[3];
};

struct farapi_linger
{
  int                l_onoff;
  int                l_linger;
};

struct farapi_timeval
{
  long               tv_sec;
  long               tv_usec;
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

/****************************************************************************
 * Name: farapi_accept
 *
 * Description:
 *   The farapi_accept function is used with connection-based socket types
 *   (FARAPI_SOCK_STREAM). It extracts the first
 *   connection request on the queue of pending connections, creates a new
 *   connected socket with mostly the same properties as 'sockfd', and
 *   allocates a new socket descriptor for the socket, which is returned. The
 *   newly created socket is no longer in the listening state. The original
 *   socket 'sockfd' is unaffected by this call.  Per file descriptor flags
 *   are not inherited across an accept.
 *
 *   The 'sockfd' argument is a socket descriptor that has been created with
 *   farapi_socket(), bound to a local address with farapi_bind(), and is
 *   listening for connections after a call to farapi_listen().
 *
 *   On return, the 'addr' structure is filled in with the address of the
 *   connecting entity. The 'addrlen' argument initially contains the size
 *   of the structure pointed to by 'addr'; on return it will contain the
 *   actual length of the address returned.
 *
 *   If no pending connections are present on the queue, and the socket is
 *   not marked as non-blocking, accept blocks the caller until a connection
 *   is present. If the socket is marked non-blocking and no pending
 *   connections are present on the queue, accept returns EAGAIN.
 *
 * Parameters:
 *   sockfd   The listening socket descriptor
 *   addr     Receives the address of the connecting client
 *   addrlen  Input: allocated size of 'addr', Return: returned size of 'addr'
 *
 * Returned Value:
 *  Returns -1 on error. If it succeeds, it returns a non-negative integer
 *  that is a descriptor for the accepted socket.
 *
 ****************************************************************************/

int farapi_accept(int sockfd, struct farapi_sockaddr *addr,
                  farapi_socklen_t *addrlen);

/****************************************************************************
 * Name: farapi_bind
 *
 * Description:
 *   farapi_bind() gives the socket 'sockfd' the local address 'addr'.
 *   'addr' is 'addrlen' bytes long. Traditionally, this is called 
 *   "assigning a name to a socket." When a socket is created with socket, 
 *   it exists in a name space (address family) but has no name assigned.
 *
 * Parameters:
 *   sockfd   Socket descriptor of the socket to bind
 *   addr     Socket local address
 *   addrlen  Length of 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 ****************************************************************************/

int farapi_bind(int sockfd, const struct farapi_sockaddr *addr,
                farapi_socklen_t addrlen);

/****************************************************************************
 * Name: farapi_close
 *
 * Description:
 *   farapi_close() closes a file descriptor of the socket.
 *
 * Parameters:
 *   sockfd   Socket descriptor to close
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately.
 *
 ****************************************************************************/

int farapi_close(int sockfd);

/****************************************************************************
 * Name: farapi_connect
 *
 * Description:
 *   farapi_connect() connects the socket referred to by the file descriptor
 *   'sockfd' to the address specified by 'addr'. The addrlen argument
 *   specifies the size of 'addr'.  The format of the address in 'addr' is
 *   determined by the address space of the socket 'sockfd'.
 *
 *   If the socket 'sockfd' is of type FARAPI_SOCK_DGRAM then 'addr' is the
 *   address to which datagrams are sent by default, and the only address
 *   from which datagrams are received. If the socket is of type
 *   FARAPI_SOCK_STREAM this call attempts to make a connection to the socket
 *   that is bound to the address specified by 'addr'.
 *
 *   Generally, connection-based protocol sockets may successfully connect()
 *   only once; connectionless protocol sockets may use connect() multiple
 *   times to change their association.  Connectionless sockets may dissolve
 *   the association by connecting to an address with the sa_family member of
 *   sockaddr set to FARAPI_AF_UNSPEC.
 *
 * Parameters:
 *   sockfd    Socket descriptor returned by farapi_socket()
 *   addr      Server address (form depends on type of socket)
 *   addrlen   Length of actual 'addr'
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 ****************************************************************************/

int farapi_connect(int sockfd, const struct farapi_sockaddr *addr,
                   farapi_socklen_t addrlen);

/****************************************************************************
 * Name: farapi_fcntl
 *
 * Description:
 *   Performs fcntl operations on socket
 *
 * Input Parameters:
 *   sockfd - Socket descriptor of the socket to operate on
 *   cmd    - The fcntl command.
 *   val    - Command-specific arguments
 *
 * Returned Value:
 *   0 is returned on success; -1 is returned on failure and
 *   the errno value is set appropriately.
 *
 ****************************************************************************/

int farapi_fcntl(int sockfd, int cmd, int val);

/****************************************************************************
 * Name: farapi_getsockname
 *
 * Description:
 *   The farapi_getsockname() function retrieves the locally-bound name of the
 *   specified socket, stores this address in the sockaddr structure pointed
 *   to by the 'addr' argument, and stores the length of this address in the
 *   object pointed to by the 'addrlen' argument.
 *
 *   If the actual length of the address is greater than the length of the
 *   supplied sockaddr structure, the stored address will be truncated.
 *
 *   If the socket has not been bound to a local name, the value stored in
 *   the object pointed to by address is unspecified.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket [in]
 *   addr     sockaddr structure to receive data [out]
 *   addrlen  Length of sockaddr structure [in/out]
 *
 * Returned Value:
 *   On success, 0 is returned, the 'addr' argument points to the address
 *   of the socket, and the 'addrlen' argument points to the length of the
 *   address. Otherwise, -1 is returned and errno is set to indicate the error.
 *
 ****************************************************************************/

int farapi_getsockname(int sockfd, struct farapi_sockaddr *addr,
                       farapi_socklen_t *addrlen);

/****************************************************************************
 * Name: farapi_getsockopt
 *
 * Description:
 *   farapi_getsockopt() retrieve thse value for the option specified by the
 *   'option' argument for the socket specified by the 'sockfd' argument. If
 *   the size of the option value is greater than 'value_len', the value
 *   stored in the object pointed to by the 'value' argument will be silently
 *   truncated. Otherwise, the length pointed to by the 'value_len' argument
 *   will be modified to indicate the actual length of the'value'.
 *
 *   The 'level' argument specifies the protocol level of the option. To
 *   retrieve options at the socket level, specify the level argument as
 *   FARAPI_SOL_SOCKET.
 *
 * Parameters:
 *   sockfd    Socket descriptor of socket
 *   level     Protocol level to set the option
 *   option    identifies the option to get
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   0 on success; -1 on error with errno set appropriately
 *
 ****************************************************************************/

int farapi_getsockopt(int sockfd, int level, int option, void *value,
                      farapi_socklen_t *value_len);

/****************************************************************************
 * Name: farapi_ioctl
 *
 * Description:
 *   Perform device specific operations.
 *
 * Parameters:
 *   sockfd   Socket descriptor of device
 *   req      The ioctl command
 *   argp     A third argument of type unsigned long is expected
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   -1 on failure with errno set properly.
 *
 ****************************************************************************/

int farapi_ioctl(int sockfd, long req, void *argp);

/****************************************************************************
 * Name: farapi_listen
 *
 * Description:
 *   To accept connections, a socket is first created with farapi_socket(), a
 *   willingness to accept incoming connections and a queue limit for incoming
 *   connections are specified with listen(), and then the connections are
 *   accepted with farapi_accept(). The farapi_listen() call applies only to
 *   sockets of type FARAPI_SOCK_STREAM.
 *
 * Parameters:
 *   sockfd   Socket descriptor of the bound socket
 *   backlog  The maximum length the queue of pending connections may grow.
 *            If a connection request arrives with the queue full, the client
 *            may receive an error with an indication of ECONNREFUSED or,
 *            if the underlying protocol supports retransmission, the request
 *            may be ignored so that retries succeed.
 *
 * Returned Value:
 *   On success, zero is returned. On error, -1 is returned, and errno is set
 *   appropriately.
 *
 ****************************************************************************/

int farapi_listen(int sockfd, int backlog);

/****************************************************************************
 * Name: farapi_read
 *
 * Description:
 *   The farapi_read() is the same as farapi_recv() with the flags parameter
 *   set to zero.
 *
 * Parameters:
 *   sockfd   File structure instance
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *
 * Return:
 *   On success, returns the number of characters received. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 ****************************************************************************/

int farapi_read(int sockfd, void *buf, size_t len);

/****************************************************************************
 * Name: farapi_recv
 *
 * Description:
 *   The farapi_recv() call is identical to farapi_recvfrom() with a NULL
 *   from parameter.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *
 * Returned Value:
 *  (see recvfrom)
 *
 ****************************************************************************/

int farapi_recv(int sockfd, void *buf, size_t len, int flags);

/****************************************************************************
 * Name: farapi_recvfrom
 *
 * Description:
 *   farapi_recvfrom() receives messages from a socket, and may be used to
 *   receive data on a socket whether or not it is connection-oriented.
 *
 *   If from is not NULL, and the underlying protocol provides the source
 *   address, this source address is filled in. The argument fromlen
 *   initialized to the size of the buffer associated with from, and modified
 *   on return to indicate the actual size of the address stored there.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Buffer to receive data
 *   len      Length of buffer
 *   flags    Receive flags
 *   from     Address of source (may be NULL)
 *   fromlen  The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters received. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 ****************************************************************************/

int farapi_recvfrom(int sockfd, void *buf, size_t len, int flags,
                    struct farapi_sockaddr *from, farapi_socklen_t *fromlen);

/****************************************************************************
 * Name: farapi_send
 *
 * Description:
 *   The farapi_send() call may be used only when the socket is in
 *   a connected state (so that the intended recipient is known). The only
 *   difference between farapi_send() and farapi_write() is the presence of
 *   flags. With zero flags parameter, farapi_send() is equivalent to 
 *   farapi_write(). Also, send(sockfd,buf,len,flags) is equivalent to
 *   farapi_sendto(sockfd,buf,len,flags,NULL,0).
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *
 * Returned Value:
 *   On success, returns the number of characters sent. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 ****************************************************************************/

int farapi_send(int sockfd, const void *buf, size_t len, int flags);

/****************************************************************************
 * Name: farapi_sendto
 *
 * Description:
 *   If farapi_sendto() is used on a connection-mode (FARAPI_SOCK_STREAM)
 *   socket, the parameters to and 'tolen' are ignored (and the error EISCONN
 *   may be returned when they are not NULL and 0), and the error ENOTCONN is
 *   returned when the socket was not actually connected.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Data to send
 *   len      Length of data to send
 *   flags    Send flags
 *   to       Address of recipient
 *   tolen    The length of the address structure
 *
 * Returned Value:
 *   On success, returns the number of characters sent. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 ****************************************************************************/

int farapi_sendto(int sockfd, const void *buf, size_t len, int flags,
                  const struct farapi_sockaddr *to, farapi_socklen_t tolen);

/****************************************************************************
 * Name: farapi_setsockopt
 *
 * Description:
 *   farapi_setsockopt() sets the option specified by the 'option' argument,
 *   at the protocol level specified by the 'level' argument, to the value
 *   pointed to by the 'value' argument for the socket associated with the
 *   file descriptor specified by the 'sockfd' argument.
 *
 *   The 'level' argument specifies the protocol level of the option. To set
 *   options at the socket level, specify the level argument
 *   as FARAPI_SOL_SOCKET.
 *
 * Parameters:
 *   sockfd    Socket descriptor of socket
 *   level     Protocol level to set the option
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *  0 on success; -1 on failure
 *
 ****************************************************************************/

int farapi_setsockopt(int sockfd, int level, int option, const void *value,
                      farapi_socklen_t value_len);

/****************************************************************************
 * Name: farapi_shutdown
 *
 * Description:
 *   The farapi_shutdown() function will cause all or part of a full-duplex
 *   connection on the socket associated with the file descriptor socket to
 *   be shut down.
 *
 *   The farapi_shutdown() function disables subsequent send and/or receive
 *   operations on a socket, depending on the value of the how argument.
 *
 * Input Paramteers:
 *   sockfd - Specifies the file descriptor of the socket.
 *   how    - Specifies the type of shutdown. The values are as follows:
 *
 *     FARAPI_SHUT_RD   - Disables further receive operations.
 *     FARAPI_SHUT_WR   - Disables further send operations.
 *     FARAPI_SHUT_RDWR - Disables further send and receive operations.
 *
 * Returned Value:
 *   Upon successful completion, farapi_shutdown() will return 0; otherwise,
 *   -1 will be returned and errno set to indicate the error.
 *
 ****************************************************************************/

int farapi_shutdown(int sockfd, int how);

/****************************************************************************
 * Name: farapi_socket
 *
 * Description:
 *   farapi_socket() creates an endpoint for communication and returns
 *   a descriptor.
 *
 * Parameters:
 *   domain   See Address family.
 *   type     See Socket protocol type.
 *   protocol See Protocol.
 *
 * Returned Value:
 *   A non-negative socket descriptor on success; -1 on error with errno set
 *   appropriately.
 *
 ****************************************************************************/

int farapi_socket(int domain, int type, int protocol);

/****************************************************************************
 * Name: farapi_write
 *
 * Description:
 *   The farapi_write() is the same as farapi_send() with the flags parameter
 *   set to zero.
 *
 * Parameters:
 *   sockfd   Socket descriptor of socket
 *   buf      Data to write
 *   len      Length of data to send
 *
 * Returned Value:
 *   On success, returns the number of characters sent. On error,
 *   -1 is returned, and errno is set appropriately.
 *
 ****************************************************************************/

int farapi_write(int sockfd, const void *buf, size_t len);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_LTE_INCLUDE_NET_FARAPI_SOCKET_H */
