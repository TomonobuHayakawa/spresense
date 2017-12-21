/**
 * @file
 * Socket API (to be used from non-TCPIP threads)
 */

/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */


#ifndef LWIP_HDR_SOCKETS_H
#define LWIP_HDR_SOCKETS_H

#include "lwip/opt.h"

#if LWIP_SOCKET /* don't build if not configured for use in lwipopts.h */

#include <stddef.h> /* for size_t */
/* FIXME: Avoid conflicting definition of F_GETFL, F_SETFL, and O_NONBLOCK. */
#include <fcntl.h>

#include "lwip/ip_addr.h"
#include "lwip/err.h"
#include "lwip/inet.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 
 * @ingroup lteiftop
 * @defgroup netlwip Socket
 * @brief Socket interface
 * @{
 */

/**
 * @defgroup lwipcomstr Structure for common interface
 * Data structure for common interface
 * @{
 */

/* If your port already typedef's in_port_t, define IN_PORT_T_DEFINED
   to prevent this code from redefining it. */
#if !defined(in_port_t) && !defined(IN_PORT_T_DEFINED)
typedef u16_t in_port_t;
#endif

#if LWIP_IPV4
/* members are in network byte order */
struct sockaddr_in {
  u8_t            sin_len;
  sa_family_t     sin_family;
  in_port_t       sin_port;
  struct in_addr  sin_addr;
#define SIN_ZERO_LEN 8
  char            sin_zero[SIN_ZERO_LEN];
};
#endif /* LWIP_IPV4 */

#if LWIP_IPV6
struct sockaddr_in6 {
  u8_t            sin6_len;      /* length of this structure    */
  sa_family_t     sin6_family;   /* AF_INET6                    */
  in_port_t       sin6_port;     /* Transport layer port #      */
  u32_t           sin6_flowinfo; /* IPv6 flow information       */
  struct in6_addr sin6_addr;     /* IPv6 address                */
  u32_t           sin6_scope_id; /* Set of interfaces for scope */
};
#endif /* LWIP_IPV6 */

struct sockaddr {
  u8_t        sa_len;
  sa_family_t sa_family;
  char        sa_data[14];
};

struct sockaddr_storage {
  u8_t        s2_len;
  sa_family_t ss_family;
  char        s2_data1[2];
  u32_t       s2_data2[3];
#if LWIP_IPV6
  u32_t       s2_data3[3];
#endif /* LWIP_IPV6 */
};

/* If your port already typedef's socklen_t, define SOCKLEN_T_DEFINED
   to prevent this code from redefining it. */
#if !defined(socklen_t) && !defined(SOCKLEN_T_DEFINED)
typedef u32_t socklen_t;
#endif

struct lwip_sock;

#if !LWIP_TCPIP_CORE_LOCKING
/** Maximum optlen used by setsockopt/getsockopt */
#define LWIP_SETGETSOCKOPT_MAXOPTLEN 16

/** This struct is used to pass data to the set/getsockopt_internal
 * functions running in tcpip_thread context (only a void* is allowed) */
struct lwip_setgetsockopt_data {
  /** socket index for which to change options */
  int s;
  /** level of the option to process */
  int level;
  /** name of the option to process */
  int optname;
  /** set: value to set the option to
    * get: value of the option is stored here */
#if LWIP_MPU_COMPATIBLE
  u8_t optval[LWIP_SETGETSOCKOPT_MAXOPTLEN];
#else
  union {
     void *p;
     const void *pc;
  } optval;
#endif
  /** size of *optval */
  socklen_t optlen;
  /** if an error occurs, it is temporarily stored here */
  err_t err;
  /** semaphore to wake up the calling task */
  void* completed_sem;
};
#endif /* !LWIP_TCPIP_CORE_LOCKING */

#if !defined(iovec)
struct iovec {
  void  *iov_base;
  size_t iov_len;
};
#endif

struct msghdr {
  void         *msg_name;
  socklen_t     msg_namelen;
  struct iovec *msg_iov;
  int           msg_iovlen;
  void         *msg_control;
  socklen_t     msg_controllen;
  int           msg_flags;
};

/* Socket protocol types (TCP/UDP/RAW) */
#define SOCK_STREAM     1
#define SOCK_DGRAM      2
#define SOCK_RAW        3

/*
 * Option flags per-socket. These must match the SOF_ flags in ip.h (checked in init.c)
 */
#define SO_REUSEADDR   0x0004 /* Allow local address reuse */
#define SO_KEEPALIVE   0x0008 /* keep connections alive */
#define SO_BROADCAST   0x0020 /* permit to send and to receive broadcast messages (see IP_SOF_BROADCAST option) */


/*
 * Additional options, not kept in so_options.
 */
#define SO_DEBUG       0x0001 /* Unimplemented: turn on debugging info recording */
#define SO_ACCEPTCONN  0x0002 /* socket has had listen() */
#define SO_DONTROUTE   0x0010 /* Unimplemented: just use interface addresses */
#define SO_USELOOPBACK 0x0040 /* Unimplemented: bypass hardware when possible */
#define SO_LINGER      0x0080 /* linger on close if data present */
#define SO_DONTLINGER  ((int)(~SO_LINGER))
#define SO_OOBINLINE   0x0100 /* Unimplemented: leave received OOB data in line */
#define SO_REUSEPORT   0x0200 /* Unimplemented: allow local address & port reuse */
#define SO_SNDBUF      0x1001 /* Unimplemented: send buffer size */
#define SO_RCVBUF      0x1002 /* receive buffer size */
#define SO_SNDLOWAT    0x1003 /* Unimplemented: send low-water mark */
#define SO_RCVLOWAT    0x1004 /* Unimplemented: receive low-water mark */
#define SO_SNDTIMEO    0x1005 /* send timeout */
#define SO_RCVTIMEO    0x1006 /* receive timeout */
#define SO_ERROR       0x1007 /* get error status and clear */
#define SO_TYPE        0x1008 /* get socket type */
#define SO_CONTIMEO    0x1009 /* Unimplemented: connect timeout */
#define SO_NO_CHECK    0x100a /* don't create UDP checksum */


/*
 * Structure used for manipulating linger option.
 */
struct linger {
       int l_onoff;                /* option on/off */
       int l_linger;               /* linger time in seconds */
};

/*
 * Level number for (get/set)sockopt() to apply to socket itself.
 */
#define  SOL_SOCKET  0xfff    /* options for socket level */


#define AF_UNSPEC       0
#define AF_INET         2
#if LWIP_IPV6
#define AF_INET6        10
#else /* LWIP_IPV6 */
#define AF_INET6        AF_UNSPEC
#endif /* LWIP_IPV6 */
#define PF_INET         AF_INET
#define PF_INET6        AF_INET6
#define PF_UNSPEC       AF_UNSPEC

#define IPPROTO_IP      0
#define IPPROTO_ICMP    1
#define IPPROTO_TCP     6
#define IPPROTO_UDP     17
#if LWIP_IPV6
#define IPPROTO_IPV6    41
#define IPPROTO_ICMPV6  58
#endif /* LWIP_IPV6 */
#define IPPROTO_UDPLITE 136
#define IPPROTO_RAW     255

/* Flags we can use with send and recv. */
#define MSG_PEEK       0x01    /* Peeks at an incoming message */
#define MSG_WAITALL    0x02    /* Unimplemented: Requests that the function block until the full amount of data requested can be returned */
#define MSG_OOB        0x04    /* Unimplemented: Requests out-of-band data. The significance and semantics of out-of-band data are protocol-specific */
#define MSG_DONTWAIT   0x08    /* Nonblocking i/o for this operation only */
#define MSG_MORE       0x10    /* Sender will send more */


/*
 * Options for level IPPROTO_IP
 */
#define IP_TOS             1
#define IP_TTL             2

#if LWIP_TCP
/*
 * Options for level IPPROTO_TCP
 */
#define TCP_NODELAY    0x01    /* don't delay send to coalesce packets */
#define TCP_KEEPALIVE  0x02    /* send KEEPALIVE probes when idle for pcb->keep_idle milliseconds */
#define TCP_KEEPIDLE   0x03    /* set pcb->keep_idle  - Same as TCP_KEEPALIVE, but use seconds for get/setsockopt */
#define TCP_KEEPINTVL  0x04    /* set pcb->keep_intvl - Use seconds for get/setsockopt */
#define TCP_KEEPCNT    0x05    /* set pcb->keep_cnt   - Use number of probes sent for get/setsockopt */
#endif /* LWIP_TCP */

#if LWIP_IPV6
/*
 * Options for level IPPROTO_IPV6
 */
#define IPV6_CHECKSUM       7  /* RFC3542: calculate and insert the ICMPv6 checksum for raw sockets. */
#define IPV6_V6ONLY         27 /* RFC3493: boolean control to restrict AF_INET6 sockets to IPv6 communications only. */
#endif /* LWIP_IPV6 */

#if LWIP_UDP && LWIP_UDPLITE
/*
 * Options for level IPPROTO_UDPLITE
 */
#define UDPLITE_SEND_CSCOV 0x01 /* sender checksum coverage */
#define UDPLITE_RECV_CSCOV 0x02 /* minimal receiver checksum coverage */
#endif /* LWIP_UDP && LWIP_UDPLITE*/


#if LWIP_MULTICAST_TX_OPTIONS
/*
 * Options and types for UDP multicast traffic handling
 */
#define IP_MULTICAST_TTL   5
#define IP_MULTICAST_IF    6
#define IP_MULTICAST_LOOP  7
#endif /* LWIP_MULTICAST_TX_OPTIONS */

#if LWIP_IGMP
/*
 * Options and types related to multicast membership
 */
#define IP_ADD_MEMBERSHIP  3
#define IP_DROP_MEMBERSHIP 4

typedef struct ip_mreq {
    struct in_addr imr_multiaddr; /* IP multicast address of group */
    struct in_addr imr_interface; /* local IP address of interface */
} ip_mreq;
#endif /* LWIP_IGMP */

/*
 * The Type of Service provides an indication of the abstract
 * parameters of the quality of service desired.  These parameters are
 * to be used to guide the selection of the actual service parameters
 * when transmitting a datagram through a particular network.  Several
 * networks offer service precedence, which somehow treats high
 * precedence traffic as more important than other traffic (generally
 * by accepting only traffic above a certain precedence at time of high
 * load).  The major choice is a three way tradeoff between low-delay,
 * high-reliability, and high-throughput.
 * The use of the Delay, Throughput, and Reliability indications may
 * increase the cost (in some sense) of the service.  In many networks
 * better performance for one of these parameters is coupled with worse
 * performance on another.  Except for very unusual cases at most two
 * of these three indications should be set.
 */
#define IPTOS_TOS_MASK          0x1E
#define IPTOS_TOS(tos)          ((tos) & IPTOS_TOS_MASK)
#define IPTOS_LOWDELAY          0x10
#define IPTOS_THROUGHPUT        0x08
#define IPTOS_RELIABILITY       0x04
#define IPTOS_LOWCOST           0x02
#define IPTOS_MINCOST           IPTOS_LOWCOST

/*
 * The Network Control precedence designation is intended to be used
 * within a network only.  The actual use and control of that
 * designation is up to each network. The Internetwork Control
 * designation is intended for use by gateway control originators only.
 * If the actual use of these precedence designations is of concern to
 * a particular network, it is the responsibility of that network to
 * control the access to, and use of, those precedence designations.
 */
#define IPTOS_PREC_MASK                 0xe0
#define IPTOS_PREC(tos)                ((tos) & IPTOS_PREC_MASK)
#define IPTOS_PREC_NETCONTROL           0xe0
#define IPTOS_PREC_INTERNETCONTROL      0xc0
#define IPTOS_PREC_CRITIC_ECP           0xa0
#define IPTOS_PREC_FLASHOVERRIDE        0x80
#define IPTOS_PREC_FLASH                0x60
#define IPTOS_PREC_IMMEDIATE            0x40
#define IPTOS_PREC_PRIORITY             0x20
#define IPTOS_PREC_ROUTINE              0x00


/*
 * Commands for ioctlsocket(),  taken from the BSD file fcntl.h.
 * lwip_ioctl only supports FIONREAD and FIONBIO, for now
 *
 * Ioctl's have the command encoded in the lower word,
 * and the size of any in or out parameters in the upper
 * word.  The high 2 bits of the upper word are used
 * to encode the in/out status of the parameter; for now
 * we restrict parameters to at most 128 bytes.
 */
#if !defined(FIONREAD) || !defined(FIONBIO)
#define IOCPARM_MASK    0x7fU           /* parameters must be < 128 bytes */
#define IOC_VOID        0x20000000UL    /* no parameters */
#define IOC_OUT         0x40000000UL    /* copy out parameters */
#define IOC_IN          0x80000000UL    /* copy in parameters */
#define IOC_INOUT       (IOC_IN|IOC_OUT)
                                        /* 0x20000000 distinguishes new &
                                           old ioctl's */
#define _IO(x,y)        (IOC_VOID|((x)<<8)|(y))

#define _IOR(x,y,t)     (IOC_OUT|(((long)sizeof(t)&IOCPARM_MASK)<<16)|((x)<<8)|(y))

#define _IOW(x,y,t)     (IOC_IN|(((long)sizeof(t)&IOCPARM_MASK)<<16)|((x)<<8)|(y))
#endif /* !defined(FIONREAD) || !defined(FIONBIO) */

#ifndef FIONREAD
#define FIONREAD    _IOR('f', 127, unsigned long) /* get # bytes to read */
#endif
#ifndef FIONBIO
#define FIONBIO     _IOW('f', 126, unsigned long) /* set/clear non-blocking i/o */
#endif

/* Socket I/O Controls: unimplemented */
#ifndef SIOCSHIWAT
#define SIOCSHIWAT  _IOW('s',  0, unsigned long)  /* set high watermark */
#define SIOCGHIWAT  _IOR('s',  1, unsigned long)  /* get high watermark */
#define SIOCSLOWAT  _IOW('s',  2, unsigned long)  /* set low watermark */
#define SIOCGLOWAT  _IOR('s',  3, unsigned long)  /* get low watermark */
#define SIOCATMARK  _IOR('s',  7, unsigned long)  /* at oob mark? */
#endif

/* commands for fnctl */
#ifndef F_GETFL
#define F_GETFL 3
#endif
#ifndef F_SETFL
#define F_SETFL 4
#endif

/* File status flags and file access modes for fnctl,
   these are bits in an int. */
#ifndef O_NONBLOCK
#define O_NONBLOCK  1 /* nonblocking I/O */
#endif
#ifndef O_NDELAY
#define O_NDELAY    1 /* same as O_NONBLOCK, for compatibility */
#endif

#ifndef SHUT_RD
  #define SHUT_RD   0
  #define SHUT_WR   1
  #define SHUT_RDWR 2
#endif

/* FD_SET used for lwip_select */
#ifndef FD_SET
#undef  FD_SETSIZE
/* Make FD_SETSIZE match NUM_SOCKETS in socket.c */
#define FD_SETSIZE    MEMP_NUM_NETCONN
#define FDSETSAFESET(n, code) do { \
  if (((n) - LWIP_SOCKET_OFFSET < MEMP_NUM_NETCONN) && (((int)(n) - LWIP_SOCKET_OFFSET) >= 0)) { \
  code; }} while(0)
#define FDSETSAFEGET(n, code) (((n) - LWIP_SOCKET_OFFSET < MEMP_NUM_NETCONN) && (((int)(n) - LWIP_SOCKET_OFFSET) >= 0) ?\
  (code) : 0)
#define FD_SET(n, p)  FDSETSAFESET(n, (p)->fd_bits[((n)-LWIP_SOCKET_OFFSET)/8] |=  (1 << (((n)-LWIP_SOCKET_OFFSET) & 7)))
#define FD_CLR(n, p)  FDSETSAFESET(n, (p)->fd_bits[((n)-LWIP_SOCKET_OFFSET)/8] &= ~(1 << (((n)-LWIP_SOCKET_OFFSET) & 7)))
#define FD_ISSET(n,p) FDSETSAFEGET(n, (p)->fd_bits[((n)-LWIP_SOCKET_OFFSET)/8] &   (1 << (((n)-LWIP_SOCKET_OFFSET) & 7)))
#define FD_ZERO(p)    memset((void*)(p), 0, sizeof(*(p)))

typedef struct fd_set
{
  unsigned char fd_bits [(FD_SETSIZE+7)/8];
} fd_set;

#elif LWIP_SOCKET_OFFSET
#error LWIP_SOCKET_OFFSET does not work with external FD_SET!
#elif FD_SETSIZE < MEMP_NUM_NETCONN
#error "external FD_SETSIZE too small for number of sockets"
#endif /* FD_SET */

/** LWIP_TIMEVAL_PRIVATE: if you want to use the struct timeval provided
 * by your system, set this to 0 and include <sys/time.h> in cc.h */
#if defined(__arm__) && defined(__ARMCC_VERSION) 
#ifndef LWIP_TIMEVAL_PRIVATE
#define LWIP_TIMEVAL_PRIVATE 1
#endif

#if LWIP_TIMEVAL_PRIVATE
struct timeval {
  long    tv_sec;         /* seconds */
  long    tv_usec;        /* and microseconds */
};
#endif /* LWIP_TIMEVAL_PRIVATE */
#endif

#define lwip_socket_init() /* Compatibility define, no init needed. */
void lwip_socket_thread_init(void); /* LWIP_NETCONN_SEM_PER_THREAD==1: initialize thread-local semaphore */
void lwip_socket_thread_cleanup(void); /* LWIP_NETCONN_SEM_PER_THREAD==1: destroy thread-local semaphore */
void all_free_socket(void);

#if LWIP_COMPAT_SOCKETS == 2
/* This helps code parsers/code completion by not having the COMPAT functions as defines */
#define lwip_accept       accept
#define lwip_bind         bind
#define lwip_shutdown     shutdown
#define lwip_getpeername  getpeername
#define lwip_getsockname  getsockname
#define lwip_setsockopt   setsockopt
#define lwip_getsockopt   getsockopt
#define lwip_close        closesocket
#define lwip_connect      connect
#define lwip_listen       listen
#define lwip_recv         recv
#define lwip_recvfrom     recvfrom
#define lwip_send         send
#define lwip_sendmsg      sendmsg
#define lwip_sendto       sendto
#define lwip_socket       socket
#define lwip_select       select
#define lwip_ioctlsocket  ioctl

#if LWIP_POSIX_SOCKETS_IO_NAMES
#define lwip_read         read
#define lwip_write        write
#define lwip_writev       writev
#undef lwip_close
#define lwip_close        close
#define closesocket(s)    close(s)
#define lwip_fcntl        fcntl
#define lwip_ioctl        ioctl
#endif /* LWIP_POSIX_SOCKETS_IO_NAMES */
#endif /* LWIP_COMPAT_SOCKETS == 2 */

int NT_Accept(int s, struct sockaddr *addr, socklen_t *addrlen);
int NT_Bind(int s, const struct sockaddr *name, socklen_t namelen);
int NT_Shutdown(int s, int how);
int NT_GetPeerName (int s, struct sockaddr *name, socklen_t *namelen);
int NT_GetSockName (int s, struct sockaddr *name, socklen_t *namelen);
int NT_GetSockOpt (int s, int level, int optname, void *optval, socklen_t *optlen);
int NT_SetSockOpt (int s, int level, int optname, const void *optval, socklen_t optlen);
int NT_Close(int s);
int NT_Connect(int s, const struct sockaddr *name, socklen_t namelen);
int NT_Listen(int s, int backlog);
int NT_Recv(int s, void *mem, size_t len, int flags);
int NT_Recvfrom(int s, void *mem, size_t len, int flags,
      struct sockaddr *from, socklen_t *fromlen);
int NT_Read(int s, void *mem, size_t len);
int NT_Sendmsg(int s, const struct msghdr *message, int flags);
int NT_Send(int s, const void *dataptr, size_t size, int flags);
int NT_Sendto(int s, const void *dataptr, size_t size, int flags,
    const struct sockaddr *to, socklen_t tolen);

int NT_Write(int s, const void *dataptr, size_t size);
int NT_Writev(int s, const struct iovec *iov, int iovcnt);
int NT_Socket(int domain, int type, int protocol);
int NT_Select(int maxfdp1, fd_set *readset, fd_set *writeset, fd_set *exceptset,
                struct timeval *timeout);
int NT_Ioctl(int s, long cmd, void *argp);
int NT_Read(int s, void *mem, size_t len);
int NT_Write(int s, const void *dataptr, size_t size);
int NT_Fcntl(int s, int cmd, int val);
int NT_Errno(void);

int NT_PPPGetLinkStatus(void);
void NT_PPPStart(void);
void NT_PPPStop(void);

#if LWIP_COMPAT_SOCKETS
#if LWIP_COMPAT_SOCKETS != 2
/**
 * @brief Accept a connection on a socket
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[out] addr : The argument addr is a result argument that is filled-in with the address of the connecting entity, as known to the communications layer.
 * @param[out] addrlen : Actual size of the peer address.
 *
 * @retval These calls return -1 on error. If they succeed, they return a non-negative integer that is a descriptor for the accepted socket.
 *
 * @detail It extracts the first connection request on the queue of pending connections for the listening socket, 
 *         sockfd, creates a new connected socket, and returns a new file descriptor referring to that socket.
 */
#define accept(a,b,c)         NT_Accept(a,b,c)

/**
 * @brief bind a name to a socket
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[in] name : The actual structure passed for the addr argument will depend on the address family.
 * @param[in] namelen : Addrlen specifies the size, in bytes, of the address structure pointed to by addr.
 *
 * @retval These calls return -1 on error. If they succeed, zero is returned.
 *
 * @detail The bind() assigns the address specified by addr to the socket referred to by the fil descriptor sockfd.
 */
#define bind(a,b,c)           NT_Bind(a,b,c)

/**
 * @brief shut down socket send and receive operations
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[in] how : Specifies the type of shutdown. The values are as type(SHUT_RD, SHUT_WR SHUT_RDWR )
 *
 * @retval Upon successful completion, shutdown() shall return 0; otherwise, -1 shall be returned and errno set to indicate the error. 
 *
 * @detail The shutdown() function shall cause all or part of a full-duplex connection on the socket associated with the file descriptor socket to be shut down.
 */
#define shutdown(a,b)         NT_Shutdown(a,b)

/**
 * @brief close an existing socket
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 *
 * @retval This routine returns zero if the operation was successful or -1 if the operation failed. In the latter case, serrno is set appropriately.  
 *
 * @detail closes an existing socket. 
 */
#define closesocket(s)        NT_Close(s)

/**
 * @brief connect a socket
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[in] name : Points to a sockaddr structure containing the peer address. The length and format of the address depend on the address family of the socket
 * @param[in] namelen : Specifies the length of the sockaddr structure pointed to by the address argument.
 *
 * @retval Upon successful completion, connect() shall return 0; otherwise, -1 shall be returned and errno set to indicate the error.
 *
 * @detail The connect() function shall attempt to make a connection on a socket.
 */
#define connect(a,b,c)        NT_Connect(a,b,c)

/**
 * @brief get the socket name 
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[out] name : The actual structure passed for the addr argument will depend on the address family.
 * @param[out] namelen : Actual size of the peer address.
 *
 * @retval Upon successful completion, 0 shall be returned, the address argument shall point to the address of the socket, and the address_len argument shall point to the length of the address. Otherwise, -1 shall be returned and errno set to indicate the error. 
 *
 * @detail The getsockname() function shall retrieve the locally-bound name of the specified socket.
 */
#define getsockname(a,b,c)    NT_GetSockName(a,b,c)

/**
 * @brief  get the name of the peer socket 
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[out] name : The actual structure passed for the addr argument will depend on the address family.
 * @param[out] namelen : Actual size of the peer address.
 *
 * @retval Upon successful completion, 0 shall be returned. Otherwise, -1 shall be returned and errno set to indicate the error. 
 *
 * @detail The getpeername() function shall retrieve the peer address of the specified socket. 
 */
#define getpeername(a,b,c)    NT_GetPeerName(a,b,c)

/**
 * @brief  set the socket options  
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[in] level : The level argument specifies the protocol level at which the option resides.
 * @param[in] optname : The option_name argument and any specified options are passed uninterpreted to the appropriate protocol module for interpretations.
 * @param[in] optval : The original data, exactly as passed to the setsockopt system call. 
 * @param[in] optlen : The original data_size, exactly as passed to the setsockopt system call. 
 *
 * @retval Upon successful completion, setsockopt() shall return 0. Otherwise, -1 shall be returned and errno set to indicate the error.
 *
 * @detail The setsockopt() function shall set the option specified by the option_name argument.
 */
#define setsockopt(a,b,c,d,e) NT_SetSockOpt(a,b,c,d,e)

/**
 * @brief  get the socket options  
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[in] level : The level argument specifies the protocol level at which the option resides.
 * @param[in] optname : The option_name argument and any specified options are passed uninterpreted to the appropriate protocol module for interpretations.
 * @param[out] optval : The original data, exactly as passed to the getsockopt system call. 
 * @param[out] optlen : The original data_size, exactly as passed to the getsockopt system call. 
 *
 * @retval Upon successful completion, getsockopt() shall return 0. Otherwise, -1 shall be returned and errno set to indicate the error.
 *
 * @detail The getsockopt() function shall retrieve  the option specified by the option_name argument.
 */
#define getsockopt(a,b,c,d,e) NT_GetSockOpt(a,b,c,d,e)

/**
 * @brief  listen - listen for socket connections and limit the queue of incoming connections 
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[in] backlog : The backlog argument provides a hint to the implementation which the implementation shall use to limit the number of outstanding connections in the socket's listen queue.
 *
 * @retval Upon successful completions, listen() shall return 0; otherwise, -1 shall be returned and errno set to indicate the error. 
 *
 * @detail The listen() function shall mark a connection-mode socket, specified by the socket argument, as accepting connections. 
 */
#define listen(a,b)           NT_Listen(a,b)

/**
 * @brief receive a message from a connected socket 
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[out] mem : Points to a buffer where the message should be stored. 
 * @param[in] len : Specifies the length in bytes of the buffer pointed to by the buffer argument. 
 * @param[in] flags : Specifies the type of message reception. 
 *
 * @retval Recv() shall return the length of the message in bytes. If no messages are available to be received and the peer has performed an orderly shutdown, recv() shall return 0. Otherwise, -1 shall be returned and errno set to indicate the error. 
 *
 * @detail The recv() function shall return the length of the message written to the buffer pointed to by the buffer argument.
 */
#define recv(a,b,c,d)         NT_Recv(a,b,c,d)

/**
 * @brief receive a message from a socket 
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[out] mem : Points to a buffer where the message should be stored. 
 * @param[in] len : Specifies the length in bytes of the buffer pointed to by the buffer argument. 
 * @param[in] flags : Specifies the type of message reception. 
 * @param[out] from : A null pointer, or points to a sockaddr structure in which the sending address is to be stored. The length and format of the address depend on the address family of the socket. 
 * @param[in/out] fromlen : Specifies the length of the sockaddr structure pointed to by the address argument.
 *
 * @retval Recvfrom() shall return the length of the message in bytes. If no messages are available to be received and the peer has performed an orderly shutdown, recvfrom() shall return 0. Otherwise, the function shall return -1 and set errno to indicate the error.
 *
 * @detail The Recvfrom() function shall return the length of the message written to the buffer pointed to by the buffer argument.
 */
#define recvfrom(a,b,c,d,e,f) NT_Recvfrom(a,b,c,d,e,f)

/**
 * @brief send a message on a socket 
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[in] dataptr : Points to the buffer containing the message to send. 
 * @param[in] len : Specifies the length of the message in bytes. 
 * @param[in] flags : Specifies the type of message reception. 
 *
 * @retval Send() shall return the number of bytes sent. Otherwise, -1 shall be returned and errno set to indicate the error. 
 *
 * @detail The send() function shall initiate transmission of a message from the specified socket to its peer.
 */
#define send(a,b,c,d)         NT_Send(a,b,c,d)

/**
 * @brief send a message on a socket 
 *
 * @param[in] s : Specifies the file descriptor associated with the socket. 
 * @param[in] dataptr : Points to the buffer containing the message to send. 
 * @param[in] len : Specifies the length of the message in bytes. 
 * @param[in] flags : Specifies the type of message reception. 
 * @param[in] to : A null pointer, or points to a sockaddr structure in which the sending address is to be stored. The length and format of the address depend on the address family of the socket. 
 * @param[in] tolen : Specifies the length of the sockaddr structure pointed to by the address argument.
 *
 * @retval Sendto() shall return the number of bytes sent. Otherwise, -1 shall be returned and errno set to indicate the error.
 *
 * @detail The sendto() function shall send a message through a connection-mode or connectionless-mode socket
 */
#define sendto(a,b,c,d,e,f)   NT_Sendto(a,b,c,d,e,f)

/**
 * @brief create an endpoint for communication  
 *
 * @param[in] domain : The domain argument specifies a communication domain
 * @param[in] type : The socket has the indicated type, which specifies the communication semantics.
 * @param[in] protocol : The protocol specifies a particular protocol to be used with the socket.
 *
 * @retval On success, a file descriptor for the new socket is returned. On error, -1 is returned, and errno is set appropriately. 
 *
 * @detail socket() creates an endpoint for communication and returns a descriptor. 
 */
#define socket(a,b,c)         NT_Socket(a,b,c)

/**
 * @brief synchronous I/O multiplexing 
 *
 * @param[in] maxfdp1 : The maxfdp1 argument specifies the range of descriptors to be tested.  
 * @param[in] readset : The readset points to an object of type fd_set that on input specifies the file descriptors to be checked for being ready to read, and on output indicates which file descriptors are ready to read. 
 * @param[in] writeset : The writeset points to an object of type fd_set that on input specifies the file descriptors to be checked for being ready to write, and on output indicates which file descriptors are ready to write. 
 * @param[in] exceptset : The exceptset points to an object of type fd_set that on input specifies the file descriptors to be checked for error conditions pending, and on output indicates which file descriptors have error conditions pending. 
 * @param[in] timeout : The maximum interval to wait for the selection to complete
 *
 * @retval The select() functions shall return the total number of bits set in the bit masks. Otherwise, -1 shall be returned, and errno shall be set to indicate the error. 
 *
 * @detail The select() function shall examine the file descriptor sets whose addresses are passed in the readfds, writefds, and errorfds parameters to see whether some of their descriptors are ready for reading, are ready for writing, or have an exceptional condition pending, respectively. 
 */
#define select(a,b,c,d,e)     NT_Select(a,b,c,d,e)

/**
 * @brief ioctls for terminals and serial lines 
 *
 * @param[in] s : Specifies the file descriptor associated with the socket.  
 * @param[in] cmd : The cmd argument selects the control function to be performed and shall depend on the STREAMS device being addressed. 
 * @param[in] argp : The arg argument represents additional information that is needed by this specific STREAMS device to perform the requested function.
 *
 * @retval The ioctl() system call returns 0 on success. On error it returns -1 and sets errno appropriately. 
 *
 * @detail The ioctl() function manipulates the underlying device parameters of special files. 
 */
#define ioctlsocket(a,b,c)    NT_Ioctl(a,b,c)

#if LWIP_POSIX_SOCKETS_IO_NAMES
/** @ingroup socket */
#define read(s,mem,len)                           NT_Read(s,mem,len)
/** @ingroup socket */
#define write(s,dataptr,len)                      NT_Write(s,dataptr,len)
/** @ingroup socket */
#define writev(s,iov,iovcnt)                      NT_Writev(s,iov,iovcnt)
/** @ingroup socket */
#define close(s)                                  NT_Close(s)
/** @ingroup socket */
#define fcntl(s,cmd,val)                          NT_Fcntl(s,cmd,val)
/** @ingroup socket */
#define ioctl(s,cmd,argp)                         NT_Ioctl(s,cmd,argp)
#endif /* LWIP_POSIX_SOCKETS_IO_NAMES */
#endif /* LWIP_COMPAT_SOCKETS != 2 */

#if LWIP_IPV4 && LWIP_IPV6
/** @ingroup socket */
#define inet_ntop(af,src,dst,size) \
    (((af) == AF_INET6) ? ip6addr_ntoa_r((const ip6_addr_t*)(src),(dst),(size)) \
     : (((af) == AF_INET) ? ip4addr_ntoa_r((const ip4_addr_t*)(src),(dst),(size)) : NULL))
/** @ingroup socket */
#define inet_pton(af,src,dst) \
    (((af) == AF_INET6) ? ip6addr_aton((src),(ip6_addr_t*)(dst)) \
     : (((af) == AF_INET) ? ip4addr_aton((src),(ip4_addr_t*)(dst)) : 0))
#elif LWIP_IPV4 /* LWIP_IPV4 && LWIP_IPV6 */
#define inet_ntop(af,src,dst,size) \
    (((af) == AF_INET) ? ip4addr_ntoa_r((const ip4_addr_t*)(src),(dst),(size)) : NULL)
#define inet_pton(af,src,dst) \
    (((af) == AF_INET) ? ip4addr_aton((src),(ip4_addr_t*)(dst)) : 0)
#else /* LWIP_IPV4 && LWIP_IPV6 */
#define inet_ntop(af,src,dst,size) \
    (((af) == AF_INET6) ? ip6addr_ntoa_r((const ip6_addr_t*)(src),(dst),(size)) : NULL)
#define inet_pton(af,src,dst) \
    (((af) == AF_INET6) ? ip6addr_aton((src),(ip6_addr_t*)(dst)) : 0)
#endif /* LWIP_IPV4 && LWIP_IPV6 */

#endif /* LWIP_COMPAT_SOCKETS */

#ifdef __cplusplus
}
#endif

#endif /* LWIP_SOCKET */

#endif /* LWIP_HDR_SOCKETS_H */
