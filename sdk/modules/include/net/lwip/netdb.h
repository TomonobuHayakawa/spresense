/**
 * @file
 * NETDB API (sockets)
 */

/*
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
 * Author: Simon Goldschmidt
 *
 */
#ifndef LWIP_HDR_NETDB_H
#define LWIP_HDR_NETDB_H

#include "net/lwip/opt.h"

#if LWIP_DNS && LWIP_SOCKET

#include <stddef.h> /* for size_t */
#include <string.h>

#include "net/lwip/arch.h"
#include "net/lwip/inet.h"
#include "net/lwip/sockets.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @defgroup lteiftop Telephony 
 * Network interface for SDK API
 */

/** 
 * @ingroup lwipiftop
 * @defgroup netlwip Socket
 * Socket interface
 * @{
 */

/* some rarely used options */
#ifndef LWIP_DNS_API_DECLARE_H_ERRNO
#define LWIP_DNS_API_DECLARE_H_ERRNO  1
#endif

#ifndef LWIP_DNS_API_DEFINE_ERRORS
#define LWIP_DNS_API_DEFINE_ERRORS    1
#endif

#ifndef LWIP_DNS_API_DEFINE_FLAGS
#define LWIP_DNS_API_DEFINE_FLAGS     1
#endif

#ifndef LWIP_DNS_API_DECLARE_STRUCTS
#define LWIP_DNS_API_DECLARE_STRUCTS  1
#endif

#if LWIP_DNS_API_DEFINE_ERRORS
/** Errors used by the DNS API functions, h_errno can be one of them */
#define EAI_NONAME      200
#define EAI_SERVICE     201
#define EAI_FAIL        202
#define EAI_MEMORY      203
#define EAI_FAMILY      204

#define HOST_NOT_FOUND  210
#define NO_DATA         211
#define NO_RECOVERY     212
#define TRY_AGAIN       213
#endif /* LWIP_DNS_API_DEFINE_ERRORS */

/**
 * @defgroup lwipnetdbstr DNS structure for common interface
 * Data structure for common interface
 * @{
 */
 
#if LWIP_DNS_API_DEFINE_FLAGS
/* input flags for struct addrinfo */
#define AI_PASSIVE      0x01
#define AI_CANONNAME    0x02
#define AI_NUMERICHOST  0x04
#define AI_NUMERICSERV  0x08
#define AI_V4MAPPED     0x10
#define AI_ALL          0x20
#define AI_ADDRCONFIG   0x40
#endif /* LWIP_DNS_API_DEFINE_FLAGS */

#if LWIP_DNS_API_DECLARE_STRUCTS
struct hostent {
    char  *h_name;      /* Official name of the host. */
    char **h_aliases;   /* A pointer to an array of pointers to alternative host names,
                           terminated by a null pointer. */
    int    h_addrtype;  /* Address type. */
    int    h_length;    /* The length, in bytes, of the address. */
    char **h_addr_list; /* A pointer to an array of pointers to network addresses (in
                           network byte order) for the host, terminated by a null pointer. */
#define h_addr h_addr_list[0] /* for backward compatibility */
};

typedef struct hostent hostent;

struct addrinfo {
    int               ai_flags;      /* Input flags. */
    int               ai_family;     /* Address family of socket. */
    int               ai_socktype;   /* Socket type. */
    int               ai_protocol;   /* Protocol of socket. */
    socklen_t         ai_addrlen;    /* Length of socket address. */
    struct sockaddr  *ai_addr;       /* Socket address of socket. */
    char             *ai_canonname;  /* Canonical name of service location. */
    struct addrinfo  *ai_next;       /* Pointer to next in list. */
};
#endif /* LWIP_DNS_API_DECLARE_STRUCTS */

#define NETDB_ELEM_SIZE           (sizeof(struct addrinfo) + sizeof(struct sockaddr_storage) + DNS_MAX_NAME_LENGTH + 1)

#if LWIP_DNS_API_DECLARE_H_ERRNO
/* application accessible error code set by the DNS API functions */
extern int h_errno;
#endif /* LWIP_DNS_API_DECLARE_H_ERRNO*/
/**@}*/

int NT_GetHostByName(const char *name, hostent* entry);
int lwip_gethostbyname_r(const char *name, struct hostent *ret, char *buf,
                size_t buflen, struct hostent **result, int *h_errnop);
void NT_FreeAddrInfo(struct addrinfo *ai);
int NT_GetAddrInfo(const char *nodename,
       const char *servname,
       const struct addrinfo *hints,
       struct addrinfo **res);

#if LWIP_COMPAT_SOCKETS
/**
 * @defgroup lwipdnsstr DNS interface
 * DNS interface for application
 * @{
 */

/**
 * @brief Get an IP address for a hostname
 *
 * @param[in] name : Here name is either a host name, or an IPv4 address in standard dot notation, 
 *                   or an IPv6 address in colon (and pos-sibly dot) notation. 
 *
 * @retval The gethostbyname() function return the hostent structure or a null pointer if an error occurs.
 *
 * @detail The gethostbyname() function returns a structure of type hostent for the given host name.
 *         Here name is either a host name, or an IPv4 address in standard dot notation, 
 *         or an IPv6 address in colon (and pos-sibly  dot) notation. 
 *         If name is an IPv4 or IPv6 address, no lookup is performed and gethostbyname() simply copies name 
 *         into the h_name field and its struct in_addr equivalent into the h_addr_list[0] field of the returned hostent structure.
 *         If name doesn't end in a dot and the environment variable HOSTALIASES is set, the alias file pointed to by HOSTALIASES
 *         will first be searched for name (see hostname(7) for the file format).
 *         The current domain and its parents are searched unless name ends in a dot.
 */
static inline hostent* gethostbyname(char* name) {
  static hostent dummy;
  int ret;
  static ip_addr_t s_hostent_addr;
  static ip_addr_t *s_phostent_addr[2];

  s_phostent_addr[0] = &s_hostent_addr;
  s_phostent_addr[1] = NULL;
  dummy.h_addr_list = (char**)&s_phostent_addr;
  dummy.h_aliases = NULL;

  ret = NT_GetHostByName(name, &dummy);
  if (ret == 0){
    return &dummy;
  } else{
    return NULL;
  }
}

#define gethostbyname_r(name, ret, buf, buflen, result, h_errnop) \
       lwip_gethostbyname_r(name, ret, buf, buflen, result, h_errnop)
/**
 * @brief Free address information
 *
 * @param[in] addrinfo : addrinfo is a pointer to a linked list of one or more addrinfo structures. 
 *
 * @retval None.
 *
 * @detail This function frees the memory allocated by the getaddrinfo() function. As the result of 
 *         the latter is a link list of addrinfo structures, freeaddrinfo() loops through the list and free each one it turn.
 */
static inline void freeaddrinfo(struct addrinfo *addrinfo)
{
  struct addrinfo *next;

  while (addrinfo != NULL) {
    next = addrinfo->ai_next;
    free(addrinfo);
    addrinfo = next;
  }
}
/**
 * @brief Get address information
 *
 * @param[in] nodname : The nodename and servname arguments are either null pointers or pointers to null-terminated strings. 
 *                      One or both of these two arguments must be a non-null pointer. 
 * @param[in] servname : 
 * @param[in] hints : hints is an optional pointer to a struct addrinfo, as defined by <netdb.h>:
 * @param[in/out] res : After a successful	call to	getaddrinfo(), *res is a pointer to a linked
 *                      list of one or more addrinfo structures. 
 *
 * @retval getaddrinfo() returns zero on success or one of the error codes listed in if an error occurs.
 *
 * @detail Given node and service, which identify an Internet host and a service, getaddrinfo() returns one or 
 *         more addrinfo structures, each of which contains an Internet address that can be specified in a call to 
 *         bind or connect. The getaddrinfo() function combines the functionality provided by the gethostbyname
 *         function into a single interface, but unlike the latter functions, 
 *         getaddrinfo() is reentrant and allows programs to eliminate IPv4-versus-IPv6 dependencies. 
 */

static inline int getaddrinfo(const char *nodname, const char *servname, const struct addrinfo *hints, struct addrinfo **res){
  struct addrinfo *tmp;
  int ret = EAI_MEMORY;

  tmp = malloc(32 + 16 + DNS_MAX_NAME_LENGTH + 1);
  if (tmp != NULL){
    memset((void*)tmp, 0, sizeof(struct addrinfo) + sizeof(struct sockaddr_in));
    tmp->ai_addr = (struct sockaddr *)((u8_t*)tmp + sizeof(struct addrinfo));
    if (nodname != NULL) {
      /* copy nodename to canonname if specified */
      tmp->ai_canonname = ((char*)tmp + sizeof(struct addrinfo) + sizeof(struct sockaddr_in));
    }
    ret = NT_GetAddrInfo(nodname, servname, hints, &tmp);
    *res = tmp;
  }

  return ret;
}
/**@}*/
#endif /* LWIP_COMPAT_SOCKETS */

#ifdef __cplusplus
}
#endif

#endif /* LWIP_DNS && LWIP_SOCKET */

#endif /* LWIP_HDR_NETDB_H */
