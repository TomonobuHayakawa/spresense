/****************************************************************************
 * modules/lte/include/net/farapi_netdb.h
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

#ifndef __MODULES_LTE_INCLUDE_NET_FARAPI_NETDB_H
#define __MODULES_LTE_INCLUDE_NET_FARAPI_NETDB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "farapi_socket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* input flags for struct farapi_addrinfo */

#define FARAPI_AI_PASSIVE     0x01
#define FARAPI_AI_CANONNAME   0x02
#define FARAPI_AI_NUMERICHOST 0x04
#define FARAPI_AI_NUMERICSERV 0x08
#define FARAPI_AI_V4MAPPED    0x10
#define FARAPI_AI_ALL         0x20
#define FARAPI_AI_ADDRCONFIG  0x40

/* Errors used by the DNS API functions, h_errno can be one of them */

#define FARAPI_EAI_NONAME     200
#define FARAPI_EAI_SERVICE    201
#define FARAPI_EAI_FAIL       202
#define FARAPI_EAI_MEMORY     203
#define FARAPI_EAI_FAMILY     204

#define FARAPI_HOST_NOT_FOUND 210
#define FARAPI_NO_DATA        211
#define FARAPI_NO_RECOVERY    212
#define FARAPI_TRY_AGAIN      213

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct farapi_hostent
{
  char  *h_name;
  char **h_aliases;
  int    h_addrtype;
  int    h_length;
  char **h_addr_list;
#define h_addr h_addr_list[0]
};

struct farapi_addrinfo
{
  int                     ai_flags;
  int                     ai_family;
  int                     ai_socktype;
  int                     ai_protocol;
  farapi_socklen_t        ai_addrlen;
  struct farapi_sockaddr *ai_addr;
  char                   *ai_canonname;
  struct farapi_addrinfo *ai_next;
};

/* application accessible error code set by the DNS API functions */

extern int farapi_h_errno;

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
 * Name: farapi_freeaddrinfo
 *
 * Description:
 *   The farapi_freeaddrinfo() function frees the memory that was allocated
 *   for the dynamically allocated linked list res. 
 *
 * Parameters:
 *   res - The res returned by farapi_getaddrinfo()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void farapi_freeaddrinfo(struct farapi_addrinfo *res);

/****************************************************************************
 * Name: farapi_getaddrinfo
 *
 * Description:
 *   Given node and service, which identify an Internet host and a service,
 *   farapi_getaddrinfo() returns one or more addrinfo structures, each of
 *   which contains an Internet address that can be specified in a call to
 *   farapi_bind() or farapi_connect(). The farapi_getaddrinfo() is reentrant
 *   and allows programs to eliminate IPv4-versus-IPv6 dependencies. 
 *
 * Parameters:
 *   nodename - Specifies either a numerical network address, or a network
 *              hostname, whose network addresses are looked up and resolved
 *   servname - Sets the port in each returned address structure
 *   hints - Points to an addrinfo structure that specifies criteria for
 *           selecting the socket address structures returned in the list
 *           pointed to by res
 *   res - Pointer to the start of the list
 *
 * Returned Value:
 *  farapi_getaddrinfo() returns 0 if it succeeds, or one of the following
 *  nonzero error codes:
 *
 *     FARAPI_EAI_NONAME
 *     FARAPI_EAI_SERVICE
 *     FARAPI_EAI_FAIL
 *     FARAPI_EAI_MEMORY
 *     FARAPI_EAI_FAMILY
 *
 ****************************************************************************/

int farapi_getaddrinfo(const char *nodename, const char *servname,
                       const struct farapi_addrinfo *hints,
                       struct farapi_addrinfo **res);

/****************************************************************************
 * Name: farapi_gethostbyname
 *
 * Description:
 *   The farapi_gethostbyname() function returns a structure of type hostent
 *   for the given host name. Here name is either a hostname, or an IPv4
 *   address in standard dot notation (as for inet_addr(3)), or an IPv6
 *   address in colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   farapi_gethostbyname_r() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned hostent structure.
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *
 * Returned Value:
 *   Upon successful completion, this function will return a pointer to a
 *   hostent structure if the requested entry was found, and a null pointer
 *   if the end of the database was reached or the requested entry was not
 *   found.
 *
 *   Upon unsuccessful completion, farapi_gethostbyname() will set h_errno to
 *   indicate the error
 *
 ****************************************************************************/

struct farapi_hostent *farapi_gethostbyname(const char *name);

/****************************************************************************
 * Name: farapi_gethostbyname_r
 *
 * Description:
 *   The farapi_gethostbyname_r() function returns a structure of type
 *   hostent for the given host name. Here name is either a hostname, or an
 *   IPv4 address in standard dot notation (as for inet_addr(3)), or an IPv6
 *   address in colon (and possibly dot) notation.
 *
 *   If name is an IPv4 or IPv6 address, no lookup is performed and
 *   farapi_gethostbyname_r() simply copies name into the h_name field
 *   and its struct in_addr equivalent into the h_addr_list[0] field of the
 *   returned hostent structure.
 *
 * Input Parameters:
 *   name - The name of the host to find.
 *   ret - Caller provided location to return the host data.
 *   buf - Caller provided buffer to hold string data associated with the
 *     host data.
 *   buflen - The size of the caller-provided buffer
 *   result - Point to the result on success.
 *   h_errnop - There h_errno value returned in the event of a failure.
 *
 * Returned Value:
 *   0 is returned on success, -1 is returned on a failure
 *   with the returned h_errno value provided the reason for the failure.
 *
 ****************************************************************************/

int farapi_gethostbyname_r(const char *name, struct farapi_hostent *ret,
                           char *buf, size_t buflen,
                           struct farapi_hostent **result, int *h_errnop);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_LTE_INCLUDE_NET_FARAPI_NETDB_H */
