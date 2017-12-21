#include "lwip/sockets.h"
#include "lwip/netdb.h"

#define NT_GetHostByName(host) gethostbyname((char *)host)
#define NT_GetAddrInfo getaddrinfo
#define NT_FreeAddrInfo freeaddrinfo
#define NT_IPAddrNtoA(addr) inet_ntoa((struct in_addr)(addr)) 
