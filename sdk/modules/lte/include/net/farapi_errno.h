/****************************************************************************
 * modules/lte/include/net/farapi_errno.h
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

#ifndef __MODULES_LTE_INCLUDE_NET_FARAPI_ERRNO_H
#define __MODULES_LTE_INCLUDE_NET_FARAPI_ERRNO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FARAPI_EPERM              1
#define FARAPI_ENOENT             2
#define FARAPI_ESRCH              3
#define FARAPI_EINTR              4
#define FARAPI_EIO                5
#define FARAPI_ENXIO              6
#define FARAPI_E2BIG              7
#define FARAPI_ENOEXEC            8
#define FARAPI_EBADF              9
#define FARAPI_ECHILD             10
#define FARAPI_EAGAIN             11
#define FARAPI_ENOMEM             12
#define FARAPI_EACCES             13
#define FARAPI_EFAULT             14
#define FARAPI_ENOTBLK            15
#define FARAPI_EBUSY              16
#define FARAPI_EEXIST             17
#define FARAPI_EXDEV              18
#define FARAPI_ENODEV             19
#define FARAPI_ENOTDIR            20
#define FARAPI_EISDIR             21
#define FARAPI_EINVAL             22
#define FARAPI_ENFILE             23
#define FARAPI_EMFILE             24
#define FARAPI_ENOTTY             25
#define FARAPI_ETXTBSY            26
#define FARAPI_EFBIG              27
#define FARAPI_ENOSPC             28
#define FARAPI_ESPIPE             29
#define FARAPI_EROFS              30
#define FARAPI_EMLINK             31
#define FARAPI_EPIPE              32
#define FARAPI_EDOM               33
#define FARAPI_ERANGE             34
#define FARAPI_ENOMSG             35
#define FARAPI_EIDRM              36
#define FARAPI_ECHRNG             37
#define FARAPI_EL2NSYNC           38
#define FARAPI_EL3HLT             39
#define FARAPI_EL3RST             40
#define FARAPI_ELNRNG             41
#define FARAPI_EUNATCH            42
#define FARAPI_ENOCSI             43
#define FARAPI_EL2HLT             44
#define FARAPI_EDEADLK            45
#define FARAPI_ENOLCK             46
#define FARAPI_EBADE              50
#define FARAPI_EBADR              51
#define FARAPI_EXFULL             52
#define FARAPI_ENOANO             53
#define FARAPI_EBADRQC            54
#define FARAPI_EBADSLT            55
#define FARAPI_EDEADLOCK          56
#define FARAPI_EBFONT             57
#define FARAPI_ENOSTR             60
#define FARAPI_ENODATA            61
#define FARAPI_ETIME              62
#define FARAPI_ENOSR              63
#define FARAPI_ENONET             64
#define FARAPI_ENOPKG             65
#define FARAPI_EREMOTE            66
#define FARAPI_ENOLINK            67
#define FARAPI_EADV               68
#define FARAPI_ESRMNT             69
#define FARAPI_ECOMM              70
#define FARAPI_EPROTO             71
#define FARAPI_EMULTIHOP          74
#define FARAPI_ELBIN              75
#define FARAPI_EDOTDOT            76
#define FARAPI_EBADMSG            77
#define FARAPI_EFTYPE             79
#define FARAPI_ENOTUNIQ           80
#define FARAPI_EBADFD             81
#define FARAPI_EREMCHG            82
#define FARAPI_ELIBACC            83
#define FARAPI_ELIBBAD            84
#define FARAPI_ELIBSCN            85
#define FARAPI_ELIBMAX            86
#define FARAPI_ELIBEXEC           87
#define FARAPI_ENOSYS             88
#define FARAPI_ENMFILE            89
#define FARAPI_ENOTEMPTY          90
#define FARAPI_ENAMETOOLONG       91
#define FARAPI_ELOOP              92
#define FARAPI_EOPNOTSUPP         95
#define FARAPI_EPFNOSUPPORT       96
#define FARAPI_ECONNRESET         104
#define FARAPI_ENOBUFS            105
#define FARAPI_EAFNOSUPPORT       106
#define FARAPI_EPROTOTYPE         107
#define FARAPI_ENOTSOCK           108
#define FARAPI_ENOPROTOOPT        109
#define FARAPI_ESHUTDOWN          110
#define FARAPI_ECONNREFUSED       111
#define FARAPI_EADDRINUSE         112
#define FARAPI_ECONNABORTED       113
#define FARAPI_ENETUNREACH        114
#define FARAPI_ENETDOWN           115
#define FARAPI_ETIMEDOUT          116
#define FARAPI_EHOSTDOWN          117
#define FARAPI_EHOSTUNREACH       118
#define FARAPI_EINPROGRESS        119
#define FARAPI_EALREADY           120
#define FARAPI_EDESTADDRREQ       121
#define FARAPI_EMSGSIZE           122
#define FARAPI_EPROTONOSUPPORT    123
#define FARAPI_ESOCKTNOSUPPORT    124
#define FARAPI_EADDRNOTAVAIL      125
#define FARAPI_ENETRESET          126
#define FARAPI_EISCONN            127
#define FARAPI_ENOTCONN           128
#define FARAPI_ETOOMANYREFS       129
#define FARAPI_EPROCLIM           130
#define FARAPI_EUSERS             131
#define FARAPI_EDQUOT             132
#define FARAPI_ESTALE             133
#define FARAPI_ENOTSUP            134
#define FARAPI_ENOMEDIUM          135
#define FARAPI_ENOSHARE           136
#define FARAPI_ECASECLASH         137
#define FARAPI_EILSEQ             138
#define FARAPI_EOVERFLOW          139
#define FARAPI_ECANCELED          140
#define FARAPI_ENOTRECOVERABLE    141
#define FARAPI_EOWNERDEAD         142
#define FARAPI_ESTRPIPE           143
#define FARAPI_EWOULDBLOCK        FARAPI_EAGAIN


#define FARAPI_ERRNO_MIN          FARAPI_EPERM
#define FARAPI_ERRNO_MAX          FARAPI_ESTRPIPE

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
 * Name: farapi_errno
 *
 * Description:
 *   Get errno for socket APIs.
 *
 * Input parameters:
 *   None
 *
 *  Return:
 *   errno
 *
 ****************************************************************************/

int32_t farapi_errno(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __MODULES_LTE_INCLUDE_NET_FARAPI_ERRNO_H */
