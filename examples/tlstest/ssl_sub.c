/****************************************************************************
 * examples/tlstest/ssl_sub.c
 *
 *   Copyright 2018 Sony Corporation
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <mbedtls/config.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/net.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/platform.h>
#include <mbedtls/ssl.h>

#include "ssl_sub.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TLS_MAX_SOCKETS     4
#define TLS_MAX_SESSIONS    6
#define TLS_RANDOM_SEED     "MBEDTLS_SSL_RANDOM_SEED"
#define TLS_READ_TIMEOUT    10000
#define TLS_CRTINFO_BUFSIZE 1024

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
  {
    int tcp_socket;
    mbedtls_ssl_context *tls_context;
    mbedtls_net_context tls_net_context;
  } tls_socket_t;

typedef struct
  {
    mbedtls_ssl_session *session;
    char *host;
  } tls_session_t;

typedef struct
  {
    const unsigned char *cert_data;
    size_t cert_size;
  } rootca_cert_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int ssl_status = 0;
static mbedtls_entropy_context ssl_entropy;
static mbedtls_ctr_drbg_context ssl_ctr_drbg;
static mbedtls_ssl_config ssl_conf;
static mbedtls_x509_crt ssl_ca;
//static mbedtls_x509_crt ssl_cli;
static mbedtls_pk_context ssl_pkey;
//static mbedtls_x509_crl ssl_crl;
static tls_socket_t ssl_sockets[TLS_MAX_SOCKETS];
static tls_session_t ssl_sessions[TLS_MAX_SESSIONS];
static int cache_enabled = 1;

/* subject:/C=US/O=GeoTrust Inc./CN=GeoTrust Global CA */
/* issuer :/C=US/O=GeoTrust Inc./CN=GeoTrust Global CA */
static const unsigned char GeoTrustGlobalCA_certificate[856]={
0x30,0x82,0x03,0x54,0x30,0x82,0x02,0x3C,0xA0,0x03,0x02,0x01,0x02,0x02,0x03,0x02,
0x34,0x56,0x30,0x0D,0x06,0x09,0x2A,0x86,0x48,0x86,0xF7,0x0D,0x01,0x01,0x05,0x05,
0x00,0x30,0x42,0x31,0x0B,0x30,0x09,0x06,0x03,0x55,0x04,0x06,0x13,0x02,0x55,0x53,
0x31,0x16,0x30,0x14,0x06,0x03,0x55,0x04,0x0A,0x13,0x0D,0x47,0x65,0x6F,0x54,0x72,
0x75,0x73,0x74,0x20,0x49,0x6E,0x63,0x2E,0x31,0x1B,0x30,0x19,0x06,0x03,0x55,0x04,
0x03,0x13,0x12,0x47,0x65,0x6F,0x54,0x72,0x75,0x73,0x74,0x20,0x47,0x6C,0x6F,0x62,
0x61,0x6C,0x20,0x43,0x41,0x30,0x1E,0x17,0x0D,0x30,0x32,0x30,0x35,0x32,0x31,0x30,
0x34,0x30,0x30,0x30,0x30,0x5A,0x17,0x0D,0x32,0x32,0x30,0x35,0x32,0x31,0x30,0x34,
0x30,0x30,0x30,0x30,0x5A,0x30,0x42,0x31,0x0B,0x30,0x09,0x06,0x03,0x55,0x04,0x06,
0x13,0x02,0x55,0x53,0x31,0x16,0x30,0x14,0x06,0x03,0x55,0x04,0x0A,0x13,0x0D,0x47,
0x65,0x6F,0x54,0x72,0x75,0x73,0x74,0x20,0x49,0x6E,0x63,0x2E,0x31,0x1B,0x30,0x19,
0x06,0x03,0x55,0x04,0x03,0x13,0x12,0x47,0x65,0x6F,0x54,0x72,0x75,0x73,0x74,0x20,
0x47,0x6C,0x6F,0x62,0x61,0x6C,0x20,0x43,0x41,0x30,0x82,0x01,0x22,0x30,0x0D,0x06,
0x09,0x2A,0x86,0x48,0x86,0xF7,0x0D,0x01,0x01,0x01,0x05,0x00,0x03,0x82,0x01,0x0F,
0x00,0x30,0x82,0x01,0x0A,0x02,0x82,0x01,0x01,0x00,0xDA,0xCC,0x18,0x63,0x30,0xFD,
0xF4,0x17,0x23,0x1A,0x56,0x7E,0x5B,0xDF,0x3C,0x6C,0x38,0xE4,0x71,0xB7,0x78,0x91,
0xD4,0xBC,0xA1,0xD8,0x4C,0xF8,0xA8,0x43,0xB6,0x03,0xE9,0x4D,0x21,0x07,0x08,0x88,
0xDA,0x58,0x2F,0x66,0x39,0x29,0xBD,0x05,0x78,0x8B,0x9D,0x38,0xE8,0x05,0xB7,0x6A,
0x7E,0x71,0xA4,0xE6,0xC4,0x60,0xA6,0xB0,0xEF,0x80,0xE4,0x89,0x28,0x0F,0x9E,0x25,
0xD6,0xED,0x83,0xF3,0xAD,0xA6,0x91,0xC7,0x98,0xC9,0x42,0x18,0x35,0x14,0x9D,0xAD,
0x98,0x46,0x92,0x2E,0x4F,0xCA,0xF1,0x87,0x43,0xC1,0x16,0x95,0x57,0x2D,0x50,0xEF,
0x89,0x2D,0x80,0x7A,0x57,0xAD,0xF2,0xEE,0x5F,0x6B,0xD2,0x00,0x8D,0xB9,0x14,0xF8,
0x14,0x15,0x35,0xD9,0xC0,0x46,0xA3,0x7B,0x72,0xC8,0x91,0xBF,0xC9,0x55,0x2B,0xCD,
0xD0,0x97,0x3E,0x9C,0x26,0x64,0xCC,0xDF,0xCE,0x83,0x19,0x71,0xCA,0x4E,0xE6,0xD4,
0xD5,0x7B,0xA9,0x19,0xCD,0x55,0xDE,0xC8,0xEC,0xD2,0x5E,0x38,0x53,0xE5,0x5C,0x4F,
0x8C,0x2D,0xFE,0x50,0x23,0x36,0xFC,0x66,0xE6,0xCB,0x8E,0xA4,0x39,0x19,0x00,0xB7,
0x95,0x02,0x39,0x91,0x0B,0x0E,0xFE,0x38,0x2E,0xD1,0x1D,0x05,0x9A,0xF6,0x4D,0x3E,
0x6F,0x0F,0x07,0x1D,0xAF,0x2C,0x1E,0x8F,0x60,0x39,0xE2,0xFA,0x36,0x53,0x13,0x39,
0xD4,0x5E,0x26,0x2B,0xDB,0x3D,0xA8,0x14,0xBD,0x32,0xEB,0x18,0x03,0x28,0x52,0x04,
0x71,0xE5,0xAB,0x33,0x3D,0xE1,0x38,0xBB,0x07,0x36,0x84,0x62,0x9C,0x79,0xEA,0x16,
0x30,0xF4,0x5F,0xC0,0x2B,0xE8,0x71,0x6B,0xE4,0xF9,0x02,0x03,0x01,0x00,0x01,0xA3,
0x53,0x30,0x51,0x30,0x0F,0x06,0x03,0x55,0x1D,0x13,0x01,0x01,0xFF,0x04,0x05,0x30,
0x03,0x01,0x01,0xFF,0x30,0x1D,0x06,0x03,0x55,0x1D,0x0E,0x04,0x16,0x04,0x14,0xC0,
0x7A,0x98,0x68,0x8D,0x89,0xFB,0xAB,0x05,0x64,0x0C,0x11,0x7D,0xAA,0x7D,0x65,0xB8,
0xCA,0xCC,0x4E,0x30,0x1F,0x06,0x03,0x55,0x1D,0x23,0x04,0x18,0x30,0x16,0x80,0x14,
0xC0,0x7A,0x98,0x68,0x8D,0x89,0xFB,0xAB,0x05,0x64,0x0C,0x11,0x7D,0xAA,0x7D,0x65,
0xB8,0xCA,0xCC,0x4E,0x30,0x0D,0x06,0x09,0x2A,0x86,0x48,0x86,0xF7,0x0D,0x01,0x01,
0x05,0x05,0x00,0x03,0x82,0x01,0x01,0x00,0x35,0xE3,0x29,0x6A,0xE5,0x2F,0x5D,0x54,
0x8E,0x29,0x50,0x94,0x9F,0x99,0x1A,0x14,0xE4,0x8F,0x78,0x2A,0x62,0x94,0xA2,0x27,
0x67,0x9E,0xD0,0xCF,0x1A,0x5E,0x47,0xE9,0xC1,0xB2,0xA4,0xCF,0xDD,0x41,0x1A,0x05,
0x4E,0x9B,0x4B,0xEE,0x4A,0x6F,0x55,0x52,0xB3,0x24,0xA1,0x37,0x0A,0xEB,0x64,0x76,
0x2A,0x2E,0x2C,0xF3,0xFD,0x3B,0x75,0x90,0xBF,0xFA,0x71,0xD8,0xC7,0x3D,0x37,0xD2,
0xB5,0x05,0x95,0x62,0xB9,0xA6,0xDE,0x89,0x3D,0x36,0x7B,0x38,0x77,0x48,0x97,0xAC,
0xA6,0x20,0x8F,0x2E,0xA6,0xC9,0x0C,0xC2,0xB2,0x99,0x45,0x00,0xC7,0xCE,0x11,0x51,
0x22,0x22,0xE0,0xA5,0xEA,0xB6,0x15,0x48,0x09,0x64,0xEA,0x5E,0x4F,0x74,0xF7,0x05,
0x3E,0xC7,0x8A,0x52,0x0C,0xDB,0x15,0xB4,0xBD,0x6D,0x9B,0xE5,0xC6,0xB1,0x54,0x68,
0xA9,0xE3,0x69,0x90,0xB6,0x9A,0xA5,0x0F,0xB8,0xB9,0x3F,0x20,0x7D,0xAE,0x4A,0xB5,
0xB8,0x9C,0xE4,0x1D,0xB6,0xAB,0xE6,0x94,0xA5,0xC1,0xC7,0x83,0xAD,0xDB,0xF5,0x27,
0x87,0x0E,0x04,0x6C,0xD5,0xFF,0xDD,0xA0,0x5D,0xED,0x87,0x52,0xB7,0x2B,0x15,0x02,
0xAE,0x39,0xA6,0x6A,0x74,0xE9,0xDA,0xC4,0xE7,0xBC,0x4D,0x34,0x1E,0xA9,0x5C,0x4D,
0x33,0x5F,0x92,0x09,0x2F,0x88,0x66,0x5D,0x77,0x97,0xC7,0x1D,0x76,0x13,0xA9,0xD5,
0xE5,0xF1,0x16,0x09,0x11,0x35,0xD5,0xAC,0xDB,0x24,0x71,0x70,0x2C,0x98,0x56,0x0B,
0xD9,0x17,0xB4,0xD1,0xE3,0x51,0x2B,0x5E,0x75,0xE8,0xD5,0xD0,0xDC,0x4F,0x34,0xED,
0xC2,0x05,0x66,0x80,0xA1,0xCB,0xE6,0x33,
};

/* subject:/C=BE/O=GlobalSign nv-sa/OU=Root CA/CN=GlobalSign Root CA */
/* issuer :/C=BE/O=GlobalSign nv-sa/OU=Root CA/CN=GlobalSign Root CA */
static const unsigned char GlobalSignRootCA_certificate[889]={
0x30,0x82,0x03,0x75,0x30,0x82,0x02,0x5D,0xA0,0x03,0x02,0x01,0x02,0x02,0x0B,0x04,
0x00,0x00,0x00,0x00,0x01,0x15,0x4B,0x5A,0xC3,0x94,0x30,0x0D,0x06,0x09,0x2A,0x86,
0x48,0x86,0xF7,0x0D,0x01,0x01,0x05,0x05,0x00,0x30,0x57,0x31,0x0B,0x30,0x09,0x06,
0x03,0x55,0x04,0x06,0x13,0x02,0x42,0x45,0x31,0x19,0x30,0x17,0x06,0x03,0x55,0x04,
0x0A,0x13,0x10,0x47,0x6C,0x6F,0x62,0x61,0x6C,0x53,0x69,0x67,0x6E,0x20,0x6E,0x76,
0x2D,0x73,0x61,0x31,0x10,0x30,0x0E,0x06,0x03,0x55,0x04,0x0B,0x13,0x07,0x52,0x6F,
0x6F,0x74,0x20,0x43,0x41,0x31,0x1B,0x30,0x19,0x06,0x03,0x55,0x04,0x03,0x13,0x12,
0x47,0x6C,0x6F,0x62,0x61,0x6C,0x53,0x69,0x67,0x6E,0x20,0x52,0x6F,0x6F,0x74,0x20,
0x43,0x41,0x30,0x1E,0x17,0x0D,0x39,0x38,0x30,0x39,0x30,0x31,0x31,0x32,0x30,0x30,
0x30,0x30,0x5A,0x17,0x0D,0x32,0x38,0x30,0x31,0x32,0x38,0x31,0x32,0x30,0x30,0x30,
0x30,0x5A,0x30,0x57,0x31,0x0B,0x30,0x09,0x06,0x03,0x55,0x04,0x06,0x13,0x02,0x42,
0x45,0x31,0x19,0x30,0x17,0x06,0x03,0x55,0x04,0x0A,0x13,0x10,0x47,0x6C,0x6F,0x62,
0x61,0x6C,0x53,0x69,0x67,0x6E,0x20,0x6E,0x76,0x2D,0x73,0x61,0x31,0x10,0x30,0x0E,
0x06,0x03,0x55,0x04,0x0B,0x13,0x07,0x52,0x6F,0x6F,0x74,0x20,0x43,0x41,0x31,0x1B,
0x30,0x19,0x06,0x03,0x55,0x04,0x03,0x13,0x12,0x47,0x6C,0x6F,0x62,0x61,0x6C,0x53,
0x69,0x67,0x6E,0x20,0x52,0x6F,0x6F,0x74,0x20,0x43,0x41,0x30,0x82,0x01,0x22,0x30,
0x0D,0x06,0x09,0x2A,0x86,0x48,0x86,0xF7,0x0D,0x01,0x01,0x01,0x05,0x00,0x03,0x82,
0x01,0x0F,0x00,0x30,0x82,0x01,0x0A,0x02,0x82,0x01,0x01,0x00,0xDA,0x0E,0xE6,0x99,
0x8D,0xCE,0xA3,0xE3,0x4F,0x8A,0x7E,0xFB,0xF1,0x8B,0x83,0x25,0x6B,0xEA,0x48,0x1F,
0xF1,0x2A,0xB0,0xB9,0x95,0x11,0x04,0xBD,0xF0,0x63,0xD1,0xE2,0x67,0x66,0xCF,0x1C,
0xDD,0xCF,0x1B,0x48,0x2B,0xEE,0x8D,0x89,0x8E,0x9A,0xAF,0x29,0x80,0x65,0xAB,0xE9,
0xC7,0x2D,0x12,0xCB,0xAB,0x1C,0x4C,0x70,0x07,0xA1,0x3D,0x0A,0x30,0xCD,0x15,0x8D,
0x4F,0xF8,0xDD,0xD4,0x8C,0x50,0x15,0x1C,0xEF,0x50,0xEE,0xC4,0x2E,0xF7,0xFC,0xE9,
0x52,0xF2,0x91,0x7D,0xE0,0x6D,0xD5,0x35,0x30,0x8E,0x5E,0x43,0x73,0xF2,0x41,0xE9,
0xD5,0x6A,0xE3,0xB2,0x89,0x3A,0x56,0x39,0x38,0x6F,0x06,0x3C,0x88,0x69,0x5B,0x2A,
0x4D,0xC5,0xA7,0x54,0xB8,0x6C,0x89,0xCC,0x9B,0xF9,0x3C,0xCA,0xE5,0xFD,0x89,0xF5,
0x12,0x3C,0x92,0x78,0x96,0xD6,0xDC,0x74,0x6E,0x93,0x44,0x61,0xD1,0x8D,0xC7,0x46,
0xB2,0x75,0x0E,0x86,0xE8,0x19,0x8A,0xD5,0x6D,0x6C,0xD5,0x78,0x16,0x95,0xA2,0xE9,
0xC8,0x0A,0x38,0xEB,0xF2,0x24,0x13,0x4F,0x73,0x54,0x93,0x13,0x85,0x3A,0x1B,0xBC,
0x1E,0x34,0xB5,0x8B,0x05,0x8C,0xB9,0x77,0x8B,0xB1,0xDB,0x1F,0x20,0x91,0xAB,0x09,
0x53,0x6E,0x90,0xCE,0x7B,0x37,0x74,0xB9,0x70,0x47,0x91,0x22,0x51,0x63,0x16,0x79,
0xAE,0xB1,0xAE,0x41,0x26,0x08,0xC8,0x19,0x2B,0xD1,0x46,0xAA,0x48,0xD6,0x64,0x2A,
0xD7,0x83,0x34,0xFF,0x2C,0x2A,0xC1,0x6C,0x19,0x43,0x4A,0x07,0x85,0xE7,0xD3,0x7C,
0xF6,0x21,0x68,0xEF,0xEA,0xF2,0x52,0x9F,0x7F,0x93,0x90,0xCF,0x02,0x03,0x01,0x00,
0x01,0xA3,0x42,0x30,0x40,0x30,0x0E,0x06,0x03,0x55,0x1D,0x0F,0x01,0x01,0xFF,0x04,
0x04,0x03,0x02,0x01,0x06,0x30,0x0F,0x06,0x03,0x55,0x1D,0x13,0x01,0x01,0xFF,0x04,
0x05,0x30,0x03,0x01,0x01,0xFF,0x30,0x1D,0x06,0x03,0x55,0x1D,0x0E,0x04,0x16,0x04,
0x14,0x60,0x7B,0x66,0x1A,0x45,0x0D,0x97,0xCA,0x89,0x50,0x2F,0x7D,0x04,0xCD,0x34,
0xA8,0xFF,0xFC,0xFD,0x4B,0x30,0x0D,0x06,0x09,0x2A,0x86,0x48,0x86,0xF7,0x0D,0x01,
0x01,0x05,0x05,0x00,0x03,0x82,0x01,0x01,0x00,0xD6,0x73,0xE7,0x7C,0x4F,0x76,0xD0,
0x8D,0xBF,0xEC,0xBA,0xA2,0xBE,0x34,0xC5,0x28,0x32,0xB5,0x7C,0xFC,0x6C,0x9C,0x2C,
0x2B,0xBD,0x09,0x9E,0x53,0xBF,0x6B,0x5E,0xAA,0x11,0x48,0xB6,0xE5,0x08,0xA3,0xB3,
0xCA,0x3D,0x61,0x4D,0xD3,0x46,0x09,0xB3,0x3E,0xC3,0xA0,0xE3,0x63,0x55,0x1B,0xF2,
0xBA,0xEF,0xAD,0x39,0xE1,0x43,0xB9,0x38,0xA3,0xE6,0x2F,0x8A,0x26,0x3B,0xEF,0xA0,
0x50,0x56,0xF9,0xC6,0x0A,0xFD,0x38,0xCD,0xC4,0x0B,0x70,0x51,0x94,0x97,0x98,0x04,
0xDF,0xC3,0x5F,0x94,0xD5,0x15,0xC9,0x14,0x41,0x9C,0xC4,0x5D,0x75,0x64,0x15,0x0D,
0xFF,0x55,0x30,0xEC,0x86,0x8F,0xFF,0x0D,0xEF,0x2C,0xB9,0x63,0x46,0xF6,0xAA,0xFC,
0xDF,0xBC,0x69,0xFD,0x2E,0x12,0x48,0x64,0x9A,0xE0,0x95,0xF0,0xA6,0xEF,0x29,0x8F,
0x01,0xB1,0x15,0xB5,0x0C,0x1D,0xA5,0xFE,0x69,0x2C,0x69,0x24,0x78,0x1E,0xB3,0xA7,
0x1C,0x71,0x62,0xEE,0xCA,0xC8,0x97,0xAC,0x17,0x5D,0x8A,0xC2,0xF8,0x47,0x86,0x6E,
0x2A,0xC4,0x56,0x31,0x95,0xD0,0x67,0x89,0x85,0x2B,0xF9,0x6C,0xA6,0x5D,0x46,0x9D,
0x0C,0xAA,0x82,0xE4,0x99,0x51,0xDD,0x70,0xB7,0xDB,0x56,0x3D,0x61,0xE4,0x6A,0xE1,
0x5C,0xD6,0xF6,0xFE,0x3D,0xDE,0x41,0xCC,0x07,0xAE,0x63,0x52,0xBF,0x53,0x53,0xF4,
0x2B,0xE9,0xC7,0xFD,0xB6,0xF7,0x82,0x5F,0x85,0xD2,0x41,0x18,0xDB,0x81,0xB3,0x04,
0x1C,0xC5,0x1F,0xA4,0x80,0x6F,0x15,0x20,0xC9,0xDE,0x0C,0x88,0x0A,0x1D,0xD6,0x66,
0x55,0xE2,0xFC,0x48,0xC9,0x29,0x26,0x69,0xE0,
};

static size_t num_rootca_certs = 2;

static const rootca_cert_t rootca_certs[] =
  {
    {GeoTrustGlobalCA_certificate, 856},
    {GlobalSignRootCA_certificate, 889},
  };


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static size_t rootca_number_of_certificates(void)
{
  return num_rootca_certs;
}

static const unsigned char *rootca_get_certificate(size_t index)
{
  if (index >= num_rootca_certs)
    {
      return NULL;
    }
  return rootca_certs[index].cert_data;
}

static size_t rootca_get_certificate_size(size_t index)
{
  if (index >= num_rootca_certs)
    {
      return 0;
    }
  return rootca_certs[index].cert_size;
}


static mbedtls_ssl_session *tls_session_find(const char *host)
{
  if (cache_enabled)
    {
      for (size_t i = 0; i < TLS_MAX_SESSIONS; i++)
        {
          if ((ssl_sessions[i].session != NULL)
              && (strncmp(host, ssl_sessions[i].host, strlen(host)) == 0))
            {
              printf("tls_session_find : [%d]\n", i);
              return ssl_sessions[i].session;
            }
        }
    }
  return NULL;
}

static void tls_session_update(mbedtls_ssl_context *ctx, const char *host)
{
  int idx = -1;

  if (!cache_enabled)
    {
      return;
    }

  // Find an empty slot or a matching entry
  for (size_t i = 0; i < TLS_MAX_SESSIONS; i++)
    {
      if (ssl_sessions[i].session == NULL)
        {
          if (idx < 0)
            {
              idx = i;
            }
        }
      else if (strncmp(host, ssl_sessions[i].host, strlen(host)) == 0)
        {
          idx = i;
          break;
        }
    }
  if (idx < 0)
    {
      // Table is full, and no entry matches. Delete the oldest entry (at index 0).
      mbedtls_ssl_session_free(ssl_sessions[0].session);
      free(ssl_sessions[0].session);
      free(ssl_sessions[0].host);
      for (size_t i = 0; i < TLS_MAX_SESSIONS - 1; i++)
        {
          memmove(&ssl_sessions[i], &ssl_sessions[i+1], sizeof(tls_session_t));
        }
      idx = TLS_MAX_SESSIONS - 1;
      ssl_sessions[idx].session = NULL;
      ssl_sessions[idx].host = NULL;
    }

  if (ssl_sessions[idx].session == NULL)
    {
      ssl_sessions[idx].session = calloc(1, sizeof(mbedtls_ssl_session));
    }
  else
    {
      mbedtls_ssl_session_free(ssl_sessions[idx].session);
    }
  mbedtls_ssl_session_init(ssl_sessions[idx].session);

  if (ssl_sessions[idx].host == NULL)
    {
      ssl_sessions[idx].host = calloc(1, strlen(host));
      strncpy(ssl_sessions[idx].host, host, strlen(host));
    }

  if (mbedtls_ssl_get_session(ctx, ssl_sessions[idx].session))
    {
      mbedtls_ssl_session_free(ssl_sessions[idx].session);
      free(ssl_sessions[idx].session);
      ssl_sessions[idx].session = NULL;
      free(ssl_sessions[idx].host);
      ssl_sessions[idx].host = NULL;
      return;
  }

  printf("tls_session_update : [%d]\n", idx);
}

static int ssl_verify_cert(void *data, mbedtls_x509_crt *crt, int depth, uint32_t *flags)
{
  int ret;
  char buf[TLS_CRTINFO_BUFSIZE];

  printf("Verify requested for depth %d\n", depth);
  memset(buf, 0, TLS_CRTINFO_BUFSIZE);
  ret = mbedtls_x509_crt_info(buf, TLS_CRTINFO_BUFSIZE, "", crt);
  printf("CrtInfo [%d bytes] :\n", ret);
  printf("%s", buf);

  if ((*flags) == 0)
    {
      printf("This certificate has no flags\n");
    }
  else
    {
      printf("This certificate has flags : %d\n", *flags);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void ssl_init(const char** cafile, int verify)
{
  int ret;
  size_t cnt;

  if (ssl_status)
    {
      return;
    }    

  // Initialize
  mbedtls_ctr_drbg_init(&ssl_ctr_drbg);
  mbedtls_ssl_config_init(&ssl_conf);
  mbedtls_entropy_init(&ssl_entropy);
  mbedtls_x509_crt_init(&ssl_ca);
//  mbedtls_x509_crt_init(&ssl_cli);
  mbedtls_pk_init(&ssl_pkey);
//  mbedtls_x509_crl_init(&ssl_crl);

  for (cnt=0; cnt<TLS_MAX_SOCKETS; cnt++)
    {
      ssl_sockets[cnt].tcp_socket = -1;
      ssl_sockets[cnt].tls_context = NULL;
      ssl_sockets[cnt].tls_net_context.fd = -1;
    }
  for (cnt=0; cnt<TLS_MAX_SESSIONS; cnt++)
    {
      ssl_sessions[cnt].session = NULL;
      ssl_sessions[cnt].host = NULL;
    }

  // Set up for random bits generation
  if ((ret = mbedtls_ctr_drbg_seed(&ssl_ctr_drbg, NULL, &ssl_entropy,
       (const unsigned char*)TLS_RANDOM_SEED, strlen(TLS_RANDOM_SEED))) != 0)
    {
      printf("mbedtls_ctr_drbg_seed error : -0x%x\n", -ret);
      ssl_fin();
      return;
    }
  mbedtls_ssl_conf_rng(&ssl_conf, NULL, &ssl_ctr_drbg);

  // General configuration
  printf("mbedtls_ssl_config_defaults : CLIENT = %d\n", MBEDTLS_SSL_IS_CLIENT);
  if ((ret = mbedtls_ssl_config_defaults(&ssl_conf, MBEDTLS_SSL_IS_CLIENT,
       MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT)) != 0)
    {
      printf("mbedtls_ssl_config_defaults error : -0x%x\n", -ret);
      ssl_fin();
      return;
    }

  // Install the root certificate(s), and set up for server authentication
  if (cafile == NULL)
    {
      for (cnt=0; cnt<rootca_number_of_certificates(); cnt++)
        {
          if ((ret = mbedtls_x509_crt_parse_der(&ssl_ca, rootca_get_certificate(cnt),
               rootca_get_certificate_size(cnt))) != 0)
            {
              printf("mbedtls_x509_crt_parse_der error : -0x%x\n", -ret);
              ssl_fin();
              return;
            }
        }
    }
  else
    {
      while (*cafile != NULL)
        {
          printf("mbedtls root certificate : add [%s]\n", *cafile);
          if ((ret = mbedtls_x509_crt_parse_file(&ssl_ca, *cafile)) != 0)
            {
              printf("mbedtls_x509_crt_parse_file error : -0x%x\n", -ret);
              ssl_fin();
              return;
            }
          cafile++;
        }
    }
  mbedtls_ssl_conf_ca_chain(&ssl_conf, &ssl_ca, NULL);

  switch (verify)
    {
      case TLSTEST_SSL_VERIFY_REQUIRED:
        {
          mbedtls_ssl_conf_authmode(&ssl_conf, MBEDTLS_SSL_VERIFY_REQUIRED);
          break;
        }
      case TLSTEST_SSL_VERIFY_CALLBACK:
        {
          mbedtls_ssl_conf_authmode(&ssl_conf, MBEDTLS_SSL_VERIFY_OPTIONAL);
          mbedtls_ssl_conf_verify(&ssl_conf, ssl_verify_cert, NULL);
          break;
        }
      default:
        {
          mbedtls_ssl_conf_authmode(&ssl_conf, MBEDTLS_SSL_VERIFY_NONE);
          break;
        }
    }

  // Set timeout
  mbedtls_ssl_conf_read_timeout(&ssl_conf, TLS_READ_TIMEOUT);

  printf("ssl_init : success\n");
  ssl_status = 1;
  return;
}


void ssl_fin()
{
  size_t cnt;

  for (cnt=0; cnt<TLS_MAX_SOCKETS; cnt++)
    {
      if (ssl_sockets[cnt].tcp_socket >= 0)
        {
          printf("ssl_fin socket released : [%d]\n", cnt);
          ssl_disconnect(cnt);
        }
    }
  for (cnt=0; cnt<TLS_MAX_SESSIONS; cnt++)
    {
      if (ssl_sessions[cnt].session != NULL)
        {
          mbedtls_ssl_session_free(ssl_sessions[cnt].session);
          free(ssl_sessions[cnt].session);
          ssl_sessions[cnt].session = NULL;
          printf("ssl_fin session released : [%d]\n", cnt);
        }
      if (ssl_sessions[cnt].host != NULL)
        {
          free(ssl_sessions[cnt].host);
          ssl_sessions[cnt].host = NULL;
        }
    }

  mbedtls_ssl_config_free(&ssl_conf);
  mbedtls_ctr_drbg_free(&ssl_ctr_drbg);
  mbedtls_entropy_free(&ssl_entropy);
  mbedtls_x509_crt_free(&ssl_ca);
//  mbedtls_x509_crt_free(&ssl_cli);
  mbedtls_pk_free(&ssl_pkey);
//  mbedtls_x509_crl_free(&ssl_crl);

  ssl_status = 0;

  printf("ssl_fin : success\n");
  return;
}


int ssl_disconnect(int sock)
{
  int ret;

  if ((sock < 0) || (sock >= TLS_MAX_SOCKETS) || (ssl_sockets[sock].tcp_socket < 0))
    {
      return -EPERM;
    }
  tls_socket_t *socket = &ssl_sockets[sock];

  if (socket->tls_context != NULL)
    {
      do
        {
          ret = mbedtls_ssl_close_notify(socket->tls_context);
        } while (ret == MBEDTLS_ERR_SSL_WANT_WRITE);

      mbedtls_net_free(&socket->tls_net_context);
      mbedtls_ssl_free(socket->tls_context);
      free(socket->tls_context);
      socket->tls_context = NULL;
      socket->tcp_socket = -1;
    }

  printf("ssl_disconnect [%d] success\n", sock);
  return 0;
}


int ssl_connect(int tcp, const char *host, const char *port, int *sock)
{
  int ret;
  int cnt;
  tls_socket_t *socket = NULL;

  for (cnt=0; cnt<TLS_MAX_SOCKETS; cnt++)
    {
      socket = &ssl_sockets[cnt];
      if (socket->tcp_socket < 0)
        {
          break;
        }
    }
  if (cnt >= TLS_MAX_SOCKETS)
    {
      printf("No more socket is available.\n");
      return -ENFILE;
    }

  mbedtls_ssl_context *tls_context = calloc(1, sizeof(mbedtls_ssl_context));
  if (tls_context == NULL)
    {
      printf("No more memory is available.\n");
      return -ENOMEM;
    }

  mbedtls_ssl_init(tls_context);
  if ((ret = mbedtls_ssl_setup(tls_context, &ssl_conf)) != 0)
    {
      printf("mbedtls_ssl_setup error : -0x%x\n", -ret);
      mbedtls_ssl_free(tls_context);
      free(tls_context);
      return ret;
    }

  if ((ret = mbedtls_net_connect(&socket->tls_net_context, host, port, tcp)) != 0)
    {
      printf("mbedtls_net_connect error : -0x%x\n", -ret);
      mbedtls_net_free(&socket->tls_net_context);
      mbedtls_ssl_free(tls_context);
      free(tls_context);
      return ret;
    }
  socket->tls_context = tls_context;
  socket->tcp_socket = socket->tls_net_context.fd;

  if ((ret = mbedtls_ssl_set_hostname(tls_context, host)) != 0)
    {
      printf("mbedtls_ssl_set_hostname error : -0x%x\n", -ret);
      ssl_disconnect(cnt);
      return ret;
    }
  mbedtls_ssl_set_bio(tls_context, &socket->tls_net_context, NULL, NULL, NULL);

  mbedtls_ssl_session *session = tls_session_find(host);
  if (session != NULL)
    {
      mbedtls_ssl_set_session(tls_context, session);
    }
  while ((ret = mbedtls_ssl_handshake(tls_context)) != 0)
    {
      if ((ret != MBEDTLS_ERR_SSL_WANT_READ) && (ret != MBEDTLS_ERR_SSL_WANT_WRITE))
        {
          printf("mbedtls_ssl_handshake error : -0x%x\n", -ret);
          ssl_disconnect(cnt);
          return ret;
        }
    }

  if ((ret = mbedtls_ssl_get_verify_result(tls_context)) != 0)
    {
      printf("mbedtls_ssl_get_verify_result error : -0x%x\n", -ret);
      ssl_disconnect(cnt);
      return ret;
    }

  tls_session_update(tls_context, host);

  printf("ssl_connect [%d/%d] success\n", cnt, socket->tcp_socket);
  *sock = cnt;

  return 0;
}


int ssl_recv(int sock, char *buf, size_t len)
{
  if ((sock < 0 || (sock >= TLS_MAX_SOCKETS)
                || (ssl_sockets[sock].tcp_socket < 0)
                || ssl_sockets[sock].tls_context == NULL))
    {
      return -EPERM;
    }
  return mbedtls_ssl_read(ssl_sockets[sock].tls_context, (unsigned char *)buf, len);
}


int ssl_send(int sock, const char *buf, size_t len)
{
  if ((sock < 0 || (sock >= TLS_MAX_SOCKETS)
                || (ssl_sockets[sock].tcp_socket < 0)
                || ssl_sockets[sock].tls_context == NULL))
    {
      return -EPERM;
    }
  return mbedtls_ssl_write(ssl_sockets[sock].tls_context, (unsigned char *)buf, len);
}


int ssl_sendrecv(int sock, int get, const char* host, const char* path, const char* data)
{
  int ret;
  int total = 0;
  char buf[TLSTEST_SSL_BUFFER_SIZE];

  if ((sock < 0 || (sock >= TLS_MAX_SOCKETS)
                || (ssl_sockets[sock].tcp_socket < 0)
                || ssl_sockets[sock].tls_context == NULL))
    {
      printf("ssl_sendrecv : illegal argument\n");
      return -EPERM;
    }

  /* Send and return code check */
  if (get == TLSTEST_SSL_GET)
    {
      snprintf(buf, TLSTEST_SSL_BUFFER_SIZE,
               "GET %s HTTP/1.0\r\nHOST: %s\r\nConnection: close\r\n\r\n",
               path, host);
      printf("Network send (GET) : %s\n", host);
    }
  else if (get == TLSTEST_SSL_POST)
    {
      snprintf(buf, TLSTEST_SSL_BUFFER_SIZE,
               "POST %s HTTP/1.0\r\nHOST: %s\r\nConnection: close\r\nContent-Length: %d\r\n\r\n",
               path, host, strlen(data));
      strncat(buf, data, TLSTEST_SSL_BUFFER_SIZE);
      strncat(buf, "\r\n", TLSTEST_SSL_BUFFER_SIZE);
      printf("Network send (POST) : %s\n", host);
    }
  else if (get == TLSTEST_SSL_HEAD)
    {
      snprintf(buf, TLSTEST_SSL_BUFFER_SIZE,
               "HEAD %s HTTP/1.0\r\nHOST: %s\r\nConnection: close\r\n\r\n",
               path, host);
      printf("Network send (HEAD) : %s\n", host);
    }
  else {
      printf("ssl_sendrecv : illegal argument\n");
      return -EPERM;
  }

  ret = ssl_send(sock, buf, strlen(buf));
  if (ret < 0)
    {
      printf("ssl_send error : -0x%x\n", -ret);
      return ret;
    }

  while (1)
    {
      memset(buf, 0, TLSTEST_SSL_BUFFER_SIZE);
      ret = ssl_recv(sock, buf, TLSTEST_SSL_BUFFER_SIZE);
      if (ret < 0)
        {
          if (ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY)
//          if ((ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY) || (ret == MBEDTLS_ERR_SSL_BAD_INPUT_DATA))
            {
              break;
            }
          if ((ret == MBEDTLS_ERR_SSL_WANT_READ) || (ret == MBEDTLS_ERR_SSL_WANT_WRITE))
            {
              continue;
            }
          printf("ssl_recv error : -0x%x\n", -ret);
          return ret;
        }
      if (ret == 0)
        {
          break;
        }
//      printf("%s", buf);
      total += ret;
    }

  return total;
}


