/****************************************************************************
 * examples/http_post/http_post_main.c
 *
 *   Copyright (C) 2017 Sony Corporation. All rights reserved.
 *   Author: Yutaka Miyajima <Yutaka.Miyajima@sony.com>
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
#include <stdlib.h>
#include <fcntl.h>

#include <lte/lte_client.h>

#include <netutils/httpc/http_socket.h>
#include <netutils/httpc/tls_socket.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BUF_SIZE               2048
#define SECURE_OPT_POSITION    1
#define SECURE_OPT_STRING      "-s"
#define SECURE_OPT_LEN         (strlen(SECURE_OPT_STRING))
#define FILE_OPT_STRING        "-f"
#define FILE_OPT_LEN           (strlen(FILE_OPT_STRING))
#define DATA_OPT_STRING        "-d"
#define DATA_OPT_LEN           (strlen(DATA_OPT_STRING))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char buf[BUF_SIZE];
static char line[BUF_SIZE + 1];

static int http_socket = -1;
struct http_chunk_client chunk_data_static;

static int need_free = 0;

static void lte_setup(void)
{
  /* Boot the modem and setup internal network */

  lte_start();

  /* Connect to the LTE network */

  lte_connect();
}


static size_t print_lines(const char *buffer)
{
  const char *p = buffer;
  const char *s;
  size_t n;

  while(1)
    {
      s = strchr(p, '\n');
      if (s == NULL)
        {
          n = strnlen(p, BUF_SIZE);
          memcpy(line, p, n);
          line[n] = '\0';
          printf("%s", line);
          break;
        }
      n = s - p;
      memcpy(line, p, n);
      line[n] = '\0';
      p = s + 1;
      printf("%s", line);
    }
  return (p - buffer);
}

static int get_file_size(char* file_name)
{
  FILE *fp;
  int file_size = 0;
  int ret;
  char tmp_buf;

  /* open file */

  fp = fopen(file_name, "rb");
  if (fp == NULL)
    {
      printf("%s open error\n", file_name);
    }
  else
    {
      /* seek EOF using fread() because fseek(SEEK_END) is not supported */

      while(1)
        {
          ret = fread(&tmp_buf, 1, 1, fp);
          if (ret <= 0)
            {
              break;
            }
          file_size += ret;
        }

      /* close file */

      fclose(fp);
    }

  return file_size;
}

static char* get_data_from_file(char* file_name)
{
  int size;
  char *data = NULL;
  FILE *fp;

  /* get specified file size */

  size = get_file_size(file_name);
  if (size <= 0)
    {
      return NULL;
    }

  printf("%s file size = %d\n", file_name, size);

  data = (char*)malloc(size);
  if (data == NULL)
    {
      printf("memory allocate failed\n");
      return NULL;
    }

  /* open file */

  fp = fopen(file_name, "rb");
  if (fp == NULL)
    {
      printf("%s open error\n", file_name);
      free(data);
      data = NULL;
    }
  else
    {
      need_free = 1;

      /* read data from file */

      fread(data, size, 1, fp);

      /* close file */

      fclose(fp);
    }

  return data;
}

static void chunked_recv(struct http_chunk_client *chunk_data)
{
  if (chunk_data != NULL)
    {
      chunk_data->chunk_buf[chunk_data->chunk_readlen] = '\0';

      printf("%s", chunk_data->chunk_buf);
      if(chunk_data->chunk_size == 0)
        {
          NT_http_close(http_socket);
          http_socket = -1;
        }
    }
}


static void http_request(int show_content, int use_https, char *domain, int port, const char *path, const char *data, size_t data_len)
{
  int r;
  int status;
  size_t n = 0;

  if (http_socket < 0)
    {
      http_socket = NT_http_create(use_https);
      printf("HTTP socket created: %d\n", http_socket);
      if (http_socket < 0)
        {
          return;
        }

      if (NT_http_connect(http_socket, domain, port) < 0)
        {
          printf("HTTP socket connect failed\n");
          NT_http_close(http_socket);
          http_socket = -1;
          return;
        }
    }
  else
    {
      printf("HTTP running process: %d\n", http_socket);
      return;
    }

  NT_http_socket_recvtimeout(http_socket, 10);

  r = NT_http_post(http_socket, path, NULL, 0, data, data_len, &status);
  printf("HTTP socket post: %d\n", r);
  if (r < 0)
    {
      NT_http_close(http_socket);
      http_socket = -1;
      return;
    }

  printf("Response status code = %d\n", status);
  if(show_content)
    {
      chunk_data_static.chunk_buf = buf;
      chunk_data_static.chunk_buflen = BUF_SIZE;
      NT_http_add_listener(http_socket, &chunk_data_static, chunked_recv);
    }
  else
    {
      char *p = buf;
      for (;;) {
          r = NT_http_read_response(http_socket, p, BUF_SIZE);
          if (r <= 0) {
        break;
      }

      p[r] = '\0';
      if ((status >= 200) && (status < 300)) {
        printf("Received %4d/%d bytes\n", r, n);
      } else {
        print_lines(buf);
      }
      n += r;
    }
  }
  if (show_content)
    {
      printf("Received %d bytes\n", n);
    }
  else
    {
       NT_http_close(http_socket);
       http_socket = -1;
    }
}

static void do_close(void)
{
  if (http_socket >= 0)
    {
      NT_http_close(http_socket);
      http_socket = -1;
    }
}

static int get_secure_option(int argc, char *argv[], int posi)
{
  if (argc >= (posi + 1))
    {
      if ((strlen(argv[posi]) == SECURE_OPT_LEN) &&
          (0 == strncmp(argv[posi], SECURE_OPT_STRING, SECURE_OPT_LEN)))
        {
          return 1;
        }
    }

  return 0;
}

static char* get_domain(int argc, char *argv[], int posi)
{
  if (argc >= (posi + 1))
    {
      return argv[posi];
    }

  printf("too few arguments.\n");
  return NULL;
}

static int get_port_num(int argc, char *argv[], int posi)
{
  if (argc >= (posi + 1))
    {
      return atoi(argv[posi]);
    }

  printf("too few arguments.\n");
  return -1;
}

static char* get_path(int argc, char *argv[], int posi)
{
  if (argc >= (posi + 1))
    {
      return argv[posi];
    }

  printf("too few arguments.\n");
  return NULL;
}

static char* get_data(int argc, char *argv[], int posi)
{
  if (argc >= (posi + 2))
    {
      if ((strlen(argv[posi]) == FILE_OPT_LEN) &&
          (0 == strncmp(argv[posi], FILE_OPT_STRING, FILE_OPT_LEN)))
        {
          /* get data from file */

          return get_data_from_file(argv[posi+1]);
        }
      else if ((strlen(argv[posi]) == DATA_OPT_LEN) &&
          (0 == strncmp(argv[posi], DATA_OPT_STRING, DATA_OPT_LEN)))
        {
          /* get data from command line */

          return argv[posi+1];
        }

      printf("invalid arguments.\n");
      return NULL;
    }

  printf("too few arguments.\n");
  return NULL;
}

static void free_data(char* data)
{
  if (need_free)
    {
      free(data);
      need_free = 0;
    }
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int http_post_main(int argc, char *argv[])
#endif
{
  int use_https;
  char *domain;
  int port;
  char *path;
  char *data;
  int posi;
  size_t data_len;

  /* Get secure option */

  use_https = get_secure_option(argc, argv, SECURE_OPT_POSITION);
  if (use_https)
    {
      posi = SECURE_OPT_POSITION + 1;
    }
  else
    {
      posi = SECURE_OPT_POSITION;
    }

  /* Get domain */

  domain = get_domain(argc, argv, posi);
  if (domain == NULL)
    {
      return 0;
    }

  posi++;

  /* Get port number */

  port = get_port_num(argc, argv, posi);
  if (port < 0)
    {
      return 0;
    }

  posi++;

  /* Get path */

  path = get_path(argc, argv, posi);
  if (path == NULL)
    {
      return 0;
    }

  posi++;

  /* Get data */

  data = get_data(argc, argv, posi);
  if (data == NULL)
    {
      return 0;
    }

  data_len = strlen(data);

  posi++;

  /* Connect to the LTE network */

  lte_setup();

  NT_tls_socket_init();
  NT_http_init();

  printf("URL = %s://%s:%d%s\n", (use_https ? "https" : "http"), domain, port, path);

  http_request(0, use_https, domain, port, path, data, data_len);

  free_data(data);

  do_close();

  return 0;
}
