/****************************************************************************
 * examples/streaming/streaming_playlist.c
 *
 *   Copyright (C) 2017 Sony Corporation
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
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include "app_playlist.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static FILE *playlist_fp;
static char line_buffer[256];

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static char *app_streaming_playlist_readLine(void)
{
  size_t read_size = 1;
  char *tmp = line_buffer;
  int ret;

  while (1)
    {
      read_size = 1;

      ret = fread(tmp, read_size, 1, playlist_fp);
      if (ret < 0)
        {
          printf("failed to read the file. ret = %d\n",ret);
          return NULL;
        }

      if (ret == 0)
        {
          printf("Reached to Eof.\n");
          *tmp = '\0';
          fseek(playlist_fp, 0, SEEK_SET);
          break;
        }
      else
        {
          if (*tmp == 0x0d)
            {
              *tmp = '\0';
            }
          else if (*tmp == 0x0a)
            {
              *tmp = '\0';
              break;
            }

          tmp++;
        }
    }

  return line_buffer;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int app_streaming_playlist_open(void)
{
  playlist_fp = fopen("/mnt/spif/streaming_playlist.txt", "rb");
  if (playlist_fp == NULL)
    {
      printf("Playlist file open error\n");
      return 1;
    }

    return 0;
}

int app_streaming_playlist_close(void)
{
  if (fclose(playlist_fp))
    {
      printf("Playlist file close error\n");
      return 1;
    }

  return 0;
}

int app_streaming_playlist_getTrack(TrackInfo *track)
{
  char *line = app_streaming_playlist_readLine();

  if (line[0] == '\0')
    {
      return 1;
    }

  printf("track info: %s\n", line);

  char *idx_top;
  char *idx_tail;

  memset(track, 0, sizeof(TrackInfo));

  /* http or https */
  idx_tail = strstr(line, ":");
  track->is_https = (strstr(line, "https:") == NULL) ? 0 : 1;

  /* domain */
  idx_top  = idx_tail + 3;
  idx_tail = strstr(idx_top,",");
  strncpy(track->domain, idx_top, (size_t)(idx_tail - idx_top));

  /* path */
  idx_top  = idx_tail + 1;
  idx_tail = strstr(idx_top,",");
  strncpy(track->path, idx_top, (size_t)(idx_tail - idx_top));

  /* codec */
  idx_tail = strstr(idx_top,".");
  track->codec_type = (strstr( idx_top, "mp3") == NULL) ? AS_CODECTYPE_WAV : AS_CODECTYPE_MP3;

  /* title */
  idx_top  = idx_tail + 1;
  idx_tail = strstr(idx_top,",");
  strcpy(track->title, idx_tail + 1);

  return 0;
}
