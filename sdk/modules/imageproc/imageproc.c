/****************************************************************************
 * sdk/modules/imageproc/imageproc.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Corporation nor the names of its contributors
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

#include <sdk/config.h>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <time.h>
#include <semaphore.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <sdk/debug.h>

#include <arch/chip/ge2d.h>
#include <imageproc/imageproc.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_IMAGEPROC_GEDEVNAME
#  define GEDEVNAME CONFIG_IMAGEPROC_GEDEVNAME
#else
#  define GEDEVNAME "/dev/ge"
#endif

#define CXD56_ROT_BASE      (CXD56_ADSP_BASE + 0x02101400)
#define ROT_INTR_STATUS     (CXD56_ROT_BASE  + 0x0000)
#define ROT_INTR_ENABLE     (CXD56_ROT_BASE  + 0x0004)
#define ROT_INTR_DISABLE    (CXD56_ROT_BASE  + 0x0008)
#define ROT_INTR_CLEAR      (CXD56_ROT_BASE  + 0x000C)
#define ROT_SET_DIRECTION   (CXD56_ROT_BASE  + 0x0014)
#define ROT_SET_SRC_HSIZE   (CXD56_ROT_BASE  + 0x0018)
#define ROT_SET_SRC_VSIZE   (CXD56_ROT_BASE  + 0x001C)
#define ROT_SET_SRC_ADDRESS (CXD56_ROT_BASE  + 0x0020)
#define ROT_SET_SRC_PITCH   (CXD56_ROT_BASE  + 0x0024)
#define ROT_SET_DST_ADDRESS (CXD56_ROT_BASE  + 0x0028)
#define ROT_SET_DST_PITCH   (CXD56_ROT_BASE  + 0x002C)
#define ROT_CONV_CTRL       (CXD56_ROT_BASE  + 0x0034)
#define ROT_RGB_ALIGNMENT   (CXD56_ROT_BASE  + 0x0038)
#define ROT_COMMAND         (CXD56_ROT_BASE  + 0x0010)

#define MSEL     1

/* Command code */

#define COPYCMD  0x4
#define ROPCMD   0x8
#define ABCMD    0xa

/* Command options */

#define SRC16BPP (1 << 10)
#define SCALING  (1 << 12)
#define PATMONO  (1 << 15)

/* Raster operation code */

#define SRCCOPY     0xcc
#define SRCPAINT    0xee
#define SRCAND      0x88
#define SRCINVERT   0x66
#define SRCERASE    0x44
#define NOTSRCCOPY  0x33
#define NOTSRCERASE 0x11
#define MARGECOPY   0xc0
#define MERGEPAINT  0xbb
#define PATCOPY     0xf0
#define PATPAINT    0xfb
#define PATINVERT   0x5a
#define DSTINVERT   0x55

/* Raster operation options */

#define CONV8BPP    (1 << 7)
#define FIXEDCOLOR  (1 << 3)
#define CMPDST      (3 << 1)
#define CMPSRC      (2 << 1)
#define CMPPAT      (1 << 1)
#define WBUNMATCHED (1 << 0)

/* Alpha blending options */

#define ALPHA1BPP   (1 << 15)
#define FIXEDSRC    (1 << 14)
#define MSBFIRST    (1 << 13)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Copy command (32 bytes) */

struct ge2d_copycmd_s
{
  uint32_t cmd;                 /* 0x00 */
  uint16_t srch;                /* 0x04 */
  uint16_t srcv;                /* 0x06 */
  uint32_t saddr;               /* 0x08 */
  uint32_t daddr;               /* 0x0c */
  uint16_t spitch;              /* 0x10 */
  uint16_t dpitch;              /* 0x12 */
  uint32_t reserved[3];
} __attribute__((aligned(16)));

/* Raster operation (ROP) command (48 bytes) */

struct ge2d_ropcmd_s
{
  uint16_t cmd;                 /* 0x00 */
  uint8_t  rop;                 /* 0x02 */
  uint8_t  options;             /* 0x03 */
  uint16_t srch;                /* 0x04 */
  uint16_t srcv;                /* 0x06 */
  uint32_t saddr;               /* 0x08 */
  uint32_t daddr;               /* 0x0c */
  uint16_t spitch;              /* 0x10 */
  uint16_t dpitch;              /* 0x12 */

  uint32_t fixedcolor;          /* 0x14 */
  uint32_t pataddr;             /* 0x18 */
  uint16_t patpitch;            /* 0x1c */
  uint8_t  pathoffset;          /* 0x1e */
  uint8_t  patvoffset;          /* 0x1f */

  uint16_t desth;               /* 0x20 */
  uint16_t destv;               /* 0x22 */
  uint16_t ratioh;              /* 0x24 */
  uint16_t ratiov;              /* 0x26 */

  uint8_t  hphaseinit;          /* 0x28 */
  uint8_t  hphaseoffset;        /* 0x29: must be 0 */
  uint8_t  vphaseinit;          /* 0x2a */
  uint8_t  vphaseoffset;        /* 0x2b: must be 0 */

  uint32_t intpmode;            /* 0x2c: interpolation mode */
} __attribute__((aligned(16)));

/* Alpha blending (AB) command (32 bytes) */

struct ge2d_abcmd_s
{
  uint16_t cmd;                 /* 0x00 */
  uint16_t mode;                /* 0x02 */
  uint16_t srch;                /* 0x04 */
  uint16_t srcv;                /* 0x06 */
  uint32_t saddr;               /* 0x08 */
  uint32_t daddr;               /* 0x0c */
  uint16_t spitch;              /* 0x10 */
  uint16_t dpitch;              /* 0x12 */

  uint32_t fixedsrc;            /* 0x14 */
  uint32_t aaddr;               /* 0x18 */
  uint16_t apitch;              /* 0x1c */
  uint16_t reserved;
} __attribute__((aligned(16)));

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_rotwait;
static sem_t g_rotexc;
static sem_t g_geexc;

static int g_gfd = -1;
static char g_gcmdbuf[256] __attribute__((aligned(16)));

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ip_semtake(sem_t *id)
{
  while (sem_wait(id) != 0)
    {
      if (errno == EINTR)
        {
          return -EINTR;
        }
    }
  return OK;
}

static void ip_semgive(sem_t *id)
{
  sem_post(id);
}

static int intr_handler_ROT(int irq, FAR void *context, FAR void *arg)
{
  putreg32(1, ROT_INTR_CLEAR);
  putreg32(0, ROT_INTR_ENABLE);
  putreg32(1, ROT_INTR_DISABLE);

  ip_semgive(&g_rotwait);

  return 0;
}

static uint16_t calc_ratio(uint16_t src, uint16_t dest)
{
  uint16_t r;

  if (src > dest)
    {
      r = src / dest;
      if (r == 2 || r == 4 || r == 8 || r == 16 || r == 32 || r == 64)
        {
          return 256 * r;
        }
    }
  else if (src < dest)
    {
      r = dest / src;
      if (r == 2 || r == 4 || r == 8 || r == 16 || r == 32 || r == 64)
        {
          return 256 / r;
        }
    }
  else
    {
      return 256;
    }

  return 0;
}

static void *set_rop_cmd(void *cmdbuf, void *srcaddr, void *destaddr,
                         uint16_t srcwidth, uint16_t srcheight, uint16_t srcpitch,
                         uint16_t destwidth, uint16_t destheight, uint16_t destpitch,
                         uint8_t rop, uint8_t options, uint16_t patcolor)
{
  struct ge2d_ropcmd_s *rc = (struct ge2d_ropcmd_s *)cmdbuf;
  uint16_t rv;
  uint16_t rh;
  uint16_t cmd = ROPCMD | SRC16BPP;

  if (((uintptr_t)srcaddr & 1) || ((uintptr_t)destaddr & 1))
    {
      return NULL;
    }

  if (srcwidth & 1 || destwidth & 1)
    {
      return NULL;
    }

  rv = calc_ratio(srcheight, destheight);
  if (rv == 0)
    {
      return NULL;
    }
  rh = calc_ratio(srcwidth, destwidth);
  if (rh == 0)
    {
      return NULL;
    }

  /* If ratio is not 256 (x1), then set scaling bit. */

  if (rv != 256 || rh != 256 || options & CONV8BPP)
    {
      cmd |= SCALING;
    }

  memset(rc, 0, sizeof(struct ge2d_ropcmd_s));

  rc->cmd = cmd;
  rc->rop = rop;
  rc->options = options;
  rc->fixedcolor = patcolor;
  rc->srch = srcwidth - 1;
  rc->srcv = srcheight - 1;
  rc->saddr = (uint32_t)(uintptr_t)srcaddr | MSEL;
  rc->daddr = (uint32_t)(uintptr_t)destaddr | MSEL;
  rc->spitch = srcpitch - 1;
  rc->dpitch = destpitch - 1;
  rc->desth = destwidth - 1;
  rc->destv = destheight - 1;
  rc->ratiov = rv - 1;
  rc->ratioh = rh - 1;
  rc->hphaseinit = 1;
  rc->vphaseinit = 1;
  rc->intpmode = 0; /* XXX: HV Linear interpolation */

  /* return next command area */

  return (void *)((uintptr_t)cmdbuf + sizeof(struct ge2d_ropcmd_s));
}

#if 0
/* This function unused now.
 * Disable it for prevent warnings.
 */

static void *set_ab_cmd(void *cmdbuf, void *srcaddr, void *destaddr,
                        uint16_t srcwidth, uint16_t srcheight,
                        uint16_t srcpitch, uint16_t destpitch,
                        void *aaddr, int options, uint8_t fixedalpha)
{
  struct ge2d_abcmd_s *ac = (struct ge2d_abcmd_s *)cmdbuf;

  memset(ac, 0, sizeof(struct ge2d_abcmd_s));

  ac->cmd = ABCMD | options;
  ac->mode = fixedalpha;
  ac->srch = srcwidth - 1;
  ac->srcv = srcheight - 1;
  ac->saddr = (uint32_t)(uintptr_t)srcaddr | MSEL;
  ac->daddr = (uint32_t)(uintptr_t)destaddr | MSEL;
  ac->spitch = srcpitch - 1;
  ac->dpitch = destpitch - 1;
  ac->fixedsrc = 0x0080;
  ac->aaddr = (uint32_t)(uintptr_t)aaddr | MSEL;
  ac->apitch = srcpitch - 1;

  return (void *)((uintptr_t)cmdbuf + sizeof(struct ge2d_abcmd_s));
}
#endif

static void *set_halt_cmd(void *cmdbuf)
{
  memset(cmdbuf, 0, 16);
  return (void *)((uintptr_t)cmdbuf + 16);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imageproc_initialize(void)
{
  sem_init(&g_rotexc, 0, 1);
  sem_init(&g_rotwait, 0, 0);
  sem_init(&g_geexc, 0, 1);
  sem_setprotocol(&g_rotwait, SEM_PRIO_NONE);

  cxd56_ge2dinitialize(GEDEVNAME);

  g_gfd = open(GEDEVNAME, O_RDWR);

  putreg32(1, ROT_INTR_CLEAR);
  putreg32(0, ROT_INTR_ENABLE);
  putreg32(1, ROT_INTR_DISABLE);

  irq_attach(CXD56_IRQ_ROT, intr_handler_ROT, NULL);
  up_enable_irq(CXD56_IRQ_ROT);
}

void imageproc_finalize(void)
{
  up_disable_irq(CXD56_IRQ_ROT);
  irq_detach(CXD56_IRQ_ROT);

  sem_destroy(&g_rotwait);
  sem_destroy(&g_rotexc);
}

void imageproc_convert_yuv2rgb(uint8_t * ibuf, uint32_t hsize, uint32_t vsize)
{
  int ret;

  ret = ip_semtake(&g_rotexc);
  if (ret)
    {
      return;
    }

  /*
   * Image processing hardware want to be set horizontal/vertical size to
   * actual size - 1.
   */

  --hsize;
  --vsize;

  putreg32(1, ROT_INTR_ENABLE);
  putreg32(0, ROT_INTR_DISABLE);
  putreg32(0, ROT_SET_DIRECTION);

  putreg32(hsize, ROT_SET_SRC_HSIZE);
  putreg32(vsize, ROT_SET_SRC_VSIZE);
  putreg32((uint32_t)(uintptr_t)ibuf,  ROT_SET_SRC_ADDRESS);

  putreg32(hsize, ROT_SET_SRC_PITCH);
  putreg32((uint32_t)(uintptr_t)ibuf,  ROT_SET_DST_ADDRESS);

  putreg32(hsize, ROT_SET_DST_PITCH);

  putreg32(1, ROT_CONV_CTRL);
  putreg32(0, ROT_RGB_ALIGNMENT);
  putreg32(1, ROT_COMMAND);

  ip_semtake(&g_rotwait);

  ip_semgive(&g_rotexc);
}

void imageproc_convert_yuv2gray(uint8_t *ibuf, uint8_t *obuf, size_t hsize, size_t vsize)
{
  uint16_t *p_src = (uint16_t *) ibuf;
  size_t ix;
  size_t iy;

  for (iy = 0; iy < vsize; iy++)
    {
      for (ix = 0; ix < hsize; ix++)
        {
          *obuf++ = (uint8_t) ((*p_src++ & 0xff00) >> 8);
        }
    }
}

int imageproc_resize(uint8_t *ibuf, uint16_t ihsize, uint16_t ivsize,
                     uint8_t *obuf, uint16_t ohsize, uint16_t ovsize, int bpp)
{
  void *cmd = g_gcmdbuf;
  size_t len;
  int ret;

  if (g_gfd <= 0)
    {
      return -ENODEV;
    }

  if (bpp != 0 && bpp != 1)
    {
      return -EINVAL;
    }

  ret = ip_semtake(&g_geexc);
  if (ret)
    {
      return ret; /* -EINTR */
    }

  /* Create descriptor to graphics engine */

  cmd = set_rop_cmd(cmd, ibuf, obuf, ihsize, ivsize, ihsize,
                    ohsize, ovsize, ohsize,
                    SRCCOPY,
                    FIXEDCOLOR | (bpp ? CONV8BPP : 0),
                    0x0080);
  if (cmd == NULL)
    {
      ip_semgive(&g_geexc);
      return -EINVAL;
    }

  /* Terminate command */

  cmd = set_halt_cmd(cmd);

  /* Process resize */

  len = (uintptr_t)cmd - (uintptr_t)g_gcmdbuf;
  ret = write(g_gfd, g_gcmdbuf, len);
  if (ret < 0)
    {
      ip_semgive(&g_geexc);
      return -EFAULT;
    }

  ip_semgive(&g_geexc);

  return 0;
}
