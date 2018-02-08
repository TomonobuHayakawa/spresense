/****************************************************************************
 * examples/epaper/epaper_bkgd.c
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (C) 2018 Sony Corporation
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

#include <nuttx/config.h>
#include <sdk/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "epaper.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select renderer -- Some additional logic would be required to support
 * pixel depths that are not directly addressable (1,2,4, and 24).
 */

#define RENDERER nxf_convert_8bpp

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void epaper_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool morem, FAR void *arg);
static void epaper_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg);
#ifdef CONFIG_NX_XYINPUT
static void epaper_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg);
#endif

#ifdef CONFIG_NX_KBD
static void epaper_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_hello[] = "SPRESENSE";

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Background window call table */

const struct nx_callback_s g_epapercb =
{
  epaper_redraw,   /* redraw */
  epaper_position  /* position */
#ifdef CONFIG_NX_XYINPUT
  , epaper_mousein /* mousein */
#endif
#ifdef CONFIG_NX_KBD
  , epaper_kbdin   /* my kbdin */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: epaper_redraw
 ****************************************************************************/

static void epaper_redraw(NXWINDOW hwnd, FAR const struct nxgl_rect_s *rect,
                        bool more, FAR void *arg)
{
  ginfo("hwnd=%p rect={(%d,%d),(%d,%d)} more=%s\n",
         hwnd, rect->pt1.x, rect->pt1.y, rect->pt2.x, rect->pt2.y,
         more ? "true" : "false");
}

/****************************************************************************
 * Name: epaper_position
 ****************************************************************************/

static void epaper_position(NXWINDOW hwnd, FAR const struct nxgl_size_s *size,
                          FAR const struct nxgl_point_s *pos,
                          FAR const struct nxgl_rect_s *bounds,
                          FAR void *arg)
{
  /* Report the position */

  ginfo("hwnd=%p size=(%d,%d) pos=(%d,%d) bounds={(%d,%d),(%d,%d)}\n",
        hwnd, size->w, size->h, pos->x, pos->y,
        bounds->pt1.x, bounds->pt1.y, bounds->pt2.x, bounds->pt2.y);

  /* Have we picked off the window bounds yet? */

  if (!g_epaper.havepos)
    {
      /* Save the background window handle */

      g_epaper.hbkgd = hwnd;

      /* Save the window limits */

      g_epaper.xres = bounds->pt2.x + 1;
      g_epaper.yres = bounds->pt2.y + 1;

      g_epaper.havepos = true;
      sem_post(&g_epaper.sem);
      ginfo("Have xres=%d yres=%d\n", g_epaper.xres, g_epaper.yres);
    }
}

/****************************************************************************
 * Name: epaper_mousein
 ****************************************************************************/

#ifdef CONFIG_NX_XYINPUT
static void epaper_mousein(NXWINDOW hwnd, FAR const struct nxgl_point_s *pos,
                         uint8_t buttons, FAR void *arg)
{
  printf("epaper_mousein: hwnd=%p pos=(%d,%d) button=%02x\n",
         hwnd,  pos->x, pos->y, buttons);
}
#endif

/****************************************************************************
 * Name: epaper_kbdin
 ****************************************************************************/

#ifdef CONFIG_NX_KBD
static void epaper_kbdin(NXWINDOW hwnd, uint8_t nch, FAR const uint8_t *ch,
                       FAR void *arg)
{
  ginfo("hwnd=%p nch=%d\n", hwnd, nch);

   /* In this example, there is no keyboard so a keyboard event is not
    * expected.
    */

   printf("epaper_kbdin: Unexpected keyboard callback\n");
}
#endif

/****************************************************************************
 * Name: epaper_center
 ****************************************************************************/

static void epaper_center(FAR struct nxgl_point_s *pos,
                           FAR const struct nx_font_s *fontset)
{
  FAR const struct nx_fontbitmap_s *fbm;
  FAR uint8_t *ptr;
  unsigned int width;

  /* Get the width of the collection of characters so that we can center the
   * hello world message.
   */

  for (ptr = (uint8_t*)g_hello, width = 0; *ptr; ptr++)
    {
      /* Get the font bitmap for this character */

      fbm = nxf_getbitmap(g_epaper.hfont, *ptr);
      if (fbm)
        {
          /* Add the font size */

          width += fbm->metric.width + fbm->metric.xoffset;
        }
      else
        {
           /* Use the width of a space */

          width += fontset->spwidth;
        }
    }

  /* Now we know how to center the string.  Create a the position and
   * the bounding box
   */

  pos->x = (g_epaper.xres - width) / 2;
  pos->y = (g_epaper.yres - fontset->mxheight) / 2;
}

/****************************************************************************
 * Name: epaper_initglyph
 ****************************************************************************/

static void epaper_initglyph(FAR uint8_t *glyph, uint8_t height,
                              uint8_t width, uint8_t stride)
{
  FAR uint8_t *ptr;
  unsigned int row;
  unsigned int col;

  /* Initialize the glyph memory to the background color */

  ptr = glyph;
  for (row = 0; row < height; row++)
    {
      /* Just copy the color value into the glyph memory */

      for (col = 0; col < width; col++)
        {
          *ptr++ = CONFIG_EXAMPLES_EPAPER_BGCOLOR;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: epaper_hello
 *
 * Description:
 *   Print "Hello, World!" in the center of the display.
 *
 ****************************************************************************/

void epaper_hello(NXWINDOW hwnd)
{
  FAR const struct nx_font_s *fontset;
  FAR const struct nx_fontbitmap_s *fbm;
  FAR uint8_t *glyph;
  FAR const char *ptr;
  FAR struct nxgl_point_s pos;
  FAR struct nxgl_rect_s dest;
  FAR const void *src[CONFIG_NX_NPLANES];
  unsigned int glyphsize;
  unsigned int mxstride;
  int ret;

  /* Get information about the font we are going to use */

  fontset = nxf_getfontset(g_epaper.hfont);

  /* Allocate a bit of memory to hold the largest rendered font */

  mxstride  = fontset->mxwidth;
  glyphsize = (unsigned int)fontset->mxheight * mxstride;
  glyph     = (FAR uint8_t*)malloc(glyphsize);

  /* NOTE: no check for failure to allocate the memory.  In a real application
   * you would need to handle that event.
   */

  /* Get a position so the "Hello, World!" string will be centered on the
   * display.
   */

  epaper_center(&pos, fontset);
  printf("epaper_hello: Position (%d,%d)\n", pos.x, pos.y);

  /* Now we can say "hello" in the center of the display. */

  for (ptr = g_hello; *ptr; ptr++)
    {
      /* Get the bitmap font for this ASCII code */

      fbm = nxf_getbitmap(g_epaper.hfont, *ptr);
      if (fbm)
        {
          uint8_t fheight;      /* Height of this glyph (in rows) */
          uint8_t fwidth;       /* Width of this glyph (in pixels) */
          uint8_t fstride;      /* Width of the glyph row (in bytes) */

          /* Get information about the font bitmap */

          fwidth  = fbm->metric.width + fbm->metric.xoffset;
          fheight = fbm->metric.height + fbm->metric.yoffset;
          fstride = (fwidth * CONFIG_EXAMPLES_EPAPER_BPP + 7) >> 3;

          /* Initialize the glyph memory to the background color */

          epaper_initglyph(glyph, fheight, fwidth, fstride);

          /* Then render the glyph into the allocated memory */

          (void)RENDERER(glyph, fheight, fwidth,
                         fstride, fbm, CONFIG_EXAMPLES_EPAPER_FONTCOLOR);

          /* Describe the destination of the font with a rectangle */

          dest.pt1.x = pos.x;
          dest.pt1.y = pos.y;
          dest.pt2.x = pos.x + fwidth - 1;
          dest.pt2.y = pos.y + fheight - 1;

          /* Then put the font on the display */

          src[0] = (FAR const void *)glyph;
          ret = nx_bitmap((NXWINDOW)hwnd, &dest, src, &pos, fstride);
          if (ret < 0)
            {
              printf("epaper_write: nx_bitmapwindow failed: %d\n", errno);
            }

           /* Skip to the right the width of the font */

          pos.x += fwidth;
        }
      else
        {
           /* No bitmap (probably because the font is a space).  Skip to the
            * right the width of a space.
            */

          pos.x += fontset->spwidth;
        }
    }

  /* Free the allocated glyph */

  free(glyph);
}
