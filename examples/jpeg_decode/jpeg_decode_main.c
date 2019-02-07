/****************************************************************************
 * examples/jpeg_decode/jpeg_decode_main.c
 *
 *   Copyright 2019 Sony Semiconductor Solutions Corporation
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

/*
 * This example is based on read_JPEG_file() function by IJG.
 *  base: example.c in http://www.ijg.org/files/jpegsrc.v9c.tar.gz 
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h> /* For Spresense Kconfig */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

/*
 * Include file for users of JPEG library.
 * You will need to have included system headers that define at least
 * the typedefs FILE and size_t before you can include jpeglib.h.
 * (stdio.h is sufficient on ANSI-conforming systems.)
 * You may also wish to include "jerror.h".
 */

#include "jpeglib.h"

/*
 * In example.c by IJG, application use setjmp() and  longjmp().
 * Because NuttX OS do not support these functions, delete.
 */

/* #include <setjmp.h> */

/* For output to Spresense LCD.
 * imageproc has the color converter(YUV422 -> RGB565) function.
 */

#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include "jpeg_decode.h"

#  ifdef CONFIG_IMAGEPROC
#    include <imageproc/imageproc.h>
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define APP_FILENAME_LEN  128

#define APP_BYTES_PER_PIXEL 2

#define APP_QVGA_WIDTH    320

/* For output to Spresense LCD */

#ifndef CONFIG_EXAMPLES_JPEG_DECODE_LCD_DEVNO
#  define CONFIG_EXAMPLES_JPEG_DECODE_LCD_DEVNO 0
#endif

#define itou8(v) ((v) < 0 ? 0 : ((v) > 255 ? 255 : (v)))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* For output to Spresense LCD */

struct uyvy_s
{
  uint8_t u0;
  uint8_t y0;
  uint8_t v0;
  uint8_t y1;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/
/* In Spresense, the input file is specified not by file pointer
 * but by file descripter
 */

static int   infile;   /* file descriptor of input file */
static  char infile_name[APP_FILENAME_LEN] = "/mnt/spif/SAMPLE.JPG";

#ifndef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
static int   outfile;  /* file descriptor of output file */
static  char outfile_name[APP_FILENAME_LEN];
#endif  /* CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD */

struct jpeg_decompress_struct cinfo;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
struct nximage_data_s g_jpeg_decode_nximage =
{
  NULL,          /* hnx */
  NULL,          /* hbkgd */
  0,             /* xres */
  0,             /* yres */
  false,         /* havpos */
  { 0 },         /* sem */
  0              /* exit code */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/******************** JPEG DECOMPRESSION SAMPLE INTERFACE *******************/

/* This half of the example shows how to read data from the JPEG decompressor.
 * It's a bit more refined than the above, in that we show:
 *   (a) how to modify the JPEG library's standard error-reporting behavior;
 *   (b) how to allocate workspace using the library's memory manager.
 *
 * Just to make this example a little different from the first one, we'll
 * assume that we do not intend to put the whole image into an in-memory
 * buffer, but to send it line-by-line someplace else.  We need a one-
 * scanline-high JSAMPLE array as a work buffer, and we will let the JPEG
 * memory manager allocate it for us.  This approach is actually quite useful
 * because we don't need to remember to deallocate the buffer separately: it
 * will go away automatically when the JPEG object is cleaned up.
 */

#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
static inline int nximage_initialize(void)
{
  FAR NX_DRIVERTYPE *dev;
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize the LCD device */

  printf("nximage_initialize: Initializing LCD\n");
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      printf("nximage_initialize: board_lcd_initialize failed: %d\n", -ret);
      return ERROR;
    }

  /* Get the device instance */

  dev = board_lcd_getdev(CONFIG_EXAMPLES_JPEG_DECODE_LCD_DEVNO);
  if (!dev)
    {
      printf("nximage_initialize: board_lcd_getdev failed, devno=%d\n",
             CONFIG_EXAMPLES_JPEG_DECODE_LCD_DEVNO);
      return ERROR;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));

  /* Then open NX */

  printf("nximage_initialize: Open NX\n");
  g_jpeg_decode_nximage.hnx = nx_open(dev);
  if (!g_jpeg_decode_nximage.hnx)
    {
      printf("nximage_initialize: nx_open failed: %d\n", errno);
      return ERROR;
    }

  /* Set background color to black */

  color = 0;
  nx_setbgcolor(g_jpeg_decode_nximage.hnx, &color);
  ret = nx_requestbkgd(g_jpeg_decode_nximage.hnx,
                       &g_jpeg_decode_nximagecb, NULL);
  if (ret < 0)
    {
      printf("nximage_initialize: nx_requestbkgd failed: %d\n", errno);
      nx_close(g_jpeg_decode_nximage.hnx);
      return ERROR;
    }

  while (!g_jpeg_decode_nximage.havepos)
    {
      (void) sem_wait(&g_jpeg_decode_nximage.sem);
    }
  printf("nximage_initialize: Screen resolution (%d,%d)\n",
         g_jpeg_decode_nximage.xres, g_jpeg_decode_nximage.yres);

  return 0;
}


#  ifndef CONFIG_IMAGEPROC
static inline void ycbcr2rgb(uint8_t y,  uint8_t cb, uint8_t cr,
                             uint8_t *r, uint8_t *g, uint8_t *b)
{
  int _r;
  int _g;
  int _b;
  _r = (128 * (y-16) +                  202 * (cr-128) + 64) / 128;
  _g = (128 * (y-16) -  24 * (cb-128) -  60 * (cr-128) + 64) / 128;
  _b = (128 * (y-16) + 238 * (cb-128)                  + 64) / 128;
  *r = itou8(_r);
  *g = itou8(_g);
  *b = itou8(_b);
}

static inline uint16_t ycbcrtorgb565(uint8_t y, uint8_t cb, uint8_t cr)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;

  ycbcr2rgb(y, cb, cr, &r, &g, &b);
  r = (r >> 3) & 0x1f;
  g = (g >> 2) & 0x3f;
  b = (b >> 3) & 0x1f;
  return (uint16_t)(((uint16_t)r << 11) | ((uint16_t)g << 5) | (uint16_t)b);
}

/* Color conversion to show on display devices. */

static void yuv2rgb(void *buf, uint32_t size)
{
  struct uyvy_s *ptr;
  struct uyvy_s uyvy;
  uint16_t *dest;
  uint32_t i;

  ptr = buf;
  dest = buf;
  for (i = 0; i < size / 4; i++)
    {
      /* Save packed YCbCr elements due to it will be replaced with
       * converted color data.
       */

      uyvy = *ptr++;

      /* Convert color format to packed RGB565 */

      *dest++ = ycbcrtorgb565(uyvy.y0, uyvy.u0, uyvy.v0);
      *dest++ = ycbcrtorgb565(uyvy.y1, uyvy.u0, uyvy.v0);
    }
}
#  endif /* !CONFIG_IMAGEPROC */
#endif /* !CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD */

/* init_put_place(), put_scanline_someplace() and fin_put_place()
 * are specific for this example.
 * These are for displaying LCD or saving file.
 */

static int init_put_place(void)
{
  int ret = OK;
#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
  ret = nximage_initialize();
  if (ret < 0)
    {
      printf("camera_main: Failed to get NX handle: %d\n", errno);
      return ERROR;
    }
#  ifdef CONFIG_IMAGEPROC
  imageproc_initialize();
#  endif
#else
  /* Output file name = Input file name without extension + .YUV" */

  strncpy(outfile_name, infile_name, strlen(infile_name) - 3 /* 3 is extension length */);
  strncat(outfile_name, "YUV", 3);

  outfile = open(outfile_name, O_WRONLY | O_CREAT);
#endif
  return ret;
}

static void put_scanline_someplace(JSAMPROW buffer, int row_stride)
{
#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
  /* Convert YUV4:2:2 to RGB565 */
#  ifdef CONFIG_IMAGEPROC
  imageproc_convert_yuv2rgb((void *)buffer,
                            row_stride/2, /* output image width = row_stride/2
                                           * This 2 means 2bytes/pixel. 
                                           */
                            1);
#  else
  yuv2rgb(buffer, row_stride);
#  endif

  /* Diplay RGB565 */

  nximage_image(g_jpeg_decode_nximage.hbkgd, buffer);
#else
  /* Save to file */

  write(outfile, buffer, row_stride);
#endif

  return;
}


static void fin_put_place(void)
{
#ifdef CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD
#  ifdef CONFIG_IMAGEPROC
  imageproc_finalize();
#  endif
  nx_close(g_jpeg_decode_nximage.hnx);
#else
  close(outfile);
#endif /* CONFIG_EXAMPLES_JPEG_DECODE_OUTPUT_LCD */

  return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/*
 * Sample routine for JPEG decompression to YUV4:2:2.
 * Assume that the source file name is passed in.
 */

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int jpeg_decode_main(int argc, char *argv[])
#endif
{
  /* This struct contains the JPEG decompression parameters and pointers to
   * working space (which is allocated as needed by the JPEG library).
   */

  /* For reduction of stack size, globalize */
  /* struct jpeg_decompress_struct cinfo; */

  /* Because Spresense do not support setjmp/longjmp,
   *  use default error handling function for now.
   */

  struct jpeg_error_mgr jerr;
  /* More stuff */
  JSAMPARRAY buffer;            /* Output row buffer */
  int row_stride;               /* physical row width in output buffer */

  /* Command parameter mean input filename in this example. */

  if (argc > 1)
    {
      strncpy(infile_name, argv[1], APP_FILENAME_LEN);
    }

  /* Original libjpeg use file pointer to specify JPEG file.
   * Spresense use file descripter, which enables to get JPEG
   *  from any readable descripter.
   */

  if ((infile = open(infile_name, O_RDONLY)) < 0) {
    fprintf(stderr, "can't open %s\n", infile_name);
    return 0;
  }

  /* Step 1: allocate and initialize JPEG decompression object */

  /* We set up the normal JPEG error routines. */
  cinfo.err = jpeg_std_error(&jerr);
  /* jerr.error_exit = my_error_exit; */
  /* Because Spresense do not support setjmp, delete setjmp */
  /* if (setjmp(jerr.setjmp_buffer)) { */
    /* If we get here, the JPEG code has signaled an error.
     * We need to clean up the JPEG object, close the input file, and return.
     */
    /* jpeg_destroy_decompress(&cinfo); */
    /* fclose(infile); */
    /* return 0;       */
  /* } */
  /* Now we can initialize the JPEG decompression object. */

  jpeg_create_decompress(&cinfo);

  /* Step 2: specify data source (eg, a file) */

  jpeg_stdio_src(&cinfo, infile);

  /* Step 3: read file parameters with jpeg_read_header() */

  (void) jpeg_read_header(&cinfo, TRUE);
  /* We can ignore the return value from jpeg_read_header since
   *   (a) suspension is not possible with the stdio data source, and
   *   (b) we passed TRUE to reject a tables-only JPEG file as an error.
   * See libjpeg.txt for more info.
   */

  /* Step 3-1: Get image information */

  fprintf(stdout, "image width  = %d\n", cinfo.image_width);
  fprintf(stdout, "image height = %d\n", cinfo.image_height);

  /* Step 4: set parameters for decompression */

  /* JCS_YCbCr means YUV4:4:4 in original libjpeg.
   * In Spresense, YCbCr means YUV4:2:2 for memory reduction.
   */

  cinfo.out_color_space = JCS_YCbCr;

  /* In this example, output to QVGA(320*240) display */
  /* For such purpose, set downscaling in large input image case. */

  cinfo.scale_num = APP_QVGA_WIDTH;
  cinfo.scale_denom = cinfo.image_width;

  /* Step 5: Start decompressor */

  (void) jpeg_start_decompress(&cinfo);
  /* We can ignore the return value since suspension is not possible
   * with the stdio data source.
   */

  /* We may need to do some setup of our own at this point before reading
   * the data.  After jpeg_start_decompress() we have the correct scaled
   * output image dimensions available, as well as the output colormap
   * if we asked for color quantization.
   * In this example, we need to make an output work buffer of the right size.
   */
  /* JSAMPLEs per row in output buffer */
  row_stride = cinfo.output_width * 2; /* YUV4:2:2 size is 2bytes/pixel */

  /* Make a one-row-high sample array that will go away when done with image */

  buffer = (*cinfo.mem->alloc_sarray)
                ((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);
  init_put_place();

  /* Step 6: while (scan lines remain to be read) */
  /*           jpeg_read_scanlines(...); */

  /* Here we use the library's state variable cinfo.output_scanline as the
   * loop counter, so that we don't have to keep track ourselves.
   */
  while (cinfo.output_scanline < cinfo.output_height)
    { 
      /* jpeg_read_scanlines expects an array of pointers to scanlines.
       * Here the array is only one element long, but you could ask for
       * more than one scanline at a time if that's more convenient.
       */
      jpeg_read_scanlines(&cinfo, buffer, 1);
      /* Assume put_scanline_someplace wants a pointer and sample count. */
      put_scanline_someplace(buffer[0], row_stride);
    }

  /* Step 7: Finish decompression */

  fin_put_place();
  (void) jpeg_finish_decompress(&cinfo);

  /* We can ignore the return value since suspension is not possible
   * with the stdio data source.
   */

  /* Step 8: Release JPEG decompression object */

  /* This is an important step since it will release a good deal of memory. */
  jpeg_destroy_decompress(&cinfo);

  /* After finish_decompress, we can close the input file.
   * Here we postpone it until after no more JPEG errors are possible,
   * so as to simplify the setjmp error logic above.  (Actually, I don't
   * think that jpeg_destroy can do an error exit, but why assume anything...)
   */
  /* fclose(infile); */
  close(infile);

  /* At this point you may want to check to see whether any corrupt-data
   * warnings occurred (test whether jerr.pub.num_warnings is nonzero).
   */

  /* And we're done! */
  return 1;
}


/*
 * SOME FINE POINTS:
 *
 * In the above code, we ignored the return value of jpeg_read_scanlines,
 * which is the number of scanlines actually read.  We could get away with
 * this because we asked for only one line at a time and we weren't using
 * a suspending data source.  See libjpeg.txt for more info.
 *
 * We cheated a bit by calling alloc_sarray() after jpeg_start_decompress();
 * we should have done it beforehand to ensure that the space would be
 * counted against the JPEG max_memory setting.  In some systems the above
 * code would risk an out-of-memory error.  However, in general we don't
 * know the output image dimensions before jpeg_start_decompress(), unless we
 * call jpeg_calc_output_dimensions().  See libjpeg.txt for more about this.
 *
 * Scanlines are returned in the same order as they appear in the JPEG file,
 * which is standardly top-to-bottom.  If you must emit data bottom-to-top,
 * you can use one of the virtual arrays provided by the JPEG memory manager
 * to invert the data.  See wrbmp.c for an example.
 *
 * As with compression, some operating modes may require temporary files.
 * On some systems you may need to set up a signal handler to ensure that
 * temporary files are deleted if the program is interrupted.  See libjpeg.txt.
 */