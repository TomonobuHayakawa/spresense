/****************************************************************************
 * examples/adc_cxd56/adc_cxd56.h
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

#ifndef __APPS_EXAMPLES_ADC_CXD56_ADC_CXD56_H
#define __APPS_EXAMPLES_ADC_CXD56_ADC_CXD56_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_NSH_BUILTIN_APPS - Build the ADC test as an NSH built-in function.
 *  Default: Built as a standalone problem
 * CONFIG_EXAMPLES_ADC_DEVPATH - The default path to the ADC device. Default: /dev/adc0
 * CONFIG_EXAMPLES_ADC_NSAMPLES - If CONFIG_NSH_BUILTIN_APPS
 *   is defined, then the number of samples is provided on the command line
 *   and this value is ignored.  Otherwise, this number of samples is
 *   collected and the program terminates.  Default:  Samples are collected
 *   indefinitely.
 * CONFIG_EXAMPLES_ADC_GROUPSIZE - The number of samples to read at once.
 *   Default: 4
 */

#ifndef CONFIG_CXD56_ADC
#  error "CXD56 ADC device support is not enabled (CONFIG_CXD56_ADC)"
#endif

#ifndef CONFIG_EXAMPLES_ADC_DEVPATH
#  define CONFIG_EXAMPLES_ADC_DEVPATH "/dev/lpadc0"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct adc_cxd56_state_s
{
  bool      initialized;
  FAR char *devpath;
  int       count;
  int       bufsize;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __APPS_EXAMPLES_ADC_CXD56_ADC_CXD56_H */
