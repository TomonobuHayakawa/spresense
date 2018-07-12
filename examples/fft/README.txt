
Usage of fft
===========================

This is an example of offloading heavy process to other DSPs with ASMP
Framework.
This sample uses CMSIS-DSP library and example source bringed.

CMSIS-DSP is under Apache-2.0 Lisence, so user must be care about using this
sample for other purpose.

Build and install
--------------------------

You must download or clone CMSIS from https://github.com/ARM-software/CMSIS_5
to where you want before build.

Next, enable this example and configure EXAMPLES_FFT_DSPLIB_PATH option to
where you CMSIS source downloaded.

After build SDK, worker binary 'dspmath' placed in dsp/math/. Then copy it to
appropriate directory specified by EXAMPLES_FFT_DSPMATH_PATH option.
For example, create BIN directory on the top of SD card and copy dspmath to
BIN. By default, SD card mounted at /mnt/sd0, so /mnt/sd0/BIN/dspmath is a
loadable path.
