
Usage of fft
===========================

This is an example of offloading heavy process to other DSPs.
This sample uses CMSIS-DSP library and fft_main.c is almost the same with a
sample of CMSIS-DSP.

CMSIS-DSP is a Apache-2.0 Lisence, so user must be care about using this sample
for other purpose.

Build and install
--------------------------

Type 'make' to build NuttX.
After that, you can see worker binary 'dspmath' in dsp/math/.
And you need to copy it to target board via USB MSC under bin/.
