
Usage of audio_recorder
===========================

Usage
---------------------------

Select options in below.

Å°Kernel
- [Library Routines]
    [Standard Math library] <= Y

- [File Systems]
    [FAT file system ] <= Y
      [FAT long file names] <= Y
      [FAT maximum file name size] <= 128
    [SMART file system] <= Y
      [Maximum file name length] <= 32

Å°SDK
- [CXD56xx Configuration Options]
    [Audio] <= Y
- [SDK audio] <= Y
    [Audio Utilities]
      [Audio Recorder] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Audio recorder example] <= Y

Build and install
--------------------------

Build Kernel and SDK.
Install 'nuttx.spk' to system.

After that, you can see worker binary 'MP3ENC', 'OPUSENC', 'SRC'
in directory sdk/modules/audio/dsp.
Store worker binary, playlist and play contents in the path specified by option.
 - Default path
    worker binary : /mnt/vfat/bin
    contents      : /mnt/vfat/rec

Execute
--------------------------

Type 'recorder' on nsh.
nsh>recorder

Audio from the microphone is recorded in the WAV file for 10 seconds.

