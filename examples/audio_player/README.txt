
Usage of audio_player
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
      [Audio Player] <= Y
      [Playlist manager] <= Y
- [Memory manager] <= Y
- [ASMP] <= Y
- [Examples]
    [Audio player example] <= Y

Build and install
--------------------------

Build Kernel and SDK.
Install 'nuttx.spk' to system.

After that, you can see worker binary 'AACDEC', 'MP3DEC', 'OPUSDEC'
in directory sdk/modules/audio/dsp.
Store worker binary, playlist and play contents in the path specified by option.
 - Default path
    worker binary : /mnt/vfat/bom
    play list     : /mnt/vfat/playlist
    contents      : /mnt/vfat/audio

Execute
--------------------------

Type 'player' on nsh.
nsh>player

The first content of playlist will be played for 10 seconds.

