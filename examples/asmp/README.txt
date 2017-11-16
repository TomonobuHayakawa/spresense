
Usage of asmp_test
===========================

Usage
---------------------------

Select options in below.

- [System Type]
    [Inter CPU Communication Device] <= Y
- [Library Routines]
    [ASMP support library] <= Y
- [Memory Management]
    [Enable Tile Allocator]    <= Y
    [Number of memory regions] <= 2
- [Application Configuration]
    [Examples]
      [ASMP ELF Loader Example] <= Y

(optional)
- [File Systems]
    [ROMFS file system] <= Y


Build and install
--------------------------

Type 'make' to build NuttX.
After that, you can see worker binary 'hello' in directory tests/hello.
If you not set ROMFS file system, then you need to copy it to target board via
USB MSC.
If you set ROMFS file system, then it already contained nuttx binary image
as ROMFS file image.

CAUTION
apps build system cannot build automatically by configuration or/and example
source modification.
Please 'make clean' first.
