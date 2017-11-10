
Spritzer ASMP Framework
============================
[TOC]

# General #     {#spritzer_asmp_framework_general}

Spritzer ASMP Framework is designed for easy to use multi-core architecture processors.
This framework based on [NuttX](http://www.nuttx.org).

This framework defines 2 tasks.
- Supervisor task (NuttX task)
- [MP task](@ref mptask)

Supervisor controls [MP task](@ref mptask) by mptask_* APIs. Additionally, MP task has a 2 task types, Worker and Collaborator. See [MP task](@ref mptask) page for more information.

![ASMP basic flow](asmp_basic_flow.png)

-# Find free Sub CPU.
-# Allocate memory from outside of heap, and load ELF image to there.
-# Main CPU maps its address space to based on zero, and bring up Sub CPU.
-# When Sub CPU is started, send start signal to Main CPU.
-# When Sub CPU is stopped by any reasons, send stop signal to Main CPU. And Main CPU do some finalize processes.

MP task is a special control block, running on other CPUs in independent environment and completely parallel.
So we need to define the special communication methods for these different tasks.

There are 3 MP objects for communication between each tasks.
- [MP message queue](@ref mpmq)
- [MP mutex](@ref mpmutex)
- [MP shared memory](@ref mpshm)

![MP message queue](asmp_mq.png)

[MP message queue](@ref mpmq) is a message exchange interface like a POSIX message queue.

![MP mutex](asmp_mutex.png)

[MP mutex](@ref mpmutex) is a exclusive lock mechanism like a pthread mutex.

![MP shared memory](asmp_shm.png)

[MP shared memory](@ref mpshm) is a sharing same memory from each tasks.

Each MP object APIs are the almost same on the each tasks. For example, mpmq_send() function can be call from each tasks.

# Configuration #   {#spritzer_asmp_framework_configuration}

~~~
[Library Routines]
  [ASMP support library]     (LIB_ASMP)        = Y

[Memory Management]
  [Number of memory regions] (MM_REGIONS)      = 2
  [Enable Tile Allocator]    (MM_TILE)         = Y

[System Type]
  [RAM size for kernel]      (CXD56_RAM1_SIZE) = 0x80000 (default, 512KB)
  [RAM size for ASMP]        (CXD56_RAM2_SIZE) = 0x100000 (default, 1MB)
~~~

ASMP framework uses additional memory subsystem named Tile allocator, for worker program loading and shared memory.
ASMP framework needs separated memory region other than kernel. It can be configured by `CXD56_RAM{1,2}_SIZE` options.
If disable ASMP framework, kernel can be used all of 1.5MB memory region.

# Sequence #   {#spritzer_asmp_framework_sequence}

For co-working with workers on the other CPUs, do following sequence.

-# Create MP task
-# Create MP objects and bind them to MP task
-# Exec MP task
-# (parallel processing...)
-# Destroy MP task and MP objects (if needed)

See [supervisor example](@ref sony_apps/examples/asmp_test/asmp_test_main.c) and [worker example](@ref sony_apps/examples/asmp_test/tests/hello/hello.c)


# MP task #   {#spritzer_asmp_framework_mptask}

MP task APIs controlling worker or any other processes running on other CPUs
such as pthreads.
 
MP task can be exec ELF format binary, it must be resolved all symbols.
The ELF binary is running on independent environment, so MP tasks are not
reference to any NuttX code.\n

MP task has two task types, Worker and Collaborator.
Worker is a small and simple program, Collaborator is a large and complex program.\n
Here is a layout of ASMP framework supported ELF file.

\image html mptask_fileformat.png
<div align="center">Supported ELF layout</div>
\n

Both of program type, must be started with 0x00000000 and each sections are continuous.\n
User can be use worker support library to create worker program.\n
Collaborator has no restriction inside the code, but it can't get some assistence from ASMP framework.\n

## Building a worker #     {#spritzer_asmp_framework_build}

Build Example:
\code
$ arm-none-eabi-gcc -c $CELFFLAGS worker.c -o worker.o
$ arm-none-eabi-ld $LDRAWELFFLAGS -L nuttx/lib -o worker  worker.o -lasmpw
\endcode

\c CELFFLAGS and \c LDRAWELFFLAGS are already defined by board \c Make.defs at the nuttx top directory. Example source and Makefile are located at \c sony_apps/example/asmp_test/tests/hello.

# Code Examples #   {#spritzer_asmp_framework_example}

-# Simple execution

This is a simple worker MP task execution example.

\dontinclude asmp_test/asmp_test_main.c

\skip mptask_init
\until ;
\skip mptask_exec
\until ;
\skip mptask_destroy
\until ;

\c wret will be stored the worker exit status code.

\example sony_apps/examples/asmp_test/asmp_test_main.c
\example sony_apps/examples/asmp_test/tests/hello/hello.c
