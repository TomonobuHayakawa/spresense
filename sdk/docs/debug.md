Debug {#debug_doc}
============================
[TOC]

--------------------------------------
# Overview {#debug_overview}

This manual describes some debug feature for Spritzer SDK.

--------------------------------------
# How to analyze Hardfaults {#debug_hardfault}

Refer to [Analyzing Cortex-M Hardfaults](http://nuttx.org/doku.php?id=wiki:howtos:cortexm-hardfault)

--------------------------------------
# Simple callstack utility tool {#debug_callstack}

This is a simple callstack utility tool.<br>
When assert or hardfault occurs, if CONFIG_ARCH_LOWPUTC=y, stack dump is output to the consoleg.<br>
Since it is only extracting the function address from Symbol.map, please note that it is not an accurate callstack.<br>

Usage:
~~~~~~~~~~~~~~~
Usage: python ../sony_apps/tools/callstack.py <System.map> <stackdump.log>
~~~~~~~~~~~~~~~

Procedure:
1. Save stack dump log into stackdump.log
~~~~~~~~~~~~~~~
$ vi stackdump.log
up_assert: Assertion failed at file:cxd56_appinit.c line: 168 task: init
up_dumpstate: sp:         0d01e238
up_dumpstate: stack base: 0d01e278
up_dumpstate: stack size: 000007ec
up_stackdump: 0d01e220: 00000000 0d017a5f 00000000 0d017a5f 00000000 0d017a5f 00000000 0d017a5f
up_stackdump: 0d01e240: 00000000 00000000 00000000 00000000 00000001 43410000 00000101 0d0156e7
up_stackdump: 0d01e260: 00000101 0d007943 00000101 0d0043bb 00000000 00000000 f7763ad4 0d01e284
~~~~~~~~~~~~~~~
2. Output callstack from System.map and stackdump.log
~~~~~~~~~~~~~~~
$ cd /path/to/nuttx
$ ../sony_apps/tools/callstack.py System.map stackdump.log
up_assert: Assertion failed at file:cxd56_appinit.c line: 168 task: init
up_dumpstate: sp:         0d01e238
up_dumpstate: stack base: 0d01e278
up_dumpstate: stack size: 000007ec
up_stackdump: 0d01e220: 00000000 0d017a5f 00000000 0d017a5f 00000000 0d017a5f 00000000 0d017a5f
up_stackdump: 0d01e240: 00000000 00000000 00000000 00000000 00000001 43410000 00000101 0d0156e7
up_stackdump: 0d01e260: 00000101 0d007943 00000101 0d0043bb 00000000 00000000 f7763ad4 0d01e284
----------------- callstack -----------------
[0d017a5f] board_app_initialize + 0x97
[0d017a5f] board_app_initialize + 0x97
[0d017a5f] board_app_initialize + 0x97
[0d017a5f] board_app_initialize + 0x97
[0d0156e7] boardctl + 0x21
[0d007943] nsh_main + 0x7
[0d0043bb] task_start + 0x1b
~~~~~~~~~~~~~~~

\n

--------------------------------------
# Debugging with OpenOCD (GDB/Eclipse)  {#debug_openocd}

Spritzer provides modern debugging environment with CMSIS compliant ICE (Incircuit Emulator) and cross GDB or Eclipse Hardware Debugging plugin.
OpenOCD acts as gdbserver, so you can use any GDB protocol supported debugger.

In this capter, we explain how to debug using cross GDB and Eclipse IDE.

## Hardware Requirement

- ULINK2, ULINK-ME or any other CMSIS compliant ICE (FW ver. 2.03 or above)
- Debug Board (DEBUG-02)

## Software Requirement

- OpenOCD source (custom package: tools/openocd-0.10.0-spritzer.zip)
- ARM cross toolchain
- Eclipse CDT (optional)

Additionally, OpenOCD needs following libraries to build.

Windows (Cygwin)
- libusb1.0, libusb1.0-devel
- libhidapi0, libhidapi-devel

Linux (Ubuntu)
~~~~~~~~~~~~~~~
$ sudo apt install libusb-1.0-0-dev libhidapi-dev
~~~~~~~~~~~~~~~

## Building OpenOCD

Build instructions:
~~~~~~~~~~~~~~~
$ unzip openocd-0.10.0-spritzer.zip
$ cd openocd-0.10.0-spritzer
$ ./configure && make
~~~~~~~~~~~~~~~

And install them.

Cygwin:
~~~~~~~~~~~~~~~
$ make install
~~~~~~~~~~~~~~~

Linux:
~~~~~~~~~~~~~~~
$ sudo make install
$ sudo cp /usr/local/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/
~~~~~~~~~~~~~~~

## Prepare to debug

1. Connect PC to target board via ICE before debugging ![JTAG cable connection](debug_jtag_cable.jpg)
2. Enable kernel option [Generate Debug Symbols] (CONFIG_DEBUG_SYMBOLS)
3. Add following hook function in your `$HOME/.gdbinit`
~~~~~~~~~~~~~~~
define hookpost-load

 if &g_readytorun != 0
  eval "monitor nuttx.pid_offset %d", &((struct tcb_s *)(0))->pid
  eval "monitor nuttx.xcpreg_offset %d", &((struct tcb_s *)(0))->xcp.regs
  eval "monitor nuttx.state_offset %d", &((struct tcb_s *)(0))->task_state
  eval "monitor nuttx.name_offset %d", &((struct tcb_s *)(0))->name
  eval "monitor nuttx.name_size %d", sizeof(((struct tcb_s *)(0))->name)
 end

end
~~~~~~~~~~~~~~~

\note Eclipse Hardware Debugging feature uses cross GDB in background, so user must configure GDB if you want to use only Eclipse.

Target board must be bring up with debug mode, confirm your debug console outputs.
~~~~~~~~~~~~~~~
Waiting for debugger connection..
~~~~~~~~~~~~~~~
If NuttX booted, then do following sequence.

1. Pressing 'r' key and reset button
2. Remove or rename 'nuttx' file from updater
~~~~~~~~~~~~~~~
updater> rm nuttx
~~~~~~~~~~~~~~~
3. reboot (push hardware reset)

## Debugging with GDB

### Run cross GDB

Move into directory that `nuttx` exists and run cross GDB.

~~~~~~~~~~~~~~~
$ cd nuttx
$ arm-none-eabi-gdb nuttx
~~~~~~~~~~~~~~~

### Debug instruction

You can run following sequence to start debugging.
~~~~~~~~~~~~~~~
(gdb) target remote | openocd -f interface/cmsis-dap.cfg -f target/cxd5602_adsp0.cfg -c "gdb_port pipe; log_output openocd.log"
(gdb) load
(gdb) continue
~~~~~~~~~~~~~~~

After NuttX is running, you can use any GDB commands (break, memory dump or etc..).
See [Debugging Programs with Multiple Threads](http://sourceware.org/gdb/current/onlinedocs/gdb/Threads.html#Threads).

After debug stop or push hardware reset button, then quit GDB with Ctrl-D.

Sample GDB log:
~~~~~~~~~~~~~~~
Program received signal SIGINT, Interrupt.
[Switching to Thread 218213264]
0x0d005920 in up_idle () at chip/cxd56_idle.c:228
228       asm("WFI");
(gdb) info threads
  Id   Target Id         Frame
  3    Thread 218227648 (Name: cxd56_pm_task, pid:2, WAIT_MQNOTEMPTY) up_switchcontext () at armv7-m/gnu/up_switchcontext.S:94
  2    Thread 218223712 (Name: init, pid:1, WAIT_SEM) up_switchcontext () at armv7-m/gnu/up_switchcontext.S:94
* 1    Thread 218213264 (Name: Idle Task, pid:0, RUNNING) 0x0d005920 in up_idle () at chip/cxd56_idle.c:228
(gdb) backtrace
#0  0x0d005920 in up_idle () at chip/cxd56_idle.c:228
#1  0x0d002e46 in os_start () at init/os_start.c:827
#2  0x0d000296 in __start () at chip/cxd56_start.c:337
(gdb) thread 2
[Switching to thread 2 (Thread 218223712)]
#0  up_switchcontext () at armv7-m/gnu/up_switchcontext.S:94
94              bx              lr
(gdb) backtrace
#0  up_switchcontext () at armv7-m/gnu/up_switchcontext.S:94
#1  0x0d00360c in sem_wait (sem=sem@entry=0xd01a9c8 <g_uart1port+24>) at semaphore/sem_wait.c:154
#2  0x0d0025f6 in uart_takesem (sem=0xd01a9c8 <g_uart1port+24>, errout=<optimized out>) at serial/serial.c:130
#3  0x0d002854 in uart_read (filep=0xd01d628, buffer=0xd01e267 "\rX\255\001\r", buflen=1) at serial/serial.c:751
#4  0x0d00b20c in file_read (filep=<optimized out>, buf=buf@entry=0xd01e267, nbytes=nbytes@entry=1)
    at vfs/fs_read.c:104
#5  0x0d00b254 in read (fd=<optimized out>, buf=buf@entry=0xd01e267, nbytes=nbytes@entry=1) at vfs/fs_read.c:178
#6  0x0d0073a0 in cle_getch (priv=0xd01e278) at cle.c:327
#7  0x0d007562 in cle_editloop (priv=0xd01e278) at cle.c:752
#8  cle (line=line@entry=0xd0217fc "", linelen=linelen@entry=80, instream=instream@entry=0xd01d730,
    outstream=<optimized out>) at cle.c:960
#9  0x0d0072d2 in nsh_session (pstate=pstate@entry=0xd0213a0) at nsh_session.c:144
#10 0x0d0070da in nsh_consolemain (argc=argc@entry=0, argv=argv@entry=0x0) at nsh_consolemain.c:122
#11 0x0d0070aa in nsh_main (argc=<optimized out>, argv=<optimized out>) at nsh_main.c:175
#12 0x0d003f58 in task_start () at task/task_start.c:151
#13 0x00000000 in ?? ()
(gdb)
~~~~~~~~~~~~~~~

## Debugging with Eclipse

For using Eclipse, require 'Eclipse CDT' and 'C/C++ GDB Hardware Debugging' plugin. 
If you are not installed it, please install from `[Help]` -> `[Install New Software...]`, and select `CDT Optional Features` -> `C/C++ GDB Hardware Debugging`.
If you can't find `CDT Optional Features` from available list, then enable CDT plugin site from `Available Software Sites`. (e.g. http://download.eclipse.org/tools/cdt/releases/indigo)

### Debug Configuration

Eclipse 'C/C++ GDB Hardware Debugging' plugin uses cross GDB in background, so many configurations are the same with GDB debugging.

1. Choose `[Run]` -> `[Debug Configurations...]`
2. Select `[GDB Hardware Debugging]` in the left pane and double click to create new debug configuration

- Main tab
  + Project  -> Your project
  + C/C++ Application -> `nuttx`
- Debugger tab
  + GDB Command -> Path to `arm-none-eabi-gdb`
  + Use remote target -> Enable
  + JTAG Device -> 'OpenOCD (via pipe)'
  + GDB Connection String -> `| openocd -f interface/cmsis-dap.cfg -f target/cxd5602_adsp0.cfg -c "gdb_port pipe; log_output openocd.log"`
- Startup tab
  + Reset and Delay -> Enable
  + Halt -> Enable
  + Load image -> Enable

![Debugger tab](debug_debugger_tab.png)

### Debug Instruction

You can run in debug mode using new configuration that created above.
When you run it, Eclipse loading `nuttx` kernel image, and stop at entry point. Then you can start debugging with `Resume` button or F8 key.
And pause with `Suspend` button, also stop debugging with `Terminate` button or Ctrl+F2, and you can use any other Eclipse debugging features.
