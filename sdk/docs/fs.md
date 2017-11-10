File System {#fs_doc}
============================
[TOC]

--------------------------------------
# Overview {#fs_overview}

Spritzer SDK supports various file systems desribed below that are originally supported in NuttX.<br>
And Spritzer SDK supports some storage devices such as SPI-Flash, eMMC and SD-Card.
- Virtual File System (VFS)
- procfs: pseudo-file system
- ROMFS file system
- SmartFS: Sector Mapped Allocation for Really Tiny (SMART) flash file system.
- NXFFS: The tiny NuttX wear-leveling FLASH file system
- FAT12/16/32 file system
- etc.

This manual mainly describes usages and configurations on Spritzer SDK.<br>
Refer to the NuttX documents for details of NuttX's file system function.
- [NuttX User's Manual --- FileSystem](http://nuttx.org/Documentation/NuttxUserGuide.html#FileSystem)
- [NuttX RTOS Porting Guide --- FileSystem](http://nuttx.org/Documentation/NuttxPortingGuide.html#NxFileSystem)
- [NuttX pseudo file system](http://www.nuttx.org/doku.php?id=wiki:vfs:pseudo-file-system)


--------------------------------------
# SPI-Flash Storage {#flash}

Supported SPI-Flash device:
- Macronix MX25U Series (MX25U8033E, MX25U1635F, MX25U3235F, MX25U6435F, MX25U12835F, MX66U51235F)
- Macronix MX25R Series (MX25R8035F, MX25R1635F, MX25R3235F, MX25R6435F, MX25R12835F)
- Winbond W25 Series (W25Q80EW, W25Q16FW, W25Q32FW, W25Q64FW, W25Q128FW)

SPI-Flash is divided into two regions.
One is "NuttX Kernel and Firmware region", and the other is "NuttX Application region".
- 1. NuttX Kernel and Firmware region
- 2. NuttX Application region

The 1st "NuttX Kernel and Firmware region" will be stored NuttX kernel-included firmware (nuttx) and some proprietary firmwares.<br>
As example of proprietary firmwares:
- Second Boot Loader (SBL)
- System Loader Firmware (loader)
- GNSS Firmware (gnssfw)
- DSP Sensor Firmware(s)
- DSP Audio Firmware(s)
- etc.

These firmwares will be included and released into SDK package.<br>
Basically theae firmwares will be encrypted and stored into the SPI-Flash.<br>
This region is formatted with the original file system which supports for wear-leveling and countermeasure against power temporary blackout.<br>
NuttX application can't access directly to this region.<br>

The 2nd "NuttX Application region" can be freely configured and used by NuttX Application.<br>
The file system supported in this region are mainly:
- SmartFS (Recommended)
- NXFFS

The following chapters describe each file system.

--------------------------------------
## SmartFS File System {#smartfs}

See below NuttX wiki page for details of SmartFS.
- [Using SmartFS](http://nuttx.org/doku.php?id=wiki:vfs:using-smartfs)
- [SmartFS Internals](http://nuttx.org/doku.php?id=wiki:vfs:smartfs-internals)



### Configuration {#smartfs_config}

- SmartFS Related configurations: See help menu of Kconfig for each detail.
 |CONFIG                                | Description                                               |Example |
 |:-------------------------------------|:----------------------------------------------------------|:-------|
 |CONFIG_FS_SMARTFS                     | SMART file system                                         |y       |
 |CONFIG_SMARTFS_ERASEDSTATE            | FLASH erased state                                        |0xff    |
 |CONFIG_SMARTFS_MAXNAMLEN              | Maximum file name length                                  |30      |
 |CONFIG_SMARTFS_MULTI_ROOT_DIRS        | Support multiple Root Directories / Mount Points          |y       |
 |CONFIG_SMARTFS_ALIGNED_ACCESS         | Ensure 16 and 32 bit accesses are aligned                 |y       |
 |CONFIG_MTD_SMART                      | SMART Flash support                                       |y       |
 |CONFIG_SMART_DEV_LOOP                 | Enable SMART loop device                                  |n       |
 |CONFIG_MTD_SMART_SECTOR_SIZE          | SMART Device sector size                                  |4096    |
 |CONFIG_MTD_SMART_SKIP_SECTOR_SIZE_SCAN| Skip secter size scan                                     |y       |
 |CONFIG_MTD_SMART_WRITEBUFFER          | Enable SMART write buffering                              |n       |
 |CONFIG_MTD_SMART_READAHEAD            | Enable SMART read-ahead buffering                         |n       |
 |CONFIG_MTD_SMART_WEAR_LEVEL           | Support FLASH wear leveling                               |y       |
 |CONFIG_MTD_SMART_CONVERT_WEAR_FORMAT  | Convert existing non wear leveling FLASH to wear leveling |n       |
 |CONFIG_MTD_SMART_ENABLE_CRC           | Enable Sector CRC error detection                         |y       |

- If use SmartFS, the following configurations must be enabled.
 - CONFIG_FS_SMARTFS=y
 - CONFIG_SMARTFS_ERASEDSTATE=0xff (default)
 - CONFIG_SMARTFS_ALIGNED_ACCESS=y (default)
 - CONFIG_MTD_SMART=y

As a example of setup, SPI-Flash is initialized and mounted on /mnt/spif in configs/{board}/cxd56_appinit.c.
- Example:
~~~~~~~~~~~~~~~{.c}
  // Initialize to provide SMARTFS on the MTD interface

  ret = smart_initialize(CONFIG_SFC_DEVNO, mtd, NULL);
  if (ret < 0)
    {
      fdbg("ERROR: SmartFS initialization failed: %d\n", ret);
      return ret;
    }

  ret = mount("/dev/smart0d1", "/mnt/spif", "smartfs", 0, NULL);
  if (ret < 0)
    {
      fdbg("ERROR: Failed to mount the SmartFS volume: %d\n", errno);
      return ret;
    }
~~~~~~~~~~~~~~~

### How to introduce SmartFS file system for the first time {#smartfs_intro}

 > If you introduce SmartFS for the first time, you have to format the SPI-Flash by using one of the following method.<br>
 > - Using Recovery Tool
 > - Using mksmartfs Tool
 > 
 > **Recovery Tool**<br>
 > Immediately after running the Recovery tool, you will be asked if you want to format the SPI-Flash.<br>
 > If type 'y' within 5 sec, after doing all-erase the SPI-Flash, SPI-Flash is automatically formatted with SmartFS file system.
 > ~~~~~~~~~~~~~~~
 > factory corvo_ast May 25 2017 22:47:57
 > Do flash all erase? (select within 5sec) [n]? (y/n) <===== select 'y' or 'n'
 > 
 > ..........................................................
 > done.
 > Checking sectors..
 > done.
 > Package validation is OK.
 > Saving package to "updater"
 > Package validation is OK.
 > Saving package to "usb.mod"
 > SBL successfully installed.
 > success
 > reboot...
 > ~~~~~~~~~~~~~~~
 > 
 > **mksmartfs**<br>
 > You can format SPI-Flash with SmartFS file system by using mksmartfs utility tool.<br>
 > For explanation of mksmartfs command, refer to following chapter [How to mksmartfs](#mksmartfs).<br>
 > This command takes several minutes, depending on SPI-Flash capacity.<br>
 > If reboot the system after the command is completed, then you will access to SPI-Flash with SmartFS file system.
 > ~~~~~~~~~~~~~~~
 > nsh> mksmartfs -s 4096 /dev/smart0d1 1
 > ~~~~~~~~~~~~~~~
 > 
 > ------

### How to mksmartfs {#mksmartfs}

 > 1. Build Configuration (Refer to sony_apps/configs/corvo/mksmartfs-defconfig)<br>
 >  - CONFIG_FSUTILS_MKSMARTFS=y
 >  - CONFIG_NSH_DISABLE_MKSMARTFS=n
 > 
 > 2. Run mksmartfs<br>
 > Usage:
 > ~~~~~~~~~~~~~~~
 > mksmartfs [-s sector-size] <path> [<num-root-directories>]
 > ~~~~~~~~~~~~~~~
 > Example:
 > ~~~~~~~~~~~~~~~
 > nsh> mksmartfs -s 4096 /dev/smart0d1 1
 > ~~~~~~~~~~~~~~~
 > In default setting, please set following values.<br>
 >  - sector-size          : 4096
 >  - path                 : /dev/smart0d1
 >  - num-root-directories : 1
 > 
 > 
 > ------

### How to nxfuse {#nxfuse}

 > **nxfuse** is a NuttX dedicated Linux FUSE tool that enables native mounting of NuttX filesystems in Linux.<br>
 > See the nxfuse/README.txt file for further information.<br>
 > - [nxfuse README](https://bitbucket.org/nuttx/tools/src/ef1fe93c58b3e541555daaa59a1d73b07a5ac32f/nxfuse/README.txt?fileviewer=file-view-default)
 > 
 > **Setup nxfuse environment**
 > 1. Create environment
 >  - Clone from Git(https://bitbucket.org/nuttx/tools.git/)
 >  - Install libfuse-dev in Linux
 > ~~~~~~~~~~~~~~~
 > $ sudo apt-get install libfuse-dev
 > ~~~~~~~~~~~~~~~
 > 
 > 2. Build nxfuse
 >  - Apply a following patch<br>
 > This patch is necessary to make the sector size 4096.
 > ~~~~~~~~~~~~~~~{.c}
 > diff --git a/nxfuse/src/smartfs/smart.c b/nxfuse/src/smartfs/smart.c
 > index 076ef98..aab16d1 100644
 > --- a/nxfuse/src/smartfs/smart.c
 > +++ b/nxfuse/src/smartfs/smart.c
 > @@ -1836,7 +1836,7 @@ static int smart_scan(FAR struct smart_struct_s *dev)
 >          {
 >            /* Read the next sector from the device */
 >  
 > -          ret = MTD_READ(dev->mtd, 0, sizeof(struct smart_sect_header_s),
 > +          ret = MTD_READ(dev->mtd, readaddress, sizeof(struct smart_sect_header_s),
 >                           (FAR uint8_t *) &header);
 >            if (ret != sizeof(struct smart_sect_header_s))
 >              {
 > @@ -1848,7 +1848,7 @@ static int smart_scan(FAR struct smart_struct_s *dev)
 >              {
 >                sectorsize = (header.status & SMART_STATUS_SIZEBITS) << 7;
 >                if (sectorsize == 0)
 > -                sectorsize = 256;
 > +                sectorsize = CONFIG_MTD_SMART_SECTOR_SIZE;
 >                break;
 >              }
 > ~~~~~~~~~~~~~~~
 >  - Compile nuttx<br>
 > In order to build nxfuse, you have to build nuttx that smartfs is enabled in advance.<br>
 > ~~~~~~~~~~~~~~~
 > $ cd sony_apps
 > $ ./tools/configure.sh corvo/nsh
 > $ cd nuttx
 > $ make menuconfig <-- Must be set CONFIG_DEBUG=n, otherwise it's fail to build nxfuse
 > $ make
 > ~~~~~~~~~~~~~~~
 >  - Compile and install nxfuse<br>
 > Specify the path to nuttx in NUTTXDIR, make nxfuse.
 > ~~~~~~~~~~~~~~~
 > $ cd tools/nxfuse
 > $ make NUTTXDIR=/path/to/nuttx
 > $ sudo make install <-- Copy nxfuse into /usr/bin and a manual into /usr/share/man.
 > ~~~~~~~~~~~~~~~
 > 
 > 3. Create SmartFS file system on Linux.<br>
 > In this example, create under "/tmp".
 >  - Create a image file (4K x N sectors)
 > ~~~~~~~~~~~~~~~
 > $ dd if=/dev/zero of=/tmp/smartfs.bin bs=4096 count=20
 > ~~~~~~~~~~~~~~~
 >  - Create working directory<br>
 > ~~~~~~~~~~~~~~~
 > $ mkdir /tmp/smartfs
 > ~~~~~~~~~~~~~~~
 >  - Create a new filesystem
 > ~~~~~~~~~~~~~~~
 > $ nxfuse -t smartfs -m -e 4096 -l 4096 -p 4096 -g 1 /tmp/smartfs.bin
 > Formatting /tmp/smartfs.bin with smartfs format
 > Format successful
 > ~~~~~~~~~~~~~~~
 >   - -m : create a new NuttX filesystem
 >   - -e : erase size
 >   - -l : sector size
 >   - -p : page size
 >   - -g : the number of root directories
 >  - Mounts directory within the /tmp/smartfs.bin file at mount point /tmp/smartfs
 > ~~~~~~~~~~~~~~~
 > $ nxfuse -t smartfs -e 4096 -l 4096 -p 4096 /tmp/smartfs /tmp/smartfs.bin
 > ~~~~~~~~~~~~~~~
 >  - Unmount a FUSE filesystem /tmp/smartfs
 > ~~~~~~~~~~~~~~~
 > $ fusermount -u /tmp/smartfs
 > ~~~~~~~~~~~~~~~
 > 
 > **Save a filesystem image on the Target into the Host PC**
 > 1. Create FAT ramdisk and mount at /tmp. Refer to [mkfatfs](#fat_util_mkfatfs).<br>
 > This is a example of using FAT ramdisk. That is, it's size is limited to the SRAM capacity.
 > ~~~~~~~~~~~~~~~
 > nsh> mkrd -m 1 -s 512 512
 > nsh> mkfatfs /dev/ram1
 > nsh> mount -t vfat /dev/ram1 /tmp
 > ~~~~~~~~~~~~~~~
 > 2. Save a raw binary image from SPI-Flash into smartfs.bin. Refer to [flashwriter](#flashwriter).
 > ~~~~~~~~~~~~~~~
 > nsh> flashwriter read /tmp/smartfs.bin -s 0 -n 20
 > ~~~~~~~~~~~~~~~
 >   - -s : start sector number
 >   - -n : numbers of sector
 > 3. Start USB Mass Storage
 > ~~~~~~~~~~~~~~~
 > nsh> umount /tmp
 > nsh> msconn
 > ~~~~~~~~~~~~~~~
 > 4. Copy smartfs.bin to USB Mass Storage drive on the Host PC
 > ~~~~~~~~~~~~~~~
 > $ cp <USB MSC drive>/smartfs.bin <local directory on the Host PC>
 > ~~~~~~~~~~~~~~~
 > 6. Mounts directory within the /tmp/smartfs.bin file at mount point /tmp/smartfs
 > ~~~~~~~~~~~~~~~
 > $ mkdir /tmp/smartfs
 > $ nxfuse -t smartfs -e 4096 -l 4096 -p 4096 /tmp/smartfs /tmp/smartfs.bin
 > ~~~~~~~~~~~~~~~
 > 7. Can access any file in /tmp/smartfs on Linux PC.
 > 8. Unmount
 > ~~~~~~~~~~~~~~~
 > $ fusermount -u /tmp/smartfs
 > ~~~~~~~~~~~~~~~
 > 
 > **Restore a filesystem image to the Target**
 > 1. Prepared for a filesystem image with SmartFS format
 > 2. Create FAT ramdisk and mount at /tmp. Refer to [mkfatfs](#fat_util_mkfatfs).<br>
 > This is a example of using FAT ramdisk. That is, it's size is limited to the SRAM capacity.
 > ~~~~~~~~~~~~~~~
 > nsh> mkrd -m 1 -s 512 512
 > nsh> mkfatfs /dev/ram1
 > ~~~~~~~~~~~~~~~
 > 3. Start USB Mass Storage
 > ~~~~~~~~~~~~~~~
 > nsh> msconn
 > ~~~~~~~~~~~~~~~
 > 4. Copy smartfs.bin on USB Mass Storage drive to the Target
 > ~~~~~~~~~~~~~~~
 > $ cp <local directory on the Host PC>/smartfs.bin <USB MSC drive>
 > ~~~~~~~~~~~~~~~
 > 5. Stop USB Mass Storage
 > ~~~~~~~~~~~~~~~
 > nsh> msdis
 > ~~~~~~~~~~~~~~~
 > 6. Mount a FAT Filesystem
 > ~~~~~~~~~~~~~~~
 > nsh> mount -t vfat /dev/ram1 /tmp
 > ~~~~~~~~~~~~~~~
 > 7. Restore a raw binary image into SPI-Flash. Refer to [flashwriter](#flashwriter).
 > ~~~~~~~~~~~~~~~
 > nsh> flashwriter write /tmp/smartfs.bin -s 0 -n 20
 > ~~~~~~~~~~~~~~~
 >   - -s : start sector number
 >   - -n : numbers of sector
 > 8. Reboot the Target
 > 
 > 
 > ------

### How to flashwriter {#flashwriter}

 > **flashwriter** is a utility tool that can save/restore raw binary image on the SPI-Flash via a file.
 > 
 > 1. Build Configuration
 >  - CONFIG_SYSTEM_FLASHWRITER=y
 > ~~~~~~~~~~~~~~~
 > [Application Configuration]
 >   [System Libraries and NSH Add-Ons]
 >    [flashwriter Command] <- [y]
 > ~~~~~~~~~~~~~~~
 > 2. Usage
 > ~~~~~~~~~~~~~~~
 > flashwriter <write|read> <file-path> [-s start-sector] [-n number-of-sector]
 > ~~~~~~~~~~~~~~~
 > If start-sector is omitted, start-sector is set 0.<br>
 > If number-of-sector is omitted, number-of-sector is set max sector number.
 > 
 > 
 > ------

### How to copy from/to SPI-Flash by using Zmodem {#zmodem}

 > The Zmodem commands can transfer files between the Target and the Host PC easily.
 > 
 > **Receiving Files on the Target from the Host PC**
 > 1. Build Configuration (Refer to sony_apps/configs/corvo/zmodem-defconfig)<br>
 >  - CONFIG_SYSTEM_ZMODEM=y
 >  - CONFIG_SYSTEM_ZMODEM_DEVNAME="/dev/console" (default)
 >  - CONFIG_SYSTEM_ZMODEM_RCVBUFSIZE=1024
 >  - CONFIG_SYSTEM_ZMODEM_PKTBUFSIZE=1024
 >  - CONFIG_SYSTEM_ZMODEM_MOUNTPOINT="/mnt/spif"
 >  - CONFIG_SERIAL_TERMIOS=y
 >  - CONFIG_UART1_RXBUFSIZE=1024
 >  - CONFIG_UART1_TXBUFSIZE=1024
 >  - CONFIG_CDCACM=y
 >  - CONFIG_CDCACM=RXBUFSIZE=32767 (No care about memory usage...)
 >  - CONFIG_SCHED_WAITPID=y (Recommend)
 > 2. Start to receive zmodem on the target<br>
 > If using console device like as DebugUART,
 > ~~~~~~~~~~~~~~~
 > nsh> rz
 > ~~~~~~~~~~~~~~~
 > If using USB CDC/ACM,
 > ~~~~~~~~~~~~~~~
 > nsh> sercon
 > sercon: Registering CDC/ACM serial driver
 > sercon: Successfully registered the CDC/ACM serial driver
 > nsh> rz -d /dev/ttyACM0
 > ~~~~~~~~~~~~~~~
 > 3. Send file via zmodem from the Host PC<br>
 > In case of TeraTerm, File -> Transfer -> ZMODEM -> Send..., and Select a file you want to send.<br>
 > You can also use other terminal with zmodem transfer function such as minicom.
 > In case of the NuttX Host PC tool, Use the following command,<br>
 > Then a file will be transferred into the CONFIG_SYSTEM_MOUNTPOINT such as "/mnt/spif".
 > ~~~~~~~~~~~~~~~
 > $ ./sz -d /dev/ttyXXX -x 1 </abspath/to/filename>
 > ~~~~~~~~~~~~~~~
 > /dev/ttyXXX : your serial device to be connected with HostPC like as ttyS[n]? or ttyUSB[n]<br>
 > </path/to/filename> : filename must be an absolute path starting at '/'<br>
 > 
 > **Sending Files from the Target to the Host PC**
 > 1. Build Configuration (Refer to sony_apps/configs/corvo/zmodem-defconfig)<br>
 >  - See above.
 > 2. Start to send zmodem on the target<br>
 > If using console device like as DebugUART,
 > ~~~~~~~~~~~~~~~
 > nsh> sz -x 1 </mnt/spif/filename>
 > ~~~~~~~~~~~~~~~
 > If using USB CDC/ACM,
 > ~~~~~~~~~~~~~~~
 > nsh> sercon
 > sercon: Registering CDC/ACM serial driver
 > sercon: Successfully registered the CDC/ACM serial driver
 > nsh> sz -d /dev/ttyACM0 -x 1 </mnt/spif/filename>
 > ~~~~~~~~~~~~~~~
 > 3. Receive file via zmodem on the Host PC<br>
 > In case of the TeraTerm, File -> Transfer -> ZMODEM -> Receive..., (you can specify the receiving directory in advance.<br>
 > You can also use other terminal with zmodem transfer function such as minicom.
 > In case of the NuttX Host PC tool, Use the following command.
 > Then a transferred file will be put under the specified or /tmp directory on your Host PC.
 > ~~~~~~~~~~~~~~~
 > $ ./rz -d /dev/ttyXXX
 > ~~~~~~~~~~~~~~~
 > /dev/ttyXXX : your serial device to be connected with HostPC like as ttyS[n]? or ttyUSB[n]<br>
 > 
 > **Building the NuttX Host PC tool**
 > 
 > 1. Compile
 > ~~~~~~~~~~~~~~~
 > $ cd /path/to/sony_apps/system/zmodem
 > $ make -f Makefile.host TOPDIR=/path/to/nuttx APPDIR=/path/to/sony_apps
 > ~~~~~~~~~~~~~~~
 > Created the executable files named as sz and rz under /path/to/sony_apps/system/zmodem.
 > 2. Usage<br>
 > **sz** command
 > ~~~~~~~~~~~~~~~
 > $ ./sz -h
 > USAGE: ./sz [OPTIONS] <lname> [<lname> [<lname> ...]]
 > 
 > Where:
 >         <lname> is the local file name
 > 
 > and OPTIONS include the following:
 >         -d <device>: Communication device to use.  Default: /dev/ttyS0
 >         -r <rname>: Remote file name.  Default <lname>
 >         -x <mode>: Transfer type
 >                 0: Normal file (default)
 >                 1: Binary file
 >                 2: Convert \n to local EOF convention
 >                 3: Resume or append to existing file
 >         -o <option>: Transfer option
 >                 0: Implementation dependent
 >                 1: Transfer if source newer or longer
 >                 2: Transfer if different CRC or length
 >                 3: Append to existing file, if any
 >                 4: Replace existing file (default)
 >                 5: Transfer if source is newer
 >                 6: Transfer if dates or lengths different
 >                 7: Protect: transfer only if dest doesn't exist
 >                 8: Change filename if destination exists
 >         -s: Skip if file not present at receiving end
 >         -h: Show this text and exit
 > ~~~~~~~~~~~~~~~
 > **rz** command
 > ~~~~~~~~~~~~~~~
 > $ ./rz -h
 > USAGE: ./rz [OPTIONS]
 > 
 > Where OPTIONS include the following:
 >         -d <device>: Communication device to use.  Default: /dev/ttyS0
 >         -h: Show this text and exit
 > ~~~~~~~~~~~~~~~
 > 
 > 
 > ------

### Link
- [Using the NuttX’s USB Mass Storage to copy SPI NOR Flash Content](https://acassis.wordpress.com/2016/01/03/using-the-nuttxs-usb-mass-storage-to-copy-spi-nor-flash-content)


--------------------------------------
## NXFFS File System {#nxffs}

See below NuttX wiki page for details of NXFFS.
- [How NXFFS Works](http://nuttx.org/doku.php?id=wiki:vfs:hownxffsworks)

As a example of setup, SPI-Flash is initialized and mounted on /mnt/spif in configs/{board}/cxd56_appinit.c.
- Example:
~~~~~~~~~~~~~~~{.c}
  // Initialize to provide NXFFS on the MTD interface

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      fdbg("ERROR: NXFFS initialization failed: %d\n", ret);
      return ret;
    }

  ret = mount(NULL, "/mnt/spif", "nxffs", 0, NULL);
  if (ret < 0)
    {
      fdbg("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
      return ret;
    }
~~~~~~~~~~~~~~~


### Configuration {#nxffs_config}

- NXFFS Related configurations: See help menu of Kconfig for each detail.
 |CONFIG                                | Description                                               |Example |
 |:-------------------------------------|:----------------------------------------------------------|:-------|
 |CONFIG_FS_NXFFS                       | NXFFS file system                                         |        |
 |CONFIG_NXFFS_SCAN_VOLUME              | Scan volume                                               |n       |
 |CONFIG_NXFFS_PREALLOCATED             | Single, preallocated volume                               |y       |
 |CONFIG_NXFFS_PACKTHRESHOLD            | Re-packing threshold                                      |32      |
 |CONFIG_NXFFS_MAXNAMLEN                | Maximum file name length                                  |255     |
 |CONFIG_NXFFS_TAILTHRESHOLD            | Tail threshold                                            |8192    |

- If use NXFFS, the following configuration must be enabled.
 - CONFIG_FS_NXFFS=y
 - CONFIG_NXFFS_ERASEDSTATE=0xff (default)


--------------------------------------
# eMMC Storage {#emmc}

If your board has eMMC device, you can use eMMC as storage media.<br>
FAT file system is supported as The file system of eMMC.<br>
Also, if your board also USB, you can use the feature of USB Mass Storage.

As a example of setup, eMMC is initialized and mounted on /mnt/vfat in configs/{board}/cxd56_appinit.c.
- Example:
~~~~~~~~~~~~~~~{.c}
  ret = cxd56_emmcinitialize();
  if (ret < 0)
    {
      fdbg("ERROR: Failed to initialize eMMC. %d\n ", errno);
    }
  ret = mount("/dev/emmc0", "/mnt/vfat", "vfat", 0, NULL);
  if (ret < 0)
    {
      dbg("ERROR: Failed to mount the eMMC. %d\n", errno);
    }
~~~~~~~~~~~~~~~


--------------------------------------
## FAT File System # {#fat_emmc}

### Configuration {#fat_config}

- FAT Related configurations: See help menu of Kconfig for each detail.
 |CONFIG                                | Description                                               |Example |
 |:-------------------------------------|:----------------------------------------------------------|:-------|
 |CONFIG_FS_FAT                         | FAT file system                                           |y       |
 |CONFIG_FAT_LCNAMES                    | FAT upper/lower names                                     |n       |
 |CONFIG_FAT_LFN                        | FAT long file names                                       |y       |
 |CONFIG_FAT_MAXFNAME                   | FAT maximum file name size                                |128     |
 |CONFIG_FS_FATTIME                     | FAT timestamps                                            |n       |
 |CONFIG_FAT_FORCE_INDIRECT             | Force direct transfers                                    |n       |
 |CONFIG_FAT_DMAMEMORY                  | DMA memory allocator                                      |n       |
 |CONFIG_FAT_DIRECT_RETRY               | Direct transfer retry                                     |n       |

- If use FAT, the following configuration must be enabled.
 - CONFIG_FS_FAT=y


### How to copy into SPI-Flash by using USB MSC {#fat_util_mkfatfs}

 > Even if your board doesn't have eMMC device, you may use FAT file system.<br>
 > By creating FAT file system's ramdisk on board, so can be shown as USB Mass Storage device to PC.<br>
 > See [NuttShell(NSH)](http://www.nuttx.org/Documentation/NuttShell.html) for details of nsh command.<br>
 > 
 > 1. Build Configuration
 >  - CONFIG_FS_FAT=y
 >  - NSH_LIBRARY=y
 >  - NSH_DISABLE_MKRD=n (enable 'mkrd' command on NutShell)
 >  - NSH_DISABLE_MKFATFS=n (enable 'mkfatfs' command on NutShell)
 >  - NSH_DISABLE_MOUNT=n (enable 'mount' command on NutShell)
 >  - CONFIG_SYSTEM_USBMSC=y
 >  - CONFIG_SYSTEM_USBMSC_DEVPATH1=/dev/ram1 (1: means ramdisk number)
 > 2. Create a FAT FileSystem RAMDISK image on target board
 > ~~~~~~~~~~~~~~~
 > nsh> mkrd -m 1 -s 512 512
 > nsh> mkfatfs /dev/ram1
 > ~~~~~~~~~~~~~~~
 > 3. Start USB Mass Storage
 > ~~~~~~~~~~~~~~~
 > nsh> msconn
 > msconn [3:100]
 > nsh> mcsonn_main: Creating block drivers
 > mcsonn_main: Configuring with NLUNS=1
 > mcsonn_main: handle=d0a1540
 > mcsonn_main: Bind LUN=0 to /dev/ram1
 > mcsonn_main: Connected
 > ~~~~~~~~~~~~~~~
 > 4. Copy any file into USB Mass Storage drive on your PC
 > and, after that, safely removing USB drive
 > ~~~~~~~~~~~~~~~
 > $ cp MYFILE <USB MSC drive> # on your PC side
 > ~~~~~~~~~~~~~~~
 > 5. Stop USB Mass Storage
 > ~~~~~~~~~~~~~~~
 > nsh> msdis
 > ~~~~~~~~~~~~~~~
 > 6. Mount a FAT Filesystem and Copy a file from FAT Filesystem to SPI-Flash
 > ~~~~~~~~~~~~~~~
 > nsh> mount -t vfat /dev/ram1 /tmp
 > nsh> ls /tmp
 > /tmp:
 >  MYFILE
 > nsh> cp /tmp/MYFILE /mnt/spif/myfile
 > nsh> ls -l /mnt/spif
 > /mnt/spif:
 >  ---x--x--x      11 myfile
 > nsh> cat /mnt/spif/myfile
 > sample file
 > ~~~~~~~~~~~~~~~
 > 
 > 
 > ------

--------------------------------------
# SD-Card Storage {#sdcard}

If your board has SD-Card device, you can use SD-Card as removable storage media.<br>
The file system that SD-Card supports is the same as eMMC.<br>

--------------------------------------
## FAT File System # {#fat_sdcard}

Refer to the [eMMC explanation](#fat_emmc).

