examples/camera
^^^^^^^^^^^^^^^

  This sample code gets an image from the image sensor.

  Supported image sensor is ISX012.


  This example can be used by camera default config.

  $ ./tools/config.py camera

  This example can be output captured images to LCD directly.
  If you want to work with LCD devices, type like this:

  $ ./tools/config.py camera ili9340

  And enable following options.

  CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD -- Show captured image on the LCD
  CONFIG_EXAMPLES_CAMERA_INFINITE   -- Capture infinitely

  It is also possible to output to ramdisk and retrieve files via usb drive.
  If you want to work with ramdisk, type like this:

  CONFIG_SYSTEM_USBMSC
  CONFIG_SYSTEM_USBMSC_DEVPATH1 "/dev/ram1"

  Execute under nsh:

  nsh> camera

  * Capture to ramdisk and retrieve files via usb drive

  nsh> camera
  FILENAME:/mnt/vfat/VIDEO001.JPG
  nsh> msconn
  mcsonn_main: Creating block drivers
  mcsonn_main: Configuring with NLUNS=1
  mcsonn_main: handle=d077a40
  mcsonn_main: Bind LUN=0 to /dev/ram1
  mcsonn_main: Connected

  * Display to LCD

  nsh> camera moni
  nximage_initialize: Initializing LCD
  nximage_initialize: Open NX
  nximage_initialize: Screen resolution (320,240)
