examples/camera
^^^^^^^^^^^^^^^

  Example for CAMERA test.

  This sample code gets an image from the image sensor.

  Supported image sensor is ISX012.


  Select the following configuration options:

  $ make buildkernel KERNCONF=nsh
  $ ./tools/config.py camera

  To use the LCD, select the following configuration options:

  $ make buildkernel KERNCONF=lcd
  $ ./tools/config.py camera ili9340

  In addition to the above, the following definitions are also selectabled:

  CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD -- Display to LCD example. Default: n
  CONFIG_EXAMPLES_CAMERA_INFINITE   -- Capture infinitely. Default: n

  Execute under nsh:

  nsh> camera
  frame: 1 at 0d032340 (size 00025800) (code=0)
  frame: 1 at 0d032340 (size 00025800) (code=0)
    :

  * Display to LCD

  nsh> camera
  nximage_initialize: Initializing LCD
  nximage_initialize: Open NX
  nximage_initialize: Screen resolution (320,240)
  frame: 1 at 0d031320 (size 00025800) (code=0)
  frame: 1 at 0d031320 (size 00025800) (code=0)
    :
