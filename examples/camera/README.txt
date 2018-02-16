examples/camera
^^^^^^^^^^^^^^^

  This sample code gets an image from the image sensor.

  Supported image sensor is ISX012.


  This example can be used by camera default config.

  $ ./tools/config.py camera

  Alternatively, this example can be output captured images to LCD directly.
  If you want to work with LCD devices, type like this:

  $ ./tools/config.py camera ili9340

  And enable following options.

  CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD -- Show captured image on the LCD
  CONFIG_EXAMPLES_CAMERA_INFINITE   -- Capture infinitely

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
