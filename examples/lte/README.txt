examples/lte
^^^^^^^^^^^^

  This application is a sample that connect to the LTE network
  and obtain the requested file from the HTTP server using the GET method.
  The obtained file is output to standard output.

  Supported LTE modem is ALT1250.

  Build kernel and SDK:

  You have to specify "KERNCONF=release_net" for kernel configuration
  as below. This application can not be used if you specified "KERNCONF=release".

  $ make buildkernel KERNCONF=release_net

  This application can be used by lte default config.

  $ ./tools/config.py examples/lte
  $ make

  Execute under nsh:

  Type 'lte <url>' on nsh like this.
  nsh> lte http://example.com/index.html

