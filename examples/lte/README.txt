examples/lte
^^^^^^^^^^^^

  This application is a sample that connect to the LTE network,
  get a file with wget, and disconnect the LTE network.

  Supported LTE modem is ALT1250.

  Build kernel and SDK:

  $ make buildkernel KERNCONF=release_net

  When using this kernel configuration, application can
  access standard socket API, with socket descriptors
  that can be used with NuttX system calls.

  This example can be used by lte default config.

  $ ./tools/config.py examples/lte
  $ make

  Execute under nsh:

  Type 'lte <url>' on nsh like this.
  nsh> lte http://example.com/

