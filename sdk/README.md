Spritzer is the code name of the product CXD5602 produced by Sony Semiconductor
Solutions Corporation.  
The CXD5602 is ARM powered SoC, including many peripherals for IoT and
wearable solutions.
  
SPRITZER SDK is based on NuttX. So please refer to [original NuttX site](http://www.nuttx.org/) to see basic
kernel information.

This SDK constructed by series of libraies, drivers and architecture specific
code, these helps to you getting started and create faster for your products.


# Directory structure

```
sdk
|-- bsp            - Board support package
|   |-- board      - Board specific code
|   |-- include    - SDK headers
|   |-- scripts    - Linker scripts
|   `-- src        - Common source
|-- configs        - Default configuration files
|   `-- kernel
|-- drivers        - External device drivers
|   |-- lcd
|   `-- sensors
|-- lib            - Kernel and SDK library output directory
|-- modules
|   |-- asmp       - ASMP framework
|   |-- audio      - Audio library
|   |-- bluetooth  - Bluetooth
|   |-- include    - Library headers
|   |-- nfc        - NFC library
|   |-- sensing    - Smart sensing library
|   |   |-- arm_gesture   - Arm gesture detection
|   |   |-- barometer     - Barometer
|   |   |-- compass       - Compass
|   |   |-- gnss          - GNSS
|   |   |-- step_counter  - Step counter
|   |   |-- tap           - Tap detection
|   |   `-- tram          - TRAM
|   `-- skeleton
|-- system         - System commands
`-- tools          - Build utilities
```

# Build instruction

Getting started

```
$ make buildkernel KERNCONF=release
$ tools/config.py default
$ make
```

SDK and NuttX kernel's configuration and build sequence are separated.

# Configuration

SDK provides `tools/config.py` as configuration frontend for SDK and NuttX
kernel configuration. `tools/config.py` invokes kconfig tool, so user must
be installed before use it.
`tools/config.py` must be called from top of SDK source tree.

`tools/config.py` can be switched with `-k` or `--kernel` option to affects
kernel side.
If no `-k` or `--kernel` option has been specified, then affects SDK side.

e.g.
```
$ tools/config.py --kernel --list (list kernel predefined configs)
$ tools/config.py --list  (list SDK predefined configs)
```

At the first time, or you want to change to predefined ones, specify
predefined `<config name>`s at the last of options.

```
$ tools/config.py <config name> [<config name>...]
```

You can specify multiple predefines, it allows combination of predefines.

e.g.
```
$ tools/config.py audio gnss
```

`tools/config.py` can be invoked menu style configuration UI by following
options.

```
$ tools/config.py --menuconfig (same as 'make menuconfig')
$ tools/config.py --qconfig    (same as 'make qconfig')
$ tools/config.py --gconfig    (same as 'make gconfig')
```

Additionally, you can use menu invokation with predefined configs like this:

```
$ tools/config.py --menuconfig default
```

This command replaces configration to "default" config and invoke menuconfig.

# Build

SDK build system has been separated from NuttX, it to be able to build NuttX
kernel indipendently, and build SDK faster.
User must be build kernel before SDK build by following command.

```
$ make buildkernel
```

If you don't interest about kernel sources, then never been built again.
And you can specify kernel default config from `KERNCONF`, configure specified
config and build at the same time.

```
$ make buildkernel KERNCONF=<config name>
```

Finally, build SDK drivers and libraries.

```
$ make
```

After built successfully, you can see `nuttx` and `nuttx.spk` files in top of
SDK source tree.
