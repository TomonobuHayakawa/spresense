# Welcome to SPRITZER project

Clone this repository and update submodules.

```
$ git clone --recursive git@code.sonymobile.net:spritzer-sdk/spritzer.git
```

After repositories cloned, set main branch for each submodules.
```
$ ./init.sh
```

or checkout each submodules by hands.

```
$ (cd nuttx; git checkout sdk2.2)
$ (cd apps; git checkout sdk2.2)
$ (cd sdk; git checkout master)
$ (cd examples; git checkout master)
$ (cd proprietary; git checkout master)
```

# Directory structure

```
- sdk/         - SPRITZER SDK sources and any PC tools
- examples/    - SPRITZER SDK examples
- proprietary/ - Sony proprietary binaries
- nuttx/       - NuttX original kernel + port of SPRITZER architecture
- apps/        - NuttX original apps
```

# Build

Build instructions are documented at [sdk repository](http://code.sonymobile.net/spritzer-sdk/sdk).
