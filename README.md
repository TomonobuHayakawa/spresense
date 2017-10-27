# Welcome to SPRITZER project

Clone this repository and update submodules.

```
$ git clone --recursive git@code.sonymobile.net:spritzer-sdk/spritzer.git
```

After repositories cloned, set main branch for each submodules.

```
$ (cd nuttx; git checkout sdk2.2)
$ (cd apps; git checkout sdk2.2)
$ (cd sdk; git checkout master)
```

# Directory structure

- sdk/   - Spritzer SDK sources and any PC tools
- nuttx/ - NuttX original kernel + port of SPRITZER architecture
- apps/  - NuttX original apps

# Build

Build instructions are documented at [sdk repository](http://code.sonymobile.net/spritzer-sdk/sdk).
