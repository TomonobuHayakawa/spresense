# Welcome to SPRITZER project

Clone this repository and update submodules.

```
$ git submodule init
$ git submodule update
```

`git submodule update` may take a long time, please wait.

After submodules updated, set main branch for each submodules.

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
