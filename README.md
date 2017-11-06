# Welcome to SPRITZER project

Clone this repository and update submodules.

```
$ git clone --recursive git@code.sonymobile.net:spritzer-sdk/spritzer.git
```

After repositories cloned, each submodules are in 'Detached HEAD'.
So you must checkout master before you getting started.
If you want to all of repository into master, just type like this.

```
$ git submodule foreach git checkout master
```

# Submodules

```
- sdk         - SPRITZER SDK sources and any PC tools
- examples    - SPRITZER SDK examples
- proprietary - Sony proprietary binaries
- nuttx       - NuttX original kernel + port of SPRITZER architecture
- apps        - NuttX original apps
```

# Build

Build instructions are documented at [sdk repository](http://code.sonymobile.net/spritzer-sdk/sdk).
