# dnnrt #

_dnnrt_ is an Deep Neural Networks RunTime optimized for for CXD5602.  
This runtime is developed through extending _nnabla-c-runtime_ and  
it does forward propagation based on a model file trained by [_Neural Network Console_](http://dl.sony.com) (NNC) or [_Neural Network Library_](http://nnabla.org/) (NNL).  
Users can bring a neural network to CXD5602 as steps below:

0. save a neural network model as an _nnp_ file ([Tutorial](https://support.dl.sony.com/docs-ja/%E3%83%81%E3%83%A5%E3%83%BC%E3%83%88%E3%83%AA%E3%82%A2%E3%83%AB%EF%BC%9Aneural-network-console%E3%81%AB%E3%82%88%E3%82%8B%E5%AD%A6%E7%BF%92%E6%B8%88%E3%81%BF%E3%83%8B%E3%83%A5%E3%83%BC%E3%83%A9/) outside this file)
1. convert the nnp file to a lightweight binary format, _nnb_ ([Model Conversion](#model-conversion))
2. configure NuttX to link dnnrt ([Configuration](#configuration))
3. write an application of dnnrt ([Application](#application))

The remaining of this file describes about each step in basic usage.  

## Model Conversion ##

Convert an nnp file to an nnb file by `nnabla_cli`.

```bash
$ nnabla_cli convert -b 1 lenet-5.nnp lenet-5.nnb
```

## Configuration ##

Start configuration of Spresense SDK by following commands. 
```
# configure SDK
cd Spresense.git/sdk
./tools/config.py -k release;
./tools/config.py board/spresense device/sdcard feature/dnn examples/dnn
```

## Application ##

In application code, use dnnrt according to the following 4 steps:

1. load an .nnb file into memory
2. initialize a runtime object
3. fill a buffer with data and do forward propagation 
4. finalize the runtime object

Users can easily get familiar with dnnrt API through 
[examples/dnnrt_lenet](../../examples/dnnrt_lenet/), which is one of the simplest application.
Detailed description of each function is available at [sdk/modules/include/dnnrt/runtime.h](../include/dnnrt/runtime.h).  
Doxygen-based documentation will be supported in future updates.  

### Build dnnrt example ###

In the case of SPRESENSE board, execute the following commands on the host side after SDK has been configured.

```bash
# build kernel and SDK
make buildkernel
make
```

### Run dnnrt example ###

Install NuttX and necessary files into CXD5602 like below.
Then, execute a command, `dnnrt_lenet`, on the console.

1. install `nuttx.spk` into CXD5602 flash
2. copy the following files from `Spresense.git/examples/dnnrt_lenet/lenet-5` onto non-volatile memory visible from NuttX file system
  - nnb file:   model/lenet-5.nnb
  - image file: data/00000.pgm
3. power on CXD5602 and enter the following command in nsh shell:

```bash
$ dnnrt_lenet <nnb file path> <pgm image path>
  default value: /mnt/sd0/lenet-5/model/lenet-5.nnb
                 /mnt/sd0/lenet-5/data/00000.pgm
```
