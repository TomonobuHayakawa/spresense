# examples/dnnrt_lenet

This is a simple example of the LeNet-5's inference using `sdk/modules/dnnrt`.  
This example loads an hand-written digit image (pgm) and a neural network model (nnb) from an SD Card  
and does forward propagation after feeding the image.  
Then, it outputs probabilities of digits[0-9] as classification result.

## Configuration Pre-requisites:

This application depends on `dnnrt` and `sdcard` configuration:

* CONFIG_DNN_RT     - dnnrt
* CONFIG_CXD56_SDIO - SDIO SD Card

## Example Configuration:

* CONFIG_EXAMPLES_DNNRT_LENET           - Enable this example
* CONFIG_EXAMPLES_DNNRT_LENET_PROGNAME  - Program name
* CONFIG_EXAMPLES_DNNRT_LENET_PRIORITY  - Example priority (default: 100)
* CONFIG_EXAMPLES_DNNRT_LENET_STACKSIZE - Example stack size (default: 2048)

## Operation:

Run `dnnrt_lenet` after storing `lenet-5.nnb` and hand-written images onto SD card.

### file arrangement on the host-side:

Train the LeNet-5 model and export it as `lenet-5.nnb`
Detailed instructions for this operation are available in the following page:

http://developer.sony.com/develop/spresense/developer-tools/get-started-using-nuttx/set-up-the-nuttx-environment
(TODO: fix this URL when the corresponding chapter is created)

### file arrangement on SD card:

First, copy a directory, `examples/dnnrt_lenet/lenet-5/`, onto the root of SD card as below.  
The `lenet-5` directory holds 10 hand-written digit images, `[0-9].pgm`, under `lenet-5/data/`.  
These images were drawn by dnnrt developers and their filenames are labels for each image (i.e. 1 is drawn in `1.pgm`).  
You can confirm actual images by `display` command in ImageMagick(http://www.imagemagick.org), etc.

```
$ cd Spresense.git/examples/dnnrt_lenet/
$ cp -r lenet-5 <sd mount point>
```

Next, locate `lenet-5.nnb` inside a neighboring directory, `lenet-5/model/`.

```
$ cp <somewhere>/lenet-5.nnb  <sd mount point>/lenet-5/model/
```

### command usage:

After inserting the SD card, install `nuttx.spk` into CXD5602.  
You can now execute `dnnrt_lenet` on the target-side console.  
This command usage is as below:

```
SYNOPSIS
       dnnrt_lenet [nnb] [pgm]
DESCRIPTION
       dnnrt_lenet instantiates a neural network defined by nnb,
       and feeds an pgm image into it. Default values of nnb and pgm
       are `/mnt/sd0/lenet-5/model/lenet-5.nnb` and
       `/mnt/sd0/lenet-5/data/0.pgm`, respectively.
```

### expected output:

`dnnrt_lenet` prints a 1D-array which `lenet-5.nnb` outputs.  
If classification succeeds, an element which it's index correspond to an image filename should be the largest.  
For example, if `lenet-5/data/3.pgm` is given into `lenet-5.nnb`, you can expect output like below:

```
nsh> dnnrt_lenet /mnt/sd0/lenet-5/model/lenet-5.nnb /mnt/sd0/lenet-5/data/3.pgm
load nnb file: /mnt/sd0/lenet-5/model/lenet-5.nnb
load pnm image: /mnt/sd0/lenet-5/data/3.pgm # 3 is hand-written
...
start dnn_runtime_forward()
output[0]=0.000000
output[1]=0.000000
output[2]=0.000000
output[3]=0.999976 # probability that 3 is written in the given image
output[4]=0.000000
output[5]=0.000017
output[6]=0.000000
output[7]=0.000000
output[8]=0.000006
output[9]=0.000000
...
```
