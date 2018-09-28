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

## Usage:

Run `dnnrt_lenet` after locating pgm and nnb files onto SD card.
Detailed instructions including model file generation are written in a tutorial document.