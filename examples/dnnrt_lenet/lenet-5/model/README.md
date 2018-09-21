# README.md

## Filename format

In this directory, each pair of .yaml and .nnb is named like below:  
(weight_dtype: 'float32' | 'fixed8' | 'fixed8')

| name | description |
| ---- | ---- |
| 'lenet-5'| * all _variables_ are `FLOAT32`<br>* dnnrts's hardware acceleration is enabled |
| 'lenet-5_' weight_dtype | * _vairables_ input to  `nn.parametric_functions` are quantized as fixed-point values <br> * dnnrts's hardware acceleration is enabled |
| 'lenet-5_' weight_dtype '_no-hwa' | * _vairables_ of parameter are quantized as fixed-point values <br> * dnnrts's hardware acceleration is disabled  |

## Generation method

```bash
#! /bin/bash

for yaml_fname in $(ls *.yaml); do
    nnb_fname=${yaml_fname:0:-4}nnb
    nnabla_cli convert -b 1 -s ${yaml_fname} lenet-5.nnp ${nnb_fname}
done
```