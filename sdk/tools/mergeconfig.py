#!/usr/bin/env python3

import os
import sys
import argparse
import re

def getconfigname(str):
    m = re.match(r'# (?P<name>CONFIG_[\w]+) is .*', str)
    if m:
        return m.group('name')
    m = re.match(r'(?P<name>CONFIG_[\w]+)=y', str)
    if m:
        return m.group('name')
    return None

parser = argparse.ArgumentParser(description='')
parser.add_argument('kernelconfig', metavar='<kernel .config>', type=str, help='kernel .config file')
parser.add_argument('sdkconfig', metavar='<SDK .config>', type=str, help='SDK .config file')
parser.add_argument('tmpdir', metavar='tmpdir', type=str, nargs='?', default=".config.tmp")

opts = parser.parse_args()

# Check input .config files and setup work directory

if not os.path.exists(opts.kernelconfig):
    print("Kernel config %s not found." % opts.kernelconfig)
    sys.exit(1)

if not os.path.exists(opts.sdkconfig):
    print("SDK config %s not found." % opts.sdkconfig)
    sys.exit(1)

if os.path.exists(opts.tmpdir):
    if not os.path.isdir(opts.tmpdir):
        print("Temporary directory creation failure.")
        sys.exit(1)
else:
    os.makedirs(opts.tmpdir)


# Generate output file path

destconfigfile = os.path.join(opts.tmpdir, ".config")

with open(destconfigfile, 'w') as dest:
    # Read kernel configuration and output first.

    with open(opts.kernelconfig, 'r') as src:
        kernelconfig = src.read()
        dest.write(kernelconfig)

    # Append SDK configs except duplicated configs. (only for enabled)

    with open(opts.sdkconfig, 'r') as src:
        for line in src:
            cn = getconfigname(line)

            if cn and '%s=y' % cn in kernelconfig:
                continue
            dest.write(line)
