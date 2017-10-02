#!/usr/bin/env python3

import os
import sys
import re

if 'TOPDIR' in os.environ:
    topdir = os.path.abspath(os.environ['TOPDIR'])
else:
    topdir = os.getcwd()

configfile = os.path.join(topdir, '.config')
if not os.path.exists(configfile):
    print(".config file not found.", file=sys.stderr)
    sys.exit(1)

# Now let's convert each configurations to C source header

print("/*  */")
print("#ifndef __BSP_INCLUDE_SDK_CONFIG_H")
print("#define __BSP_INCLUDE_SDK_CONFIG_H")
print()

with open(configfile, 'r') as f:
    for line in f:
        line = re.sub(r'(.*)#.*', r'\1', line).strip()
        if len(line) == 0:
            continue
        
        m = re.match(r'(?P<name>.*)=(?P<value>.*)', line)
        if not m:
            print("Illegal config format.", file=sys.stderr)
            sys.exit(2)
        name = m.group('name')
        value = m.group('value')

        if value == 'y':
            print("#define", name, '1')
        else:
            print("#define", name, value)

print()
print("#endif /* __BSP_INCLUDE_SDK_CONFIG_H */")
