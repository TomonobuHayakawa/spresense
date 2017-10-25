#!/usr/bin/env python3

import os
import sys

# Append external directories to SDK Kconfig

EXTDIRS = ['examples', 'demo', 'test', 'diag']

if len(sys.argv) < 2:
    print('output file not specified.', file=sys.stderr)
    sys.exit(1)

with open('Kconfig', 'r') as f:
    buf = f.read()

if sys.argv[1] == '-':
    outfile = sys.stdout
else:
    outfile = open(sys.argv[1], 'w')

print(buf, file=outfile)

# Search exact directories and top of Kconfig

for d in EXTDIRS:
    path = os.path.join('..', d, 'Kconfig')
    if os.path.exists(path):
        print('source "../%s/Kconfig"' % d, file=outfile)

outfile.close()
