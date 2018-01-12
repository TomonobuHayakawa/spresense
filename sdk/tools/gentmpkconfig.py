#!/usr/bin/env python3

import os
import sys
import glob

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

# Search top of Kconfig in the same level directories

kconfigs = glob.glob('../*/Kconfig')
for c in kconfigs:

    # Add Kconfig excpet nuttx, apps and sdk directories

    if 'nuttx' not in c and 'apps' not in c and 'sdk' not in c:
        print('source "%s"' % d, file=outfile)

outfile.close()
