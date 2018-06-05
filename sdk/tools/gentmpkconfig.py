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

EXCLUDES = ['nuttx', 'apps', 'sdk']

# Search top of Kconfig in the same level directories

kconfigs = glob.glob('../*/Kconfig')
for c in kconfigs:
    dn = os.path.dirname(c).split('/')[-1]

    # Add Kconfig excpet nuttx, apps and sdk directories

    if dn not in EXCLUDES:
        print('source "%s"' % c, file=outfile)

outfile.close()
