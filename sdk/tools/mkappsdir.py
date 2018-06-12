#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import re

TOOL_DESCRIPTION = '''
Create a new application directory at the outside of sdk repository

This tool must be run on sdk directory.
'''

TEMPLATENAME = 'examples'

def replaced_copy(srcfile, dstfile, appname):
    with open(srcfile, 'r') as src:
        with open(dstfile, 'w') as dst:
            for line in src:
                l = line.replace(TEMPLATENAME, appname)
                dst.write(l)

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                     description=TOOL_DESCRIPTION)
    parser.add_argument('dirname', metavar='<dir name>', type=str,
                        help='New application directory name')
    parser.add_argument('desc', type=str, nargs="?", help='Menu description')
    parser.add_argument('-v', '--verbose', action='count', default=0,
                        help='verbose messages')
    opts = parser.parse_args()

    verbose = opts.verbose

    srcdir = os.path.join('..', TEMPLATENAME)
    targetdir = os.path.join('..', opts.dirname)

    # Sanity checks

    if re.search(r'\s', opts.dirname):
        print('Any white spaces not allowed in <dir name>', file=sys.stderr)
        sys.exit(1)

    if not os.path.exists(srcdir):
        print('examples directory not found', file=sys.stderr)
        sys.exit(2)

    try:
        os.mkdir(targetdir)
    except:
        print('%s already exists' % targetdir, file=sys.stderr)
        sys.exit(3)

    # Required files for extending application series to outside of sdk repos.

    filelist = ['LibTarget.mk', 'Makefile', 'Make.defs', 'Application.mk', '.gitignore']

    for f in filelist:
        src = os.path.join(srcdir, f)
        dst = os.path.join(targetdir, f)
        if verbose > 0:
            print('Copying file %s -> %s' % (src, dst))
        replaced_copy(src, dst, opts.dirname)

    # Finally, replace menu description

    makefile = os.path.join(targetdir, 'Makefile')

    if opts.desc:
        desc = opts.desc
    else:
        desc = opts.dirname.capitalize()

    pat = re.compile(r'MENUDESC\s*=\s*".*"')
    with open(makefile, "r") as f:
        buf = f.read()
    buf = pat.sub('MENUDESC = "%s"' % desc, buf)
    with open(makefile, "w") as f:
        f.write(buf)
