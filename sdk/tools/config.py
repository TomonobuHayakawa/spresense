#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import logging
import glob
import shutil

def get_defconfigs(directory):
    return list(map(lambda x: os.path.basename(str(x)),
                    glob.glob(os.path.join(directory, '*-defconfig'))))

def install(src, dest, mode=0o644):
    logging.debug(src)
    logging.debug(dest)
    logging.debug(mode)

    shutil.copy(src, dest)
    os.chmod(dest, mode)
    return

if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(description='Configration tool')
    parser.add_argument('configname', metavar='<config name>', type=str, nargs='?',
                        help='configuration name')
    parser.add_argument('--kernel', action='store_true',
                        help='kernel config')
    parser.add_argument('-l', '--list', action='store_true',
                        help='list default configurations.\nshow kernel defconfigs with --kernel.')
    parser.add_argument('-v', '--verbose', action='count',
                        help='verbose messages')
    opts = parser.parse_args()

    loglevel = logging.WARNING
    if opts.verbose == 1:
        loglevel = logging.INFO
    if opts.verbose == 2:
        loglevel = logging.DEBUG
    logging.basicConfig(level=loglevel)
    
    sdkdir = os.getcwd()
    topdir = os.path.abspath(os.path.join(sdkdir, '..', 'nuttx'))
    configdir = os.path.join(sdkdir, 'configs')
    kconfigdir = os.path.join(sdkdir, 'configs', 'kernel')

    if not os.path.isdir(configdir) or not os.path.isdir(kconfigdir):
        print('Config directory not found.', file=sys.stderr)
        sys.exit(1)

    if opts.kernel:
        configs = get_defconfigs(kconfigdir)
    else:
        configs = get_defconfigs(configdir)

    if opts.list:
        print('Available configurations:')

        for c in configs:
            print('\t%s' % c.replace('-defconfig', ''))

        sys.exit(0)

    if opts.configname == None:
        print('Error: config name must be specified.', file=sys.stderr)
        parser.print_help()
        sys.exit(2)

    defconfig = opts.configname + '-defconfig'
    logging.debug('Using config file: %s', defconfig)

    if defconfig not in configs:
        print('Error: config "%s" not found' % opts.configname, file=sys.stderr)
        sys.exit(3)

    # Copy Make.defs file first, because SDK Makefile depends on Make.defs in
    # kernel, but there is nothing if kernel not configured.

    srcmakedefs = os.path.join(sdkdir, 'bsp', 'scripts', 'Make.defs.nuttx')
    destmakedefs = os.path.join(topdir, 'Make.defs')
    install(srcmakedefs, destmakedefs)

    if opts.kernel:
        src = os.path.join(kconfigdir, defconfig)
        dest = os.path.join(topdir, '.config')
        install(src, dest)
        postproc = 'make -C %s olddefconfig' % topdir
    else:
        src = os.path.join(configdir, defconfig)
        dest = os.path.join(sdkdir, '.config')
        install(src, dest)
        postproc = 'make olddefconfig' 

    if logging.getLogger().getEffectiveLevel() > logging.INFO:
        postproc += ' 2>&1 >/dev/null'
    ret = os.system(postproc)
    if ret != 0:
        print('Post process failed. %d' % ret)

    sys.exit(ret)
