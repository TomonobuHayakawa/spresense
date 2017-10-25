#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import logging
import glob
import shutil

MODE_MENUCONFIG = "menuconf"
MODE_QCONFIG = "qconfig"
MODE_GCONFIG = "gconfig"

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

def apply_defconfig(configname, configlist, topdir, sdkdir, kernel):
    defconfig = configname + '-defconfig'
    logging.debug('Using config file: %s', defconfig)

    if defconfig not in configs:
        print('Error: config "%s" not found' % configname, file=sys.stderr)
        sys.exit(3)

    # Copy Make.defs file first, because SDK Makefile depends on Make.defs in
    # kernel, but there is nothing if kernel not configured.

    srcmakedefs = os.path.join(sdkdir, 'bsp', 'scripts', 'Make.defs.nuttx')
    destmakedefs = os.path.join(topdir, 'Make.defs')
    install(srcmakedefs, destmakedefs)

    if kernel:
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
    return ret

def do_kconfig_conf(mode, sdkdir):
    command = 'make %s' % mode
    if logging.getLogger().getEffectiveLevel() > logging.INFO:
        command += ' 2>&1 >/dev/null'
    ret = os.system(command)
    return ret

if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(description='Configration tool')
    parser.add_argument('configname', metavar='<config name>', type=str, nargs='?',
                        help='configuration name')
    parser.add_argument('-k', '--kernel', action='store_true',
                        help='kernel config')
    parser.add_argument('-l', '--list', action='store_true',
                        help='list default configurations.\nshow kernel defconfigs with --kernel.')
    parser.add_argument('-m', '--menuconfig', action='store_true',
                        help='run config in "menuconfig"')
    parser.add_argument('-q', '--qconfig', action='store_true',
                        help='run config in "qconfig"')
    parser.add_argument('-g', '--gconfig', action='store_true',
                        help='run config in "gconfig"')
    parser.add_argument('-v', '--verbose', action='count',
                        help='verbose messages')
    opts = parser.parse_args()

    loglevel = logging.WARNING
    if opts.verbose == 1:
        loglevel = logging.INFO
    if opts.verbose == 2:
        loglevel = logging.DEBUG
    logging.basicConfig(level=loglevel)

    menumode = None
    if opts.menuconfig: menumode = MODE_MENUCONFIG
    if opts.qconfig:    menumode = MODE_QCONFIG
    if opts.gconfig:    menumode = MODE_GCONFIG

    # Setup paths

    sdkdir = os.getcwd()
    topdir = os.path.abspath(os.path.join(sdkdir, '..', 'nuttx'))
    configdir = os.path.join(sdkdir, 'configs')
    kconfigdir = os.path.join(sdkdir, 'configs', 'kernel')

    if not os.path.isdir(configdir) or not os.path.isdir(kconfigdir):
        print('Config directory not found.', file=sys.stderr)
        sys.exit(2)

    if opts.kernel:
        configs = get_defconfigs(kconfigdir)
    else:
        configs = get_defconfigs(configdir)

    if opts.list:
        print('Available configurations:')

        for c in configs:
            print('\t%s' % c.replace('-defconfig', ''))

        sys.exit(0)

    if opts.configname:
        ret = apply_defconfig(opts.configname, configs, topdir, sdkdir, opts.kernel)
        if ret != 0:
            sys.exit(ret)

    if menumode:
        if opts.kernel:
            menumode += 'kernel'
        do_kconfig_conf(menumode, sdkdir)
