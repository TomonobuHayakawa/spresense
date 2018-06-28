#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import logging
import re

CONFIG_APPS_DIR = 'CONFIG_APPS_DIR="../sdk/tools/empty_apps"'

def make_savedefconfig(d):
    ''' Run 'make savedefconfig' at specified directory
    '''
    command = 'make -C ' + d + ' savedefconfig'
    if logging.getLogger().getEffectiveLevel() > logging.INFO:
        command += ' 2>&1 >/dev/null'

    logging.debug('command: "%s"', command)

    return os.system(command)

def confirm(msg):
    ''' Tell to user with msg
    '''
    while True:
        try:
            res = input(msg + ' (y/n)? ')
        except KeyboardInterrupt:
            print()
            sys.exit(0)
        if res == 'y':
            return True
        if res == 'n':
            return False

def save_default_config(basedir, confpath, kernel):
    ret = make_savedefconfig(basedir)
    if ret != 0:
        print("Create defconfig failed. %d" % (ret), file=sys.stderr)
        sys.exit(ret)

    defconfig = os.path.join(basedir, 'defconfig')
    if os.path.exists(confpath):
        yes = confirm(configname + ' is already exists, overwrite? ')
        if not yes:
            sys.exit(0)

    logging.debug("Output: %s\n" % (confpath))
    if kernel:
        os.replace(defconfig, confpath)
        with open(confpath, 'a') as dest:
            dest.write(CONFIG_APPS_DIR + "\n")
    else:
        with open(confpath, 'w') as dest:
            with open(defconfig, 'r') as src:
                for line in src:
                    if not re.match(r'CONFIG_BOARD_.*', line):
                        dest.write(line)

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(description='Make default config from current config')
    parser.add_argument('configname', metavar='<config name>', type=str, nargs=1,
                        help='configuration name')
    parser.add_argument('-k', dest='kernel', action='store_true',
                        help='save kernel configuration')
    parser.add_argument('-d', '--dir', type=str, nargs=1,
                        help='change configs directory')
    parser.add_argument('--all', action='store_true',
                        help='Save SDK and kernel configuration with same name')
    parser.add_argument('--verbose', '-v', action='count',
                        help='verbose messages')

    opts = parser.parse_args()

    configname = opts.configname[0]

    savesdk = True
    savekernel = False
    if opts.kernel:
        savesdk = False
        savekernel = True
    if opts.all:
        savesdk = True
        savekernel = True

    loglevel = logging.WARNING
    if opts.verbose == 1:
        loglevel = logging.INFO
    if opts.verbose == 2:
        loglevel = logging.DEBUG
    logging.basicConfig(level=loglevel)

    sdkdir = os.getcwd()
    topdir = os.path.abspath(os.path.join(sdkdir, '..', 'nuttx'))

    # This script allows run in 'sdk'

    if not os.path.isdir('../nuttx'):
        print('Error: NuttX directory not found.', file=sys.stderr)
        sys.exit(1)

    # Test kernel and sdk already configured

    if savesdk and not os.path.exists(os.path.join(sdkdir, '.config')):
        print('Error: SDK not configured.', file=sys.stderr)
        sys.exit(2)

    if not os.path.exists(os.path.join(topdir, '.config')):
        print('Error: Kernel not configured.', file=sys.stderr)
        sys.exit(3)

    # Setup all of paths

    configdir = os.path.join(sdkdir, 'configs')
    kconfigdir = os.path.join(sdkdir, 'configs', 'kernel')

    # If -d options has been specified, then replace base config directory to
    # specified ones.

    if opts.dir:
        d = opts.dir[0]
        configdir = d
        kconfigdir = os.path.join(d, 'kernel')

    if savekernel:
        if not os.path.isdir(kconfigdir):
            print('Error: kernel configuration directory not found.', file=sys.stderr)
            sys.exit(4)
    elif savesdk:
        if not os.path.isdir(configdir):
            print('Error: configuration directory not found.', file=sys.stderr)
            sys.exit(5)

    logging.debug('Kernel dir: %s', topdir)
    logging.debug('SDK dir   : %s', sdkdir)
    logging.debug('Config dir: %s\n', configdir)

    # Do 'savedefconfig' target

    if savesdk:
        confpath = os.path.join(configdir, configname + '-defconfig')
        save_default_config(sdkdir, confpath, False)

    if savekernel:
        confpath = os.path.join(kconfigdir, configname + '-defconfig')
        save_default_config(topdir, confpath, True)
