#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import re

TOOL_DESCRIPTION = '''
Create a new application

  This tool create a new application in current directory.
  So you need to move to where you want before run this tool.

  e.g.
  $ cd examples
  $ ../sdk/tools/mkapp.py myapp
'''

KCONFIG_TMPL = '''
config {configname}
	bool "{appname} app"
	default n
	---help---
		Enable the {appname} app

if {configname}

config {configname}_PROGNAME
	string "Program name"
	default "{appname}"
	depends on BUILD_KERNEL
	---help---
		This is the name of the program that will be use when the NSH ELF
		program is installed.

config {configname}_PRIORITY
	int "{appname} task priority"
	default 100

config {configname}_STACKSIZE
	int "{appname} stack size"
	default 2048

endif
'''

MAKEFILE_TMPL = '''
-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

CONFIG_{configname}_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_{configname}_STACKSIZE ?= 2048

APPNAME = {appname}
PRIORITY = $(CONFIG_{configname}_PRIORITY)
STACKSIZE = $(CONFIG_{configname}_STACKSIZE)

ASRCS =
CSRCS =
MAINSRC = {appname}_main.c

CONFIG_{configname}_PROGNAME ?= {appname}$(EXEEXT)
PROGNAME = $(CONFIG_{configname}_PROGNAME)

include $(APPDIR)/Application.mk
'''

MAKEDEFS_TMPL = '''
ifeq ($(CONFIG_{configname}),y)
CONFIGURED_APPS += {appname}
endif
'''

MAINCSRC_TMPL = '''
#include <sdk/config.h>
#include <stdio.h>

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int {appname}_main(int argc, char *argv[])
#endif
{{
  return 0;
}}
'''

GITIGNORE = '''/Make.dep
/.depend
/.built
/*.asm
/*.obj
/*.rel
/*.lst
/*.sym
/*.adb
/*.lib
/*.src
'''

def create_from_template(template, filename, appname, configname):
    with open(filename, "w") as f:
        f.write(template.format(appname=appname, configname=configname))

if __name__ == '__main__':

    import argparse

    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                     description=TOOL_DESCRIPTION)
    parser.add_argument('appname', metavar='<app name>', type=str, help='New application name')
    parser.add_argument('desc', type=str, nargs="?", help='Menu description')
    parser.add_argument('-v', '--verbose', action='count', default=0, help='verbose messages')
    opts = parser.parse_args()

    verbose = opts.verbose

    appname = opts.appname
    optprefix = os.path.basename(os.getcwd()).upper()
    configname = optprefix + '_' + appname.upper()
    maincsrcfile = appname + '_main.c'
    targetdir = appname

    # Sanity checks

    if re.search(r'\s', opts.appname):
        print('Any white spaces not allowed in <appname>', file=sys.stderr)
        sys.exit(1)

    try:
        os.mkdir(targetdir)
    except:
        print('%s already exists' % targetdir, file=sys.stderr)
        sys.exit(3)

    # Create application

    os.chdir(targetdir)

    create_from_template(KCONFIG_TMPL, 'Kconfig', appname, configname)
    create_from_template(MAKEFILE_TMPL, 'Makefile', appname, configname)
    create_from_template(MAKEDEFS_TMPL, 'Make.defs', appname, configname)
    create_from_template(MAINCSRC_TMPL, maincsrcfile, appname, configname)

    with open('.gitignore', "w") as f:
        f.write(GITIGNORE)

    print("New %s app successfully created. Please 'make clean' from sdk first.")

