#!/bin/sh

SUBDIRS='sdk'

git submodule init
git submodule update

# Checkout NuttX sources as our branch

(cd nuttx; git checkout sdk2.2) || exit
(cd apps; git checkout sdk2.2) || exit

# Checkout our sources as master

for d in $SUBDIRS
do
    (cd $d; git checkout master) || exit
done
