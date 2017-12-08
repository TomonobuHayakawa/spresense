#!/bin/sh

package_name=spritzer_sdk_beta
rev=HEAD
modules=`git submodule status | cut -d ' ' -f 3`
tmpdir=`mktemp -d`

# Copy files into temporary directory

for m in ${modules}
do
    echo Processing module: ${m}
    (cd ${m}; \
     git archive --format=tar --prefix=${package_name}/${m}/ ${rev} | \
         (cd ${tmpdir} && tar xf -)) || exit 1
done
(git archive --format=tar --prefix=${package_name}/ ${rev} | \
        (cd ${tmpdir} && tar xf -)) || exit 2

topdir=$PWD
cd $tmpdir
echo Create archive: ${topdir}/${package_name}.zip
zip -r9 -q ${topdir}/${package_name}.zip ${package_name} || exit 3
cd $topdir

rm -rf $tmpdir
