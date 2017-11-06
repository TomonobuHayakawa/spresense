#!/bin/sh

# Do submodule initialization process anyway.
# These commands will not be affected if do again and again.

git submodule init
git submodule update
git submodule foreach git checkout master
