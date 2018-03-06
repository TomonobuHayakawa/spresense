#!/bin/sh

TOPDIR=${TOPDIR:-nuttx}
TAG=${1:-HEAD}

APP_VERSION="0.0.0"
SDK_VERSION="SDK2.2"
if [ -r sdk_version ]; then
    SDK_VERSION="SDK`cat sdk_version`"
fi

NUTTX_VERSION="7.22"

# Get short hash for specified tag
GIT_REVISION=`git rev-parse ${TAG} | cut -b -7`

BUILD_ID="${APP_VERSION}-${SDK_VERSION}-${GIT_REVISION}"

# BUILD_ID must be 40 characters or less
if [ ${#BUILD_ID} -gt 40 ]; then
    echo "BUILD_ID too long! ${BUILD_ID}"
    exit 1
fi

${TOPDIR}/tools/version.sh -v ${NUTTX_VERSION} -b "${BUILD_ID}" .version
