#!/bin/bash

# terminate if some error occurs
set -e

CONFIG=$1
PACKAGE=$2
PACKAGE_BASE=`basename $PACKAGE .dmg`
EMBREE_SIGN_FILE=$3

# create package
cmake --build . --config $CONFIG --target package

if [ $PACKAGE_BASE == $PACKAGE ]; then
    exit 0
fi

if [ -z "${EMBREE_SIGN_FILE}" ]; then
    exit 0
fi

hdiutil attach $PACKAGE
cp -r /Volumes/$PACKAGE_BASE/$PACKAGE_BASE.pkg $PACKAGE_BASE.pkg
hdiutil detach /Volumes/$PACKAGE_BASE
${EMBREE_SIGN_FILE} -c embree -vv $PACKAGE_BASE.pkg
rm $PACKAGE
hdiutil create -fs HFS+ -srcfolder "$PACKAGE_BASE.pkg" -volname "$PACKAGE_BASE" "$PACKAGE_BASE.dmg"
