#!/bin/bash
# Required env variables:
# SLC: Path to SLC-CLI
# SDK: Path to SDK root
# COMMANDER: Path to Simplicity Commander binary

if [ -z "${SLC}" ]; then
	echo "ERROR: env variable SLC must be set to SLC-CLI binary"
	exit 1
fi

if [ -z "${SDK}" ]; then
	echo "ERROR: env variable SDK must be set to SDK root"
	exit 1
fi

if [ -z "${COMMANDER}" ]; then
	echo "ERROR: env variable COMMANDER must be set to Simplicity Commander binary"
	exit 1
fi

POST_BUILD_EXE=tools/mkgbl.sh
PROJ_NAME=nc_controller_ncp

rm -rf build/
rm -f *.Makefile
rm -f *.mak

$SLC signature trust --sdk $SDK

# Find the toolchain if not set
if [ -z "${TOOLCHAIN}" ]; then
	TOOLCHAIN=$(find /opt -maxdepth 1 -type d -name "*arm-none-eabi*" | head -n 1)
	echo "Found toolchain: $TOOLCHAIN"
fi

$SLC generate \
	--project-file $PROJ_NAME.slcp \
	--export-destination build/ \
	--sdk "$SDK" \
	--no-copy \
	--toolchain toolchain_gcc \
	--output-type makefile

node tools/patch_makefile.mjs

cp build/*.Makefile ./
cp build/*.mak ./
# Fix the include path to point to the current directory
sed -i 's/-I\.\. /-I\. /g' *.mak

make release -B -f $PROJ_NAME.Makefile POST_BUILD_EXE=$POST_BUILD_EXE ARM_GCC_DIR=$TOOLCHAIN