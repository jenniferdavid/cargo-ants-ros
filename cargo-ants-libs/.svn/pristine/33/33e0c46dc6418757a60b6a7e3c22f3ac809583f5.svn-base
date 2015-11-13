#!/bin/bash

ENVROOT="/users/fred/robox-linux/build/ppc_buildroot_v012"
SRCROOT=".."

$ENVROOT/env-ppc-linux \
  $SRCROOT/configure \
  --prefix=/robox-linux/ \
  --with-boost=/robox-linux \
  --with-robox=/robox-linux \
  --build=i386-linux \
  --host=powerpc-linux
