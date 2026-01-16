#!/usr/bin/bash
set -ex

mkdir -p build
cd build
cmake ../
make -j10

