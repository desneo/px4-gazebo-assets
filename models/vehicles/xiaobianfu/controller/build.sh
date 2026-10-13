#!/bin/sh
set -eu
D=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
OUT=${1:-/tmp/xiaobianfu-plugins}
mkdir -p "$OUT"
c++ -std=c++17 -shared -fPIC -O2 "$D/HandLaunch.cc" -o "$OUT/libHandLaunch.so" $(pkg-config --cflags --libs gz-sim8 gz-plugin2 gz-transport13)
