#!/bin/bash

make ARCH=mips CROSS_COMPILE=mipsel-linux- $1 -j4
