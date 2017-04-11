#!/usr/bin/env bash
if [ ! -e build ]
then
  mkdir build
fi
cd build
cmake ..
make -j8 $1 
