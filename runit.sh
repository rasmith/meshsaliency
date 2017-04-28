#/usr/bin/env bash
EXE=$1
shift
dsymutil build/bin/$EXE
ASAN_SYMBOLIZER_PATH=/usr/local/Cellar/llvm/3.9.0/bin/llvm-symbolizer \
build/bin/$EXE $@
