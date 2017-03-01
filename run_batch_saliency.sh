#!/usr/bin/env bash

PYTHON_PATH='/usr/local/bin/python'
SCRIPT_PATH='/Users/randallsmith/Documents/github/mlnet/main.py'
SCRIPT_PATH_DIR='/Users/randallsmith/Documents/github/mlnet'
DIRECTORIES=$(find . -type d -name png)
TEMP_DIR='tmp'
CWD=`pwd`
mkdir $CWD/$TEMP_DIR
mkdir $CWD/$TEMP_DIR/jpg

for DIR in ${DIRECTORIES[@]}; do
  echo $CWD/$DIR
  cd $CWD/$DIR
  mogrify -format jpg *.png
  mv *.jpg $CWD/$TEMP_DIR/jpg
  echo "TMP=$CWD/$TEMP_DIR/jpg"
  cd $SCRIPT_PATH_DIR
  $PYTHON_PATH $SCRIPT_PATH test $CWD/$TEMP_DIR/jpg/
  mkdir $CWD/$DIR/saliency
  mv *.jpg $CWD/$DIR/saliency
done

