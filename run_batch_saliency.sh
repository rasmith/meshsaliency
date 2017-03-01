#!/usr/bin/env bash

# Usage:  
# ./run_batch_saliency.sh <input_directory>
# input_directory - directory output from run_batch_views.py
#
# This will run saliency on all of the png files output to the tree
# contained at <input_directory>.

# This script uses:
#    https://github.com/marcellacornia/mlnet
#
#  A Deep Multi-Level Network for Saliency Prediction,
#  Marcella Cornia, Lorenzo Baraldi, Giuseppe Serra, Rita Cucchiara,
#  ICPR 2016
#
# The following variables should be set
#
# PYTHON_PATH - the path to your Python executable
# SCRIPT_PATH_DIR  - path to the mlnet script directory, as detailed above,
#
# the weights file 'vgg16_weights.h5' should be in <SCRIPT_PATH_DIR>.
#

PYTHON_PATH="/usr/local/bin/python"
SCRIPT_PATH_DIR="/Users/randallsmith/Documents/github/mlnet"
SCRIPT_PATH="${SCRIPT_PATH_DIR}/main.py"

INPUT_DIRECTORY=$1

DIRECTORIES=$(find $INPUT_DIRECTORY -type d -name png)
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
  mkdir $CWD/$DIR/../saliency
  mv *.jpg $CWD/$DIR/../saliency
done
