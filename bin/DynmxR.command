#!/bin/bash
# Script executes binary with input argument after copying to folder 

# Copy exe here, so one can keep coding and building the original exe
cd `dirname $0`
EXE="../xcode/build/Release/cinder_dynmx.app/Contents/MacOS/cinder_dynmx"
cp $EXE .

# Execute from here with first input argument
CMD=./cinder_dynmx
$CMD 
