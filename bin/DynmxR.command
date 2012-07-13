#!/bin/bash
# Script executes binary with input argument after copying to folder 

# Copy exe here, so one can keep coding and building the original exe
cd `dirname $0`
EXE="../xcode/build/Debug/cinder_dynmx.app/Contents/MacOS/cinder_dynmx"
cp $EXE .

# Execute from here with first input argument
CMD=./cinder_dynmx
#CFG="Configs/EvoArmOpLpInterSegMv1_A.xml"
CFG="/Users/thomasbuhrmann/Experiments/eSMCs/DerivativeOnly/12_06_25__16_47_51/SMCAgentEvo.xml"
$CMD $CFG
