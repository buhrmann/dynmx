#!/bin/bash
# Executes specified program N times with first command line param, then N times with second etc..
# until all command line params have been processed

N=4
CFGs=("Configs/EvoArmInteractHillInterSegMv1.xml" "Configs/EvoArmInteractHillInterSegMv2.xml")

# Original executable
# Copy exe here, so one can keep coding and building the original exe
cd `dirname $0`
EXE="../xcode/build/Release/cinder_dynmx.app/Contents/MacOS/cinder_dynmx"
cp $EXE .

# What to execute
CMD="./cinder_dynmx"

for i in ${CFGs[@]}
do
  for ((j=0; j<$N; j++))
  do 
    sleep 1s						# since output of above program is stored in folder /hh_mm_ss
    $CMD $i > /dev/null 2>&1 &		# run in background and suppress any output
    #$CMD $i  &
  done
  wait								# wait until all N processes have finished before starting the next N
done

wait