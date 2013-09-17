#!/bin/bash
# Executes specified program N times with first param, then N times with second etc..
# until all params have been processed

# Params
N=11
CFGs=("Configs/SMCArmEvo.xml" "Configs/SMCArmEvo.xml")

# Copy exe here, so one can keep coding and building the original exe
cd `dirname $0`
EXE="./../xcode4/build/Release/dynmx.app/Contents/MacOS/dynmx"

cp $EXE ./dynmx

# What to execute
CMD="./dynmx"

for i in ${CFGs[@]}
do
  for ((j=0; j<$N; j++))
  do 
    sleep 2s				# since output of above program is stored in folder /hh_mm_ss
    $CMD $i > /dev/null 2>&1 &		# run in background and suppress any output
    #$CMD $i  &
  done
  wait					# wait until all N processes have finished before starting the next N
done

osascript -e 'tell application "System Events" to sleep'