#!/bin/bash
# Executes binary with specified argument N times

N=1
#CFG="Configs/EvoArmIbIsDirs4M.xml"
CFG="Configs/SMCArmEvo.xml"

# Copy exe here, so one can keep coding and building the original exe
cd `dirname $0`
EXE="./../xcode4/build/Release/dynmx.app/Contents/MacOS/dynmx"

cp $EXE ./dynmx

# What to execute
CMD="./dynmx $CFG"
CD="cd `dirname $0`"

# Start with first then open terminal tabs for rest
for ((i=0; i<$N; i++))
do 
  sleep 2s

  # open new tab
  osascript 2>/dev/null << EOF
    tell application "System Events"
      tell process "Terminal" to keystroke "t" using command down
    end tell

    tell application "Terminal"
      activate
	  do script with command "$CD" in window 1
      do script with command "$CMD" in window 1 
    end tell
EOF

done

wait