#!/bin/bash

exe="../xcode4/build/Release/dynmx.app/Contents/MacOS/dynmx"

# Start first with new log file
$exe "$1" > ./log.txt &

# Append log with subsequent ones
for ((i=0;i<"$2"-1;i++)) 
do
  sleep 2s
  $exe "$1" >> ./log.txt &
done

less +F log.txt
