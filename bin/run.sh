#!/bin/bash 

exe="../xcode4/build/Release/dynmx.app/Contents/MacOS/dynmx"

# Start first with new log file
i=0
echo Starting process $i
$exe "$1" > ./log.txt &

# Append log with subsequent ones
for ((i=1;i<"$2";i++)) 
do
  sleep 2s
  echo Starting process $i
  $exe "$1" >> ./log.txt &
done

less +F log.txt
