#!/bin/bash

# Start first with new log file
./dynmx "$1" > log.txt &

# Append log with subsequent ones
for ((i=0;i<$2;i++)) 
  do sleep 2s
  ./dynmx Configs/SMCArmEvo.xml >> log.txt
done