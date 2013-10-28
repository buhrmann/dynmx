#!/bin/bash
for folder in $1/*
do
  f="$folder/Evolvable.xml"
  fit=$(grep "SMCArm Fitness" "$f")
  fnm=$(basename $folder)
  echo "$fnm: $fit"
done
