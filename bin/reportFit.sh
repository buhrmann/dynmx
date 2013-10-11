#!/bin/bash
for folder in $1/*
do
  f="$folder/Evolvable.xml"
  fit=$(grep "Genome Fitness" "$f")
  fnm=$(basename $folder)
  echo "$fnm: $fit"
done
