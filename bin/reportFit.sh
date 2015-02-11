#!/bin/bash
for folder in $1/*
do
  f="$folder/GA_BestGenome.xml"
  fit=$(grep "Genome Fitness" "$f")
  fnm=$(basename $folder)
  echo "$fnm: $fit"
done
