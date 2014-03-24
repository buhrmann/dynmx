#!/bin/bash
outfnm="fit.txt"

# Clear file
echo -n > $outfnm

for folder in $1/*
do
  f="$folder/Evolvable.xml"
  fit=$(ack -o '(?<=Genome Fitness=\")[0-9]+.[0-9]+' "$f")
  echo $fit >> $outfnm
  #echo "$fit"
  #fnm=$(basename $folder)
  #echo "$fnm: $fit"
done

result=$(Rscript -e "x = read.csv('fit.txt', header = F); fmax=max(x[,1]); fmu=mean(x[,1]); fsd=sd(x[,1]); cat(sprintf('max: %2.3f | mean: %2.3f | sd: %2.3f',fmax,fmu,fsd));") 
echo "$result"
