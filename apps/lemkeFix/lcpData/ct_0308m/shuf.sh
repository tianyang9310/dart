#!/bin/zsh

for ((i=1; i<9; i++)) 
do
  shuf lcp_data$i.csv > new_lcp_data$i.csv
done
