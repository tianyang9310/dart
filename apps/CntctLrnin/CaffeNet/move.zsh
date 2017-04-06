#!/bin/zsh

for ((i=2; i<=10; ++i)); do
  mv ./download-caffe-model/numContactToLearn_${i}_$[i-1]/* ./numContactToLearn_$i/
done
