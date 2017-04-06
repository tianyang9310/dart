#!/bin/zsh

# DROPBOX_PATH =  ~/Dropbox
for ((i=1; i<=10; ++i)); do
  # rm ./numContactToLearn_${i}/*.caffemodel
  ln -s ~/Dropbox/CntctLrnin/CaffeNet/numContactToLearn_${i}/train_iter_100000.caffemodel numContactToLearn_${i}/train_iter_100000.caffemodel
done
