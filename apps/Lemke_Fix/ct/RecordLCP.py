#!/usr/bin/env python2

import numpy as np

# import data
index = 4
pre_dir = 'ct'
cur_dir = str(index)
post_dir = 'z.txt'
dir = pre_dir+cur_dir+post_dir
data = np.genfromtxt(dir,dtype=str,delimiter=',')
row = data.shape
groupSize = 2 + index
numCt = row[0]/groupSize;
#  fd = np.array(filter(None,(data[1].strip()).split(' ')))
#  print np.sum(fd.astype(np.float64) > 1e-25)

# massage data

RecordZero = 1e-25
new_data = np.zeros(shape=(0,index))
for iter in xrange(numCt):
    value = [9]*index
    value = np.array(value)
    z_group = data[iter*groupSize:(iter+1)*groupSize]
    # decompose 
    fn = np.array(filter(None,(z_group[0].strip()).split(' ')))
    fn = fn.astype(np.float64)
    fd_batch = np.zeros(shape=(index,8))
    for idx in xrange(index):
        #  print z_group[1+idx]
        fd_batch[idx,:] = np.array([filter(None,(z_group[1+idx].strip()).split(' '))])
    #  print fd_batch
    fd_batch = fd_batch.astype(np.float64)
    mylambda=np.array(filter(None,(z_group[-1].strip()).split(' ')))
    mylambda = mylambda.astype(np.float64)
    for idx in xrange(index):
        #  print "@@@"
        #  print fn[idx]
        #  print fd_batch[idx]
        #  print mylambda[idx] 
        if fn[idx] < RecordZero:
            value[idx] = 9
        elif mylambda[idx] < RecordZero:
            value[idx] = 8
        else:
            fd = fd_batch[idx]
            nonZerofd = []
            for idxBasis in range(8):
                if fd[idxBasis] > RecordZero:
                    nonZerofd.append(idxBasis)
            #  print fd
            value[idx] = np.random.choice(nonZerofd)
    new_data = np.append(new_data,np.array([value]),axis=0)

# write data
new_dir = pre_dir+cur_dir+'_value_'+post_dir
np.savetxt(new_dir, new_data, delimiter = ',', fmt='%s')
