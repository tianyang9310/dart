#!/usr/bin/env python2

import numpy as np

# import data
index = 2
pre_dir = '/Users/Yang/Material/Research/dart/build/data/lcp_data'
cur_dir = str(index)
post_dir = '.csv'
dir = pre_dir+cur_dir+post_dir
data = np.genfromtxt(dir,dtype=str,delimiter=',')
row, col = data.shape

# massage data
max_num = 1000
size = [10]*index
gridBin = np.zeros(tuple(size))

new_data = np.zeros(shape=(0,col))
for iter in data:
    label = iter[-index:]
    coord = label.astype(np.int)
    if (gridBin[tuple(coord)] <= max_num):
        new_data = np.append(new_data,np.array([iter]),axis=0)
        gridBin[tuple(coord)] +=1
    else:
        continue

for iter in data:
    label = iter[-index:]
    coord = label.astype(np.int)
    if (gridBin[tuple(coord)] <= max_num):
        repeatTimes = np.random.randint(5)
        for repeat in xrange(repeatTimes):
            new_data = np.append(new_data,np.array([iter]),axis=0)
            gridBin[tuple(coord)] +=1
    else:
        continue

# write data
new_dir = pre_dir+cur_dir+'_trim_'+str(max_num)+post_dir
np.savetxt(new_dir, new_data, delimiter = ',', fmt='%s')
