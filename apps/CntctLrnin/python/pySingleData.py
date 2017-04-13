#!/usr/bin/env python
# -*- coding: utf-8 -*-

# In order to use old data so write this parser

import numpy as np
from threading import Thread
from multiprocessing import Pool, Manager

pre_dir = './tmp'
post_dir = '.csv'
numBasis = 4

def ParseSingleData(numContactsToLearn):
    dir = pre_dir+post_dir
    data = np.genfromtxt(dir,dtype=str,delimiter=',')
    data = data.astype(np.float64)
    print data.shape
    # row, col = data.shape
    print dir
    # if row == 0:
    #     return
    ASize = numContactsToLearn * (numBasis + 1)
    inputSize = ASize + (ASize+1) * ASize /2 + numContactsToLearn
    outputSize = numContactsToLearn * (numBasis + 2)
    
    z = data[-outputSize:]
    print z
    np.savetxt('ct'+str(numContactsToLearn)+'z.txt',z,delimiter=',',fmt='%s')
    b = data[inputSize-ASize:-outputSize]
    print b
    b = np.append(b, np.zeros((numContactsToLearn,)),axis=0)
    np.savetxt('ct'+str(numContactsToLearn)+'b.txt',b,delimiter=',',fmt='%s')

    A = data[:inputSize-ASize]

    AMatrix = np.zeros(shape=(0,numContactsToLearn*(numBasis+2)))

    Aline = A
    AMatrix_tmp = np.zeros(shape=(numContactsToLearn*(numBasis+2),numContactsToLearn*(numBasis+2)))
    tmp = Aline[:-numContactsToLearn]
    mu = Aline[-numContactsToLearn:]
    for i in range(numContactsToLearn*(numBasis+1)):
        for j in range(i,numContactsToLearn*(numBasis+1)):
            AMatrix_tmp[i,j] = tmp[(numContactsToLearn*(numBasis+1)+numContactsToLearn*(numBasis+1)-i+1)*i/2 + j-i]

    AMatrix_tmp = AMatrix_tmp + np.triu(AMatrix_tmp,1).T
    mu = np.diag(mu)
    E = np.zeros(shape=(numContactsToLearn*numBasis,numContactsToLearn))
    for i in range(numContactsToLearn):
        E[i*numBasis:(i+1)*numBasis,i] = np.ones((numBasis,))*4
    AMatrix_tmp[numContactsToLearn*(numBasis+1):,:numContactsToLearn] = mu
    AMatrix_tmp[numContactsToLearn*(numBasis+1):,numContactsToLearn:numContactsToLearn+numContactsToLearn*(numBasis)] = -E.T
    AMatrix_tmp[numContactsToLearn:numContactsToLearn+numContactsToLearn*(numBasis),numContactsToLearn*(numBasis+1):] = E
    AMatrix = np.append(AMatrix, AMatrix_tmp, axis=0)

    np.savetxt('ct'+str(numContactsToLearn)+'A.txt',AMatrix,delimiter=',',fmt='%s')


if __name__=='__main__':
    ParseSingleData(8)
    #  p = Pool();
    #  [p.apply_async(myparse, args=(i,)) for i in range(1,13)]
    #  p.close()
    #  p.join()

    #  for i in range(2,13):
    #      t = Thread(target = myparse, args=(i,))
    #      t.start()
