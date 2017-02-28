#!/usr/bin/env python2

import numpy as np
from threading import Thread
from multiprocessing import Pool, Manager

pre_dir = './lcp_data'
post_dir = '.csv'

def myparse(numContactsToLearn):
    dir = pre_dir+str(numContactsToLearn)+post_dir
    data = np.genfromtxt(dir,dtype=str,delimiter=',')
    data = data.astype(np.float64)
    print data.shape
    row, col = data.shape
    print dir
    if row == 0:
        return
    ASize = numContactsToLearn * 9
    inputSize = ASize + (ASize+1) * ASize /2 + numContactsToLearn
    outputSize = numContactsToLearn
    
    z = data[:,-outputSize:]
    np.savetxt('ct'+str(numContactsToLearn)+'z.txt',z,delimiter=',',fmt='%s')

    b = data[:,inputSize-ASize:-outputSize]
    b = np.append(b, np.zeros((row,numContactsToLearn)),axis=1)
    np.savetxt('ct'+str(numContactsToLearn)+'b.txt',b,delimiter=',',fmt='%s')

    A = data[:,:inputSize-ASize]

    AMatrix = np.zeros(shape=(0,numContactsToLearn*10))
    for idx in xrange(row):
        Aline = A[idx,:]
        AMatrix_tmp = np.zeros(shape=(numContactsToLearn*10,numContactsToLearn*10))
        tmp = Aline[:-numContactsToLearn]
        mu = Aline[-numContactsToLearn:]
        for i in range(numContactsToLearn*9):
            for j in range(i,numContactsToLearn*9):
                AMatrix_tmp[i,j] = tmp[(numContactsToLearn*9+numContactsToLearn*9-i+1)*i/2 + j-i]

        AMatrix_tmp = AMatrix_tmp + np.triu(AMatrix_tmp,1).T
        mu = np.diag(mu)
        E = np.zeros(shape=(numContactsToLearn*9,numContactsToLearn))
        for i in range(numContactsToLearn):
            E[i*8:(i+1)*8,i] = np.ones((8,))*4
        AMatrix_tmp[numContactsToLearn*9:,:numContactsToLearn] = mu
        AMatrix_tmp[numContactsToLearn*9:,numContactsToLearn:numContactsToLearn+numContactsToLearn*9] = -E.T
        AMatrix_tmp[numContactsToLearn:numContactsToLearn+numContactsToLearn*9,numContactsToLearn*9:] = E
        AMatrix = np.append(AMatrix, AMatrix_tmp, axis=0)

    np.savetxt('ct'+str(numContactsToLearn)+'A.txt',AMatrix,delimiter=',',fmt='%s')


if __name__=='__main__':
    myparse(2)
    #  p = Pool();
    #  [p.apply_async(myparse, args=(i,)) for i in range(1,13)]
    #  p.close()
    #  p.join()

    #  for i in range(2,13):
    #      t = Thread(target = myparse, args=(i,))
    #      t.start()
