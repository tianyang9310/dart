#!/usr/bin/env python
# -*- coding: utf-8 -*-

from multiprocessing import Pool, Manager
import multiprocessing
import numpy as np
import h5py
import time
import os

# import data
numContactsToLearn = 3
numProcess = multiprocessing.cpu_count()
numClass = 10
numMax = 500

pre_dir = '../../../build/data/lcp_data'
cur_dir = str(numContactsToLearn)
post_dir = '.csv'
dir = pre_dir + cur_dir + post_dir

data = np.genfromtxt(dir, dtype=str, delimiter=',')
row, col = data.shape

numRow_PerProcess = row / numProcess
data_Pool = []
newData_Pool = []
for idxProcess in xrange(numProcess):
    data_Pool.append(data[idxProcess * numRow_PerProcess:(idxProcess + 1) * numRow_PerProcess, :])
    newData_Pool.append(np.zeros(shape=(0, col)))
print "Data has been separated {} parts...".format(numProcess)


# print data_Pool

def index_from_XD_to_1D(*indices):
    # all indices should be [0, numClass]
    res = 0
    indices = list(indices)
    indices.reverse()
    for idx, item in enumerate(indices):
        res += item * numClass ** idx
    return res


# Keep at most maximum data
def recordNewDataBelowMax(idxProcess, Bin):
    print 'Run recording task %s (%s)...' % (idxProcess, os.getpid())
    newSubData = np.zeros(shape=(0, col))
    for iter in data_Pool[idxProcess]:
        label = iter[-numContactsToLearn:]
        coord = tuple(label.astype(np.int))
        coord = index_from_XD_to_1D(*coord)
        if (Bin[coord] <= numMax):
            newSubData = np.append(newSubData, np.array([iter]), axis=0)
            Bin[coord] += 1
        else:
            continue
    print 'Finish recording task %s (%s)...' % (idxProcess, os.getpid())
    return newSubData


def repeatNewDataReachMax(idxProcess, Bin):
    print 'Run repeating task %s (%s)...' % (idxProcess, os.getpid())
    newSubData = np.zeros(shape=(0, col))
    for iter in data_Pool[idxProcess]:
        label = iter[-numContactsToLearn:]
        coord = tuple(label.astype(np.int))
        coord = index_from_XD_to_1D(*coord)
        if (Bin[coord] <= numMax):
            repeatTimes = np.random.randint(5)
            for repeat in xrange(repeatTimes):
                newSubData = np.append(newSubData, np.array([iter]), axis=0)
                Bin[coord] += 1
        else:
            continue
    print 'Finish repeating task %s (%s)...' % (idxProcess, os.getpid())
    return newSubData


if __name__ == '__main__':
    #  Multiprocessing.Pool可以提供指定数量的进程供用户调用，当有新的请求提交到pool中时，
    #  如果池还没有满，那么就会创建一个新的进程用来执行该请求；但如果池中的进程数已经达
    #  到规定最大值，那么该请求就会等待，直到池中有进程结束，才会创建新的进程来执行它。
    #  在共享资源时，只能使用Multiprocessing.Manager类，而不能使用Queue或者Array。
    print 'Parent process %s.' % os.getpid()
    p1 = Pool()
    manager = Manager()
    gridBin = manager.Array('i', [0] * (numClass) ** numContactsToLearn)
    newData_Pool = [p1.apply_async(recordNewDataBelowMax, args=(idxProcess, gridBin)) for idxProcess in
                    range(numProcess)]

    print 'Waiting for all recording subprocesses done...'
    try:
        print "Waiting 5 seconds"
        time.sleep(5)
    except KeyboardInterrupt:
        print "Caught KeyboardInterrupt, terminating workers"
        p1.terminate()
        p1.join()
    else:
        print "No KeyboardInterrupt, continues until quitting normally"
        p1.close()
        p1.join()

    print 'All recording subprocesses done.'

    new_data = np.zeros(shape=(0, col))
    for res in newData_Pool:
        new_data = np.append(new_data, res.get(timeout=1), axis=0)

    p2 = Pool()
    newData_Pool = [p2.apply_async(repeatNewDataReachMax, args=(idxProcess, gridBin)) for idxProcess in
                    range(numProcess)]

    print 'Waiting for all repeating subprocesses done...'
    try:
        print "Waiting 5 seconds"
        time.sleep(5)
    except KeyboardInterrupt:
        print "Caught KeyboardInterrupt, terminating workers"
        p2.terminate()
        p2.join()
    else:
        print "No KeyboardInterrupt, continues until quitting normally"
        p2.close()
        p2.join()
    print 'All repeating subprocesses done.'
    for res in newData_Pool:
         new_data = np.append(new_data, res.get(timeout=1), axis=0)

    new_dir = pre_dir + cur_dir + '_trim_' + str(numMax) + '.h5'
    h5file = h5py.File(new_dir,'w')
    h5file.create_dataset('ct data',data=new_data)
    h5file.close()
    # np.savetxt(new_dir, new_data, delimiter=',', fmt='%s')
