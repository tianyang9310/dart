import os
import caffe
from caffe import layers as L
from caffe import params as P
from caffe.proto.caffe_pb2 import TRAIN, TEST

numContactsToLearn = 5
mBatchSize = 32
mHiddenUnit = 4096
numUniqueLabel = 10
numBasis = 4
ASize = numContactsToLearn * (numBasis + 1)
inputSize = ASize + (ASize + 1) * ASize / 2 + numContactsToLearn
outputSize = 1

def nonlinear_net(hdf5, batch_size, phase):
    n = caffe.NetSpec()
    if phase == TRAIN:
        n.data, n.label0 = L.HDF5Data(batch_size=batch_size, source=hdf5, ntop=2) #, n.label1, n.label2, n.label3, n.label4, n.label5, n.label6, n.label7 
    else:
        n.data = L.Input(shape=dict(dim=[1, inputSize]))

    n.ip1 = L.InnerProduct(n.data, num_output=mHiddenUnit, weight_filler=dict(type='xavier'))
    n.relu1 = L.ReLU(n.ip1, in_place=True)
    
#     n.dropout1 = L.Dropout(n.relu1)

    n.ip3 = L.InnerProduct(n.relu1, num_output=mHiddenUnit, weight_filler=dict(type='xavier'))
    n.relu3 = L.ReLU(n.ip3, in_place=True)
    
    
    n.ip4 = L.InnerProduct(n.relu3, num_output=mHiddenUnit, weight_filler=dict(type='xavier'))
    n.relu4 = L.ReLU(n.ip4, in_place=True)
    
    n.ip2 = L.InnerProduct(n.relu4, num_output=mHiddenUnit, weight_filler=dict(type='xavier'))
    n.relu2 = L.ReLU(n.ip2, in_place=True)
    
    n.final0 = L.InnerProduct(n.relu2, num_output=numUniqueLabel, weight_filler=dict(type='xavier'))
#     n.final1 = L.InnerProduct(n.ip2, num_output=numUniqueLabel, weight_filler=dict(type='xavier'))
#     n.final2 = L.InnerProduct(n.ip2, num_output=numUniqueLabel, weight_filler=dict(type='xavier'))
#     n.final3 = L.InnerProduct(n.ip2, num_output=numUniqueLabel, weight_filler=dict(type='xavier'))
#     n.final4 = L.InnerProduct(n.ip2, num_output=numUniqueLabel, weight_filler=dict(type='xavier'))
#     n.final5 = L.InnerProduct(n.ip2, num_output=numUniqueLabel, weight_filler=dict(type='xavier'))
#     n.final6 = L.InnerProduct(n.ip2, num_output=numUniqueLabel, weight_filler=dict(type='xavier'))
#     n.final7 = L.InnerProduct(n.ip2, num_output=numUniqueLabel, weight_filler=dict(type='xavier'))

    '''
    n.final2 = L.InnerProduct(n.ip2, num_output=10, weight_filler=dict(type='xavier'))
    n.final3 = L.InnerProduct(n.ip2, num_output=10, weight_filler=dict(type='xavier'))
    '''

#     n.accuracy1 = L.Accuracy(n.final1, n.label1)
#     n.accuracy2 = L.Accuracy(n.final2, n.label2)
#     n.accuracy3 = L.Accuracy(n.final3, n.label3)
#     n.accuracy4 = L.Accuracy(n.final4, n.label4)
#     n.accuracy5 = L.Accuracy(n.final5, n.label5)
#     n.accuracy6 = L.Accuracy(n.final6, n.label6)
#     n.accuracy7 = L.Accuracy(n.final7, n.label7)

    '''

    n.accuracy3 = L.Accuracy(n.final3, n.label3)
    '''
    if phase == TRAIN:
        n.accuracy0 = L.Accuracy(n.final0, n.label0)
        n.loss0 = L.SoftmaxWithLoss(n.final0, n.label0)
    else:
        n.predict = L.ArgMax(n.final0)
#     n.loss1 = L.SoftmaxWithLoss(n.final1, n.label1)
#     n.loss2 = L.SoftmaxWithLoss(n.final2, n.label2)
#     n.loss3 = L.SoftmaxWithLoss(n.final3, n.label3)
#     n.loss4 = L.SoftmaxWithLoss(n.final4, n.label4)
#     n.loss5 = L.SoftmaxWithLoss(n.final5, n.label5)
#     n.loss6 = L.SoftmaxWithLoss(n.final6, n.label6)
#     n.loss7 = L.SoftmaxWithLoss(n.final7, n.label7)

    return n.to_proto()

if __name__ == "__main__":
    dirname = os.path.abspath('./numContactToLearn_{}'.format(numContactsToLearn))
    if not os.path.exists(dirname):
        os.makedirs(dirname)

    train_net_path = os.path.join(dirname, 'train.prototxt') 
    with open(train_net_path, 'w') as f:
        f.write(str(nonlinear_net(os.path.join(dirname, 'train.txt'), mBatchSize, TRAIN)))

    test_net_path = os.path.join(dirname, 'test.prototxt')
    with open(test_net_path, 'w') as f:
        f.write(str(nonlinear_net(os.path.join(dirname, 'test.txt'), mBatchSize, TRAIN)))
        
    deploy_net_path = os.path.join(dirname, 'deploy.prototxt')
    with open(deploy_net_path, 'w') as f:
        f.write(str(nonlinear_net(os.path.join(dirname, 'deploy.txt'), mBatchSize, TEST)))
