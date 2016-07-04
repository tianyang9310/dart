# Caffe Network Constructor
caffe_root = '/opt/caffe/'

import json
import sys
sys.path.insert(0, caffe_root+'python')
#print sys.path

import caffe
from caffe import layers as L, params as P

from caffe.proto.caffe_pb2 import TRAIN, TEST

def NNConstructor(dim_input, dim_output, dim_hidden, batch_size, phase, mPhi=0):
    '''
    dim_input  ---   x_dim
    dim_output ---   u_dim
    dim_hidden ---   None
    batch_size ---   ? what's the batch size? T of trajector or real batch of NN training
    phase      ---   TRAIN and TEST
    '''

    if phase == TRAIN:
        data_layer_info = json.dumps({
            'shape': [{'dim': (batch_size, dim_input)},
                      {'dim': (batch_size, dim_output)},
                      {'dim': (batch_size, dim_output, dim_output)}
                      ]})
        [net_input, action, precision] = L.Python(ntop=3, python_param=dict(module='NNBuilder', param_str=data_layer_info, layer='PolicyDataLayer'))
    elif phase == TEST:
        data_layer_info = json.dumps({
            'shape': [{'dim': (batch_size, dim_input)}
                      ]})
        net_input = L.Python(ntop=1, python_param=dict(module='NNBuilder', param_str=data_layer_info, layer='PolicyDataLayer'))
    elif phase == "ISLOSS":
        data_layer_info = json.dumps({
            'shape': [{'dim': (mPhi*batch_size, dim_input)},
                      {'dim': (mPhi*batch_size, dim_output)},
                      {'dim': (mPhi*batch_size, dim_output, dim_output)},
                      {'dim': (mPhi*batch_size, 1)},
                      {'dim': (1, 1)}
                      ]})
        [net_input, action, precision, Logq, wr] = L.Python(ntop=5, python_param=dict(module='NNBuilder', param_str=data_layer_info, layer='PolicyDataLayer'))
    else:
        raise Exception('Unknown Network Phase')

    cur_top = net_input
    cur_top = L.InnerProduct(cur_top, num_output=dim_hidden, weight_filler=dict(type='gaussian',std=0.01), bias_filler=dict(type='constant',value=0))
    cur_top = L.ReLU(cur_top, in_place=True)
    cur_top = L.InnerProduct(cur_top, num_output=dim_output, weight_filler=dict(type='gaussian',std=0.01), bias_filler=dict(type='constant',value=0))

    if phase == TRAIN:
        out = L.Python(cur_top, action, precision, loss_weight=1.0, python_param=dict(module='NNBuilder', layer='SumLogProbLoss'))
    elif phase == TEST:
        out = cur_top
    elif phase == "ISLOSS":
        PhiLoss_layer_info = json.dumps({
            'mPhi': mPhi,
            'batch_size': batch_size
            })
        out = L.Python(cur_top, action, precision, Logq, wr, loss_weight=1.0, python_param=dict(module='NNBuilder', param_str=PhiLoss_layer_info, layer='PhiLoss'))

    return out.to_proto()

class PolicyDataLayer(caffe.Layer):
    def setup(self, bottom, top):
        info = json.loads(self.param_str)
        for ind, top_blob in enumerate(info['shape']):
            top[ind].reshape(*top_blob['dim'])

    def reshape(self, bottom, top):
        pass

    def forward(self, bottom, top):
        pass

    def backward(self, top, propagate_down, bottom):
        pass


class SumLogProbLoss(caffe.Layer):
    def setup(self, bottom, top):
        pass

    def reshape(self, bottom, top):
        top[0].reshape(1)

    def forward(self, bottom, top):
        batch_size = bottom[0].data.shape[0]
        self.diff_ = bottom[0].data - bottom[1].data
        loss = 0.0
        for i in range(batch_size):
            loss += self.diff_[i].dot(bottom[2].data[i].dot(self.diff_[i]))
        top[0].data[...] = loss / 2.0 / batch_size

    def backward(self, top, propagate_down, bottom):
        batch_size = bottom[0].shape[0]
        for i in range(2):
            if propagate_down[i]:
                sign = 1 if i == 0 else -1
                alpha = sign * top[0].diff[0] / batch_size
                for j in range(batch_size):
                    bottom[i].diff[j] = bottom[2].data[j].dot(self.diff_[j])
                bottom[i].diff[...] *= alpha

class PhiLoss(caffe.Layer):
    def setup(self, bottom, top):
        info = json.loads(self.param_str)
        self.mPhi = info['mPhi']
        self.batch_size = info['batch_size']
        pass

    def reshape(self, bottom, top):
        top[0].reshape(1)

    def forward(self, bottom, top):
        # bottom[0] mPhi*batch_size  x
        # bottom[1] mPhi*batch_size  u
        # bottom[2] mPhi*batch_size  precision
        # bottom[3] mPhi*batch_size  Logq
        # bottom[4] mPhi*batch_size  wr
        pass

    def backward(self, top, propagate_down, bottom):
        pass
