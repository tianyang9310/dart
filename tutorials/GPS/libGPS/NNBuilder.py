# Caffe Network Constructor
caffe_root = '/opt/caffe/'

import json
import numpy as np
from scipy.stats import norm
import math
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
                      {'dim': (mPhi*batch_size, dim_input)},
                      {'dim': (mPhi*batch_size, dim_output)},
                      {'dim': (mPhi*batch_size, dim_output, dim_output)},
                      {'dim': (mPhi*batch_size, 1)},
                      {'dim': (1, 1)}
                      ]})
        [net_input, net_input2, action, precision, Logq, wr] = L.Python(ntop=6, python_param=dict(module='NNBuilder', param_str=data_layer_info, layer='PolicyDataLayer'))
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
        out = L.Python(cur_top, net_input2, action, precision, Logq, wr, loss_weight=1.0, python_param=dict(module='NNBuilder', param_str=PhiLoss_layer_info, layer='PhiLoss'))

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

    def reshape(self, bottom, top):
        top[0].reshape(1)

    def forward(self, bottom, top):
        # bottom[0] mPhi*batch_size*dim_output  u given x
        # bottom[1] mPhi*batch_size*dim_input  x
        # bottom[2] mPhi*batch_size*dim_output  u of sample
        # bottom[3] 1*dim_output*dim_output  precision
        # bottom[4] mPhi*batch_size*1  Logq
        # bottom[5] 1*1  wr

        loss = 0.0
        Log_Pi_theta = np.zeros(self.mPhi)
        self.Log_Pi_theta_List = np.zeros((self.batch_size, self.mPhi))
        self.J_tilt_List = np.zeros(self.batch_size)

        for t_idx in range(self.batch_size): # t=0~T-2
            inner_sum = 0.0
            Zt        = 0.0
            for m_idx in range(self.mPhi):
                cur_idx = m_idx*self.batch_size + t_idx
                Log_Pi_theta[m_idx] = Log_Pi_theta[m_idx] + np.log(norm(bottom[0].data[cur_idx][0], np.linalg.inv(bottom[3].data[cur_idx])[0,0]).pdf(bottom[2].data[cur_idx][0]))

                # h=Log_Pi_theta[m_idx]       # float number
                # a=bottom[0].data[cur_idx]   # ndarray 1d
                # b=bottom[1].data[cur_idx]   # ndarray 1d
                # c=bottom[2].data[cur_idx]   # ndarray 1d
                # d=bottom[3].data[cur_idx]   # ndarray 2d
                # e=bottom[4].data[cur_idx]   # ndarray 1d
                # f=bottom[5].data[0]   # ndarray 1d
                # g=self.StepCost(bottom[1].data[cur_idx], bottom[2].data[cur_idx]) # float number
                # error = Log_Pi_theta[m_idx] - bottom[4].data[cur_idx][0]

                inner_sum = inner_sum + np.exp(Log_Pi_theta[m_idx] - bottom[4].data[cur_idx][0])* self.StepCost(bottom[1].data[cur_idx], bottom[2].data[cur_idx])
                Zt        = Zt + np.exp(Log_Pi_theta[m_idx] - bottom[4].data[cur_idx][0])
                
            self.Log_Pi_theta_List[t_idx] = Log_Pi_theta

            # TODO remove regularizer
            tmpLoss = loss
            loss = loss + float(1)/Zt*inner_sum # + bottom[5].data[0][0]*np.log(Zt)
            if tmpLoss>loss:
                raise Exception('Loss is not increasing')

            self.J_tilt_List[t_idx] = float(1)/Zt*inner_sum
        top[0].data[...] = loss

    def backward(self, top, propagate_down, bottom):
        # bottom[0] mPhi*batch_size*dim_output u given x
        # bottom[1] mPhi*batch_size*dim_input  x
        # bottom[2] mPhi*batch_size*dim_output  u of sample
        # bottom[3] 1*dim_output*dim_output  precision
        # bottom[4] mPhi*batch_size*1  Logq
        # bottom[5] 1*1  wr

        for t_idx in range(self.batch_size):
            for m_idx in range(self.mPhi):
                cur_idx = m_idx*self.batch_size + t_idx
                gradient = (bottom[2].data[cur_idx] - bottom[0].data[cur_idx]).dot(bottom[3].data[cur_idx])
                inner_sum_t_p = 0.0
                for t_p_idx in range(t_idx, self.batch_size):
                    cur_p_idx = np.arange(self.mPhi)*self.batch_size + t_p_idx
                    # compute zt_p  xi_t_p

                    zt_p = np.sum(np.exp((self.Log_Pi_theta_List[t_p_idx]-np.reshape(bottom[4].data[cur_p_idx],self.mPhi))))
                    J_tilt = self.J_tilt_List[t_p_idx]

                    # TODO remove regularizer
                    xi_t_p = self.StepCost(bottom[1].data[m_idx*self.batch_size+t_p_idx], bottom[2].data[m_idx*self.batch_size+t_p_idx]) - J_tilt # + bottom[5].data[0]

                    
                    inner_sum_t_p = inner_sum_t_p + float(1)/zt_p*np.exp(self.Log_Pi_theta_List[t_p_idx,m_idx]-bottom[4].data[m_idx*self.batch_size+t_p_idx])*xi_t_p
                gradient = gradient * inner_sum_t_p
                bottom[0].diff[cur_idx] = gradient 

    def StepCost(self,_x,_u):
        '''
        _x: dim_intput ndarray
        _u: dim_output ndarray
        return: float number (not a ndarray)
        '''
        xd = np.array([0, math.pi, 0, 0])
        Q  = np.array([[0.01,0,0,0],
                       [0   ,5,0,0],
                       [0   ,0,0,0],
                       [0   ,0,0,0]])
        R  = np.array([1])
        # Matrix4d Qf = Matrix4d::Identity()
        # Qf(1,1)				= 500
        Q = Q*0.001
        R = R*0.001
        return (0.5*(_x-xd).dot(Q.dot(_x-xd)) + 0.5*_u.dot(R.dot(_u)))[0]
