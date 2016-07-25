# Caffe Network Constructor
caffe_root = '/opt/caffe/'

import json
import warnings
import numpy as np
from scipy.misc import logsumexp
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
                      {'dim': (1, dim_output, dim_output)}
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
                      {'dim': (1, dim_output, dim_output)},
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
        info = json.loads(self.param_str)
        for ind, top_blob in enumerate(info['shape']):
            top[ind].reshape(*top_blob['dim'])

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
        # variance is arbitrarily set as 0.1, therefore it is not important here
        batch_size = bottom[0].data.shape[0]
        self.diff_ = bottom[0].data - bottom[1].data
        loss = 0.0
        for i in range(batch_size):
            loss += self.diff_[i].dot(bottom[2].data[0].dot(self.diff_[i]))
        top[0].data[...] = loss / 2.0 / batch_size

    def backward(self, top, propagate_down, bottom):
        # variance is arbitrarily set as 0.1, therefore it is not important here
        batch_size = bottom[0].shape[0]
        for i in range(2):
            if propagate_down[i]:
                sign = 1 if i == 0 else -1
                alpha = sign * top[0].diff[0] / batch_size
                for j in range(batch_size):
                    bottom[i].diff[j] = bottom[2].data[0].dot(self.diff_[j])
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
        self.Log_Zt_List = np.zeros(self.batch_size)

        for t_idx in range(self.batch_size): # t=0~T-2
            cur_idx = np.arange(self.mPhi)*self.batch_size + t_idx

            with warnings.catch_warnings():
                warnings.filterwarnings('error')
                try:
                    Log_Pi_theta = Log_Pi_theta + np.reshape(np.log(self.GaussianEvaluator_m_batch(bottom[0].data[cur_idx], np.linalg.inv(bottom[3].data[0])[0,0], bottom[2].data[cur_idx])),self.mPhi)
                except Warning as e:
                    print 'Probability is approaching 0'

            inner_sum_ind = np.exp(Log_Pi_theta - bottom[4].data[cur_idx].reshape(self.mPhi)) *self.StepCost_m_batch(bottom[1].data[cur_idx], bottom[2].data[cur_idx])
            Log_Zt_ind = Log_Pi_theta - bottom[4].data[cur_idx].reshape(self.mPhi)

            inner_sum = np.sum(inner_sum_ind)
            Log_Zt        = logsumexp(Log_Zt_ind)

            # debugging
            # print np.exp(Log_Zt)
            # print '                         ',inner_sum

            self.Log_Pi_theta_List[t_idx] = Log_Pi_theta
            self.Log_Zt_List[t_idx] = Log_Zt

            # tmpLoss = loss
            loss = loss + np.exp(-Log_Zt)*inner_sum  + bottom[5].data[0][0]*Log_Zt
            # if tmpLoss>loss:
            #     raise Exception('Loss is not increasing')

            self.J_tilt_List[t_idx] = np.exp(-Log_Zt)*inner_sum

        loss_wo = np.sum(self.J_tilt_List)
        # loss is minimized in optimization stage. Therefore the objective should be negative of above function
        loss_wo = -loss_wo
        loss    = -loss

        self.lossvalue_wo = loss_wo
        top[0].data[...] = loss

    def backward(self, top, propagate_down, bottom):
        # bottom[0] mPhi*batch_size*dim_output u given x
        # bottom[1] mPhi*batch_size*dim_input  x
        # bottom[2] mPhi*batch_size*dim_output  u of sample
        # bottom[3] 1*dim_output*dim_output  precision
        # bottom[4] mPhi*batch_size*1  Logq
        # bottom[5] 1*1  wr

        bottom[0].diff[...] = 0
        for t_idx in range(self.batch_size):
            for m_idx in range(self.mPhi):
                cur_idx = m_idx*self.batch_size + t_idx
                gradient = (bottom[2].data[cur_idx] - bottom[0].data[cur_idx]).dot(bottom[3].data[0])
                inner_sum_t_p_ind = np.zeros(self.batch_size-t_idx)
                for t_p_idx in range(t_idx, self.batch_size):
                    # zt_p_log = logsumexp(self.Log_Pi_theta_List[t_p_idx]-np.reshape(bottom[4].data[cur_p_idx],self.mPhi))
                    J_tilt = self.J_tilt_List[t_p_idx]

                    xi_t_p = self.StepCost(bottom[1].data[m_idx*self.batch_size+t_p_idx], bottom[2].data[m_idx*self.batch_size+t_p_idx]) - J_tilt  + bottom[5].data[0][0]

                    inner_sum_t_p_ind[t_p_idx-t_idx]=np.exp(-self.Log_Zt_List[t_p_idx]+self.Log_Pi_theta_List[t_p_idx,m_idx]-bottom[4].data[m_idx*self.batch_size+t_p_idx])* xi_t_p
                inner_sum_t_p = np.sum(inner_sum_t_p_ind)
                gradient = gradient * inner_sum_t_p

                # loss is minimized in optimization stage. Therefore the objective should be negative of above function
                gradient = -gradient

                bottom[0].diff[cur_idx] = gradient 

        # debugging
        b = bottom[0].diff

        bottom[1].diff[...] = 0
        bottom[2].diff[...] = 0
        bottom[3].diff[...] = 0
        bottom[4].diff[...] = 0
        bottom[5].diff[...] = 0

    def StepCost(self,_x,_u):
        '''
        _x: dim_intput ndarray
        _u: dim_output ndarray
        return: float number (not a ndarray)
        '''
        xd = np.array([0, math.pi, 0, 0])
        Q  = np.array([[0.001,0,0,0],
                       [0   ,1,0,0],
                       [0   ,0,0,0],
                       [0   ,0,0,0]])
        R  = np.array([1])
        # Matrix4d Qf = Matrix4d::Identity()
        # Qf(1,1)				= 500
        Q = Q
        R = R*0.001
        return -((0.5*(_x-xd).dot(Q.dot(_x-xd)) + 0.5*_u.dot(R.dot(_u)))[0])

    def StepCost_m_batch(self,_x,_u):
        '''
        _x: dim_intput ndarray
        _u: dim_output ndarray
        return: float number (not a ndarray)
        '''
        xd = np.array([0, math.pi, 0, 0])
        Q  = np.array([[0.001,0,0,0],
                       [0   ,1,0,0],
                       [0   ,0,0,0],
                       [0   ,0,0,0]])
        R  = np.array([[1]])
        # Matrix4d Qf = Matrix4d::Identity()
        # Qf(1,1)				= 500
        Q = Q
        R = R*0.001
        return -(0.5*np.diag((_x-xd).dot(Q.dot((_x-xd).T))) + 0.5*np.diag(_u.dot(R.dot(_u.T))))

    def GaussianEvaluator(self, mean, covariance, testX):
        probability = 0.0
        probability = 1.0/np.sqrt(2*np.pi*covariance)*np.exp(-0.5*((mean-testX)**2)/covariance)
        if probability < np.finfo(np.double).tiny:  
            probability = np.finfo(np.double).tiny  
        return probability

    def GaussianEvaluator_m_batch(self, mean, covariance, testX):
        probability = np.zeros(len(mean))
        probability = 1.0/np.sqrt(2*np.pi*covariance)*np.exp(-0.5*((mean-testX)**2)/covariance)
        probability = np.maximum(probability, np.finfo(np.double).tiny)
        print probability
        return probability
