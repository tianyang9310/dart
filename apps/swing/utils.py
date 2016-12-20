import sys
import os
# os.environ['GLOG_minloglevel']='2'
# ------------------------------------------------------------------------------
import caffe
from caffe.proto.caffe_pb2 import SolverParameter, TRAIN, TEST
from caffe import layers as L
# ------------------------------------------------------------------------------
import json
import numpy as np
from google.protobuf.text_format import MessageToString

BATCH_SIZE = 32
DIM_INPUT = 32
DIM_ACTION = 32
DIM_PAIR = 3 # num of pair
DIM_HIDDEN = [256, 128]

def CaffeNet(batch_size, dim_input, dim_pair, dim_action, dim_hidden, phase):
    n = caffe.NetSpec()
    
    # data layer
    if phase == TRAIN:
        data_layer_info = json.dumps({
            'shape': [{'dim': (batch_size, dim_input)},  # observation input
                      {'dim': (batch_size, dim_pair)}, # pair input
                      {'dim': (batch_size, dim_action)}, # action input
                      {'dim': (batch_size, 1)},           # target_input
                      {'dim': (2)},           # target_input
                      ]})
        [n.net_input, n.pair_input, n.action_input, n.target_input, n.update_flag] = L.Python(ntop=5, python_param=dict(module='utils', param_str=data_layer_info, layer='DataLayer'))
    elif phase == TEST:
        data_layer_info = json.dumps({
            'shape': [{'dim': (1, dim_input)}
                      ]})
        n.net_input = L.Python(ntop=1, python_param=dict(module='utils', param_str=data_layer_info, layer='DataLayer'))
    else:
        raise Exception("  unknown phase...")
        
    # hidden layer
    n.ip0 = L.InnerProduct(n.net_input, num_output=dim_hidden[0], weight_filler=dict(type='xavier'), bias_filler=dict(type='constant',value=0))
    n.relu0 = L.ReLU(n.ip0, in_place=True)

    # ==========================================================================
    # actor: num needs to be dim_pair
    n.actor0 = L.InnerProduct(n.relu0, num_output=dim_hidden[1],weight_filler=dict(type='xavier'),bias_filler=dict(type='constant',value=0))
    n.actor_relu0 = L.ReLU(n.actor0, in_place=True)
    n.actor_out0 = L.InnerProduct(n.actor_relu0, num_output=dim_action,weight_filler=dict(type='xavier'), bias_filler=dict(type='constant',value=0))

    n.actor1 = L.InnerProduct(n.relu0, num_output=dim_hidden[1],weight_filler=dict(type='xavier'),bias_filler=dict(type='constant',value=0))
    n.actor_relu1 = L.ReLU(n.actor1, in_place=True)
    n.actor_out1 = L.InnerProduct(n.actor_relu1, num_output=dim_action,weight_filler=dict(type='xavier'), bias_filler=dict(type='constant',value=0))

    n.actor2 = L.InnerProduct(n.relu0, num_output=dim_hidden[1],weight_filler=dict(type='xavier'),bias_filler=dict(type='constant',value=0))
    n.actor_relu2 = L.ReLU(n.actor2, in_place=True)
    n.actor_out2 = L.InnerProduct(n.actor_relu2, num_output=dim_action,weight_filler=dict(type='xavier'), bias_filler=dict(type='constant',value=0))

    # ==========================================================================
    # critic: 
    n.critics = L.InnerProduct(n.relu0, num_output=dim_hidden[1],weight_filler=dict(type='xavier'), bias_filler=dict(type='constant',value=0))
    n.critics_relu = L.ReLU(n.critics, in_place=True)
    n.critics_out = L.InnerProduct(n.critics_relu, num_output=dim_pair,weight_filler=dict(type='xavier'), bias_filler=dict(type='constant',value=0))
    
    # output layer or loss layer
    if phase == TRAIN:
        n.neg_action_input = L.Power(n.action_input, scale = -1.0)
        # ======================================================================
        # collapse critics
        n.critics_out_eltwise = L.Eltwise(n.critics_out, n.pair_input,operation=0)
        n.critics_out_rdct = L.Reduction(n.critics_out_eltwise, operation=1, axis=1)
        n.critics_loss = L.EuclideanLoss(n.critics_out_rdct, n.target_input) 
        # ======================================================================
        # collapse actor: num needs to be dim_pair
        n.actor_sbstrct0 = L.Eltwise(n.actor_out0, n.neg_action_input, operation=1)
        n.actor_loss0 = L.Reduction(n.actor_sbstrct0, operation=3, axis=1)
        n.actor_loss0_reshape = L.Reshape(n.actor_loss0, reshape_param={'shape':{'dim': [batch_size, 1]}})

        n.actor_sbstrct1 = L.Eltwise(n.actor_out1, n.neg_action_input, operation=1)
        n.actor_loss1 = L.Reduction(n.actor_sbstrct1, operation=3, axis=1)
        n.actor_loss1_reshape = L.Reshape(n.actor_loss1, reshape_param={'shape':{'dim': [batch_size, 1]}})

        n.actor_sbstrct2 = L.Eltwise(n.actor_out2, n.neg_action_input, operation=1)
        n.actor_loss2 = L.Reduction(n.actor_sbstrct2, operation=3, axis=1)
        n.actor_loss2_reshape = L.Reshape(n.actor_loss2, reshape_param={'shape':{'dim': [batch_size, 1]}})

        n.actor_loss_concat = L.Concat(n.actor_loss0_reshape,n.actor_loss1_reshape,n.actor_loss2_reshape, axis=1)
        n.actor_loss_eltwise = L.Eltwise(n.actor_loss_concat, n.pair_input, operation=0)
        n.actor_loss_rdct1 = L.Reduction(n.actor_loss_eltwise, operation=1, axis=1)
        n.actor_loss = L.Reduction(n.actor_loss_rdct1, operation=4, axis=0, coeff = 0.5)
        # ======================================================================
        # collapse loss
        n.loss_conct = L.Concat(n.critics_loss, n.actor_loss, axis=0)
        n.loss_eltwise = L.Eltwise(n.loss_conct, n.update_flag, operation=0)
        n.loss = L.Reduction(n.loss_eltwise, operation=1, axis=0)
    else:
        pass
    return n.to_proto()


def CaffeSolver():
    solver_param = SolverParameter()
    solver_param.display = 0  
    solver_param.base_lr = 1e-3
    solver_param.lr_policy = 'fixed'
    # solver_param.momentum = 0.95
    # solver_param.weight_decay = 0.05
    solver_param.type = 'RMSProp'
    solver_param.random_seed = 1
    
    solver_param.train_net_param.CopyFrom(CaffeNet(BATCH_SIZE, DIM_INPUT, DIM_PAIR,DIM_ACTION,  DIM_HIDDEN, TRAIN))
    solver_param.test_net_param.add().CopyFrom(CaffeNet(BATCH_SIZE, DIM_INPUT,DIM_PAIR,DIM_ACTION, DIM_HIDDEN, TEST))
    
    solver_param.test_iter.append(1)
    solver_param.test_interval = 1000000
    
    with open('/tmp/CaffeNet.prototxt','w') as f:
        f.write(MessageToString(solver_param))
    
    # return caffe.get_solver(f.name)

class DataLayer(caffe.Layer):
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

if __name__ == '__main__':
    pass
