import numpy as np
import caffe
from caffe.proto.caffe_pb2 import SolverParameter,TRAIN, TEST
from google.protobuf.text_format import MessageToString
from NNBuilder import NNConstructor
from Policy_Caffe import CaffePolicy

class PolicyOptCaffe():
    def __init__(self, x_dim, u_dim):
        caffe.set_mode_cpu()
        self.batch_size = 25
        self.x_dim = x_dim
        self.u_dim = u_dim
        self.hidden_dim = 50
        
        self.init_solver()
        self.caffe_iter = 0
        self.var = 0.1 * np.ones(self.u_dim) # here 0.1 is the parameter set arbitrarily. It would be a better idea to bundle all parameter in a separate file.
        self.policy = CaffePolicy(self.solver.test_nets[0], np.zeros(self.u_dim))

    def init_solver(self):
        solver_param = SolverParameter()
        solver_param.display = 0  # Don't display anything.
        solver_param.base_lr = 0.001
        solver_param.lr_policy = 'fixed'
        solver_param.momentum = 0.9
        solver_param.weight_decay = 0.005
        solver_param.type = 'Adam'
        solver_param.random_seed = 1
         
        solver_param.train_net_param.CopyFrom(NNConstructor(self.x_dim,self.u_dim,self.hidden_dim,self.batch_size,TRAIN))
        solver_param.test_net_param.add().CopyFrom(NNConstructor(self.x_dim,self.u_dim,self.hidden_dim,1,TEST))
        
        solver_param.test_iter.append(1)
        solver_param.test_interval = 1000000
        
        with open('NeuralNetworks.prototxt','w') as f:
            f.write(MessageToString(solver_param))
        
        self.solver=caffe.get_solver(f.name)

    def setFoo(self):
        self.policy.foo = 100
