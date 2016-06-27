import numpy as np
import caffe
from caffe.proto.caffe_pb2 import SolverParameter,TRAIN, TEST
from google.protobuf.text_format import MessageToString
from NNBuilder import NNConstructor
from Policy_Caffe import CaffePolicy
from file2numpy import file2numpy

class PolicyOptCaffe():
    def __init__(self, x_dim, u_dim, T, N):
        caffe.set_mode_cpu()
        self.batch_size = 25
        self.x_dim = x_dim
        self.u_dim = u_dim
        self.T     = T
        self.N     = N
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

    def pretrain(self):
    # def pretrain(self""", itr, inner_itr"""):
        self.var = np.mean(self.Quu_inv,axis=0)
        self.policy.var = self.var
        
        # Normalize obs, but only compute normalzation at the beginning.
        """
        No normalization right now
        if itr == 0 and inner_itr == 1:
            # 1e-3 to avoid infs if some state dimensions don't change in the
            # first batch of samples
            self.policy.scale = np.diag(1.0 / np.maximum(np.std(self.x, axis=0),
                                                         1e-3))
            self.policy.bias = -np.mean(self.x.dot(self.policy.scale), axis=0)
        obs = obs.dot(self.policy.scale) + self.policy.bias
        """
        print '---------------------------------------------------------'
        print '------------- Pretrain the neural networks --------------'
        print '---------------------------------------------------------'
            
        blob_names = self.solver.net.blobs.keys()
        
        # Assuming that N*T >= self.batch_size.
        batches_per_epoch = np.floor(N*T / self.batch_size)
        idx = range(N*T)
        cumulative_loss = 0
        np.random.shuffle(idx)
        for i in range(self.caffe_iteration):
            # Load in data for this batch.
            start_idx = int(i * self.batch_size %
                            (batches_per_epoch * self.batch_size))
            idx_i = idx[start_idx:start_idx+self.batch_size]
            self.solver.net.blobs[blob_names[0]].data[:] = self.x[idx_i]
            self.solver.net.blobs[blob_names[1]].data[:] = self.u[idx_i]
            self.solver.net.blobs[blob_names[2]].data[:] = self.var
        
            self.solver.step(1)
        
            # To get the training loss:
            train_loss = self.solver.net.blobs[blob_names[-1]].data
            cumulative_loss += train_loss
        
        self.policy.net.share_with(self.solver.net)        

    def ReadX(self):
        self.x = file2numpy('X.numpyout')
        print self.x

    def ReadU(self):
        # Supposing u_dim is 1, s.t. each row is an instance
        self.u = file2numpy('U.numpyout')
        print self.u

    def ReadQuu_inv(self):
        # Supposing u_dim is 1, s.t. each row is an instance
        self.Quu_inv = file2numpy('Quu_inv.numpyout')
        self.Quu_inv = np.reshape(self.Quu_inv,(-1,self.u_dim,self.u_dim))
        print self.Quu_inv

    def setFoo(self):
        self.policy.foo = 100
