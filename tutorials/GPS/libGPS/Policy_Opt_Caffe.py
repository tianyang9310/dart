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

    def update(self, obs, tgt_mu, tgt_prc, tgt_wt, itr, inner_itr):
        """
        Update policy.
        Args:
            obs: Numpy array of observations, N x T x dO.
            tgt_mu: Numpy array of mean controller outputs, N x T x dU.
            tgt_prc: Numpy array of precision matrices, N x T x dU x dU.
            tgt_wt: Numpy array of weights, N x T.
        Returns:
            A CaffePolicy object with updated weights.
        """
        N, T = obs.shape[:2]
        dU, dO = self._dU, self._dO

        # TODO - Make sure all weights are nonzero?

        # Save original tgt_prc.
        tgt_prc_orig = np.reshape(tgt_prc, [N*T, dU, dU])

        # Renormalize weights.
        tgt_wt *= (float(N * T) / np.sum(tgt_wt))
        # Allow weights to be at most twice the robust median.
        mn = np.median(tgt_wt[(tgt_wt > 1e-2).nonzero()])
        for n in range(N):
            for t in range(T):
                tgt_wt[n, t] = min(tgt_wt[n, t], 2 * mn)
        # Robust median should be around one.
        tgt_wt /= mn

        # Reshape inputs.
        obs = np.reshape(obs, (N*T, dO))
        tgt_mu = np.reshape(tgt_mu, (N*T, dU))
        tgt_prc = np.reshape(tgt_prc, (N*T, dU, dU))
        tgt_wt = np.reshape(tgt_wt, (N*T, 1, 1))

        # Fold weights into tgt_prc.
        tgt_prc = tgt_wt * tgt_prc

        #TODO: Find entries with very low weights?

        # Normalize obs, but only compute normalzation at the beginning.
        if itr == 0 and inner_itr == 1:
            # 1e-3 to avoid infs if some state dimensions don't change in the
            # first batch of samples
            self.policy.scale = np.diag(1.0 / np.maximum(np.std(obs, axis=0),
                                                         1e-3))
            self.policy.bias = -np.mean(obs.dot(self.policy.scale), axis=0)
        obs = obs.dot(self.policy.scale) + self.policy.bias

        blob_names = self.solver.net.blobs.keys()

        # Assuming that N*T >= self.batch_size.
        batches_per_epoch = np.floor(N*T / self.batch_size)
        idx = range(N*T)
        average_loss = 0
        np.random.shuffle(idx)
        for i in range(self._hyperparams['iterations']):
            # Load in data for this batch.
            start_idx = int(i * self.batch_size %
                            (batches_per_epoch * self.batch_size))
            idx_i = idx[start_idx:start_idx+self.batch_size]
            self.solver.net.blobs[blob_names[0]].data[:] = obs[idx_i]
            self.solver.net.blobs[blob_names[1]].data[:] = tgt_mu[idx_i]
            self.solver.net.blobs[blob_names[2]].data[:] = tgt_prc[idx_i]

            self.solver.step(1)

            # To get the training loss:
            train_loss = self.solver.net.blobs[blob_names[-1]].data
            average_loss += train_loss
            if i % 500 == 0 and i != 0:
                LOGGER.debug('Caffe iteration %d, average loss %f',
                             i, average_loss / 500)
                average_loss = 0

        # Keep track of Caffe iterations for loading solver states.
        self.caffe_iter += self._hyperparams['iterations']

        # Optimize variance.
        A = np.sum(tgt_prc_orig, 0) + 2 * N * T * \
                self._hyperparams['ent_reg'] * np.ones((dU, dU))
        A = A / np.sum(tgt_wt)

        # TODO - Use dense covariance?
        self.var = 1 / np.diag(A)

        self.policy.net.share_with(self.solver.net)
        return self.policy


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
        np.reshape(self.Quu_inv,(-1,self.u_dim,self.u_dim))
        print self.Quu_inv

    def setFoo(self):
        self.policy.foo = 100
