'''
# =============================================================================
#      FileName: Qnetwork.py
#          Desc: Deep Q network
#        Author: Yang Tian
#         Email: tianyang9310@gmail.com
#    LastChange: 2016-11-04 17:52:51
# =============================================================================
'''
import os
os.environ['GLOG_minloglevel']='2'

import sys
# caffe_python_root = '/opt/caffe/python'
# sys.path.insert(0,caffe_python_root)

# import modules
# ------------------------------
import gym
# ------------------------------
import caffe
from caffe.proto.caffe_pb2 import SolverParameter, TRAIN, TEST
from caffe import layers as L
# ------------------------------
import time
import json
import numpy as np
from collections import deque
from google.protobuf.text_format import MessageToString
# ------------------------------
utils_path = '../'
sys.path.insert(0,utils_path)
from utils.FigureBundle import IncrementalPlotting

# ------------------------------
# global variables
INITEPSILON = 0.5
FINALEPSILON = 0.2
MEMBUFFERSIZE = 8000
BATCHSIZE = 32
DIMHIDDEN = 20
MAXEPISODE = 10000
MAXSTEPS = 500
GAMMA = 0.9
EPSILONDECAY = 15000
TARGETUPDATEF = 100
TESTITER = 100
# ------------------------------
# Qnetwork solver parameter
# solver_param.display = 0  
# solver_param.base_lr = 1e-3
# solver_param.lr_policy = 'fixed'
# # solver_param.momentum = 0.95
# # solver_param.weight_decay = 0.05
# solver_param.type = 'SGD'
# solver_param.random_seed = 1

# Q-network class
class Qnet(object):
    def __init__(self, env):
        self.env = env
        
        caffe.set_mode_gpu()
        self.batch_size = BATCHSIZE
        self.dim_input = env.observation_space.shape[0]
        self.dim_output = env.action_space.n
        self.dim_hidden = DIMHIDDEN
        self.CaffeSolver()
        
        self.TargetSolver()
        self.UpdateTarget()
        self.mem_buffer = deque()
        self.mem_buffer_size = MEMBUFFERSIZE
        self.epsilon = INITEPSILON

    def CaffeNet(self, batch_size, dim_input, dim_output, dim_hidden, phase):
        n = caffe.NetSpec()
        
        # data layer
        if phase == TRAIN:
            data_layer_info = json.dumps({
                'shape': [{'dim': (batch_size, dim_input)},  # observation input
                          {'dim': (batch_size, dim_output)}, # action input
                          {'dim': (batch_size, 1)}           # target_input
                          ]})
            [net_input, action_input, target_input] = L.Python(ntop=3, python_param=dict(module='Qnetwork', param_str=data_layer_info, layer='DataLayer'))
        elif phase == TEST:
            data_layer_info = json.dumps({
                'shape': [{'dim': (1, dim_input)}
                          ]})
            net_input = L.Python(ntop=1, python_param=dict(module='Qnetwork', param_str=data_layer_info, layer='DataLayer'))
        else:
            raise Exception("  unknown phase...")
            
        # hidden layer
        cur_top = net_input
        cur_top = L.InnerProduct(cur_top, num_output=dim_hidden, weight_filler=dict(type='xavier'), bias_filler=dict(type='constant',value=0))
        cur_top = L.ReLU(cur_top, in_place=True)
        cur_top = L.InnerProduct(cur_top, num_output=dim_output, weight_filler=dict(type='xavier'),bias_filler=dict(type='constant',value=0))
        
        # output layer or loss layer
        if phase == TRAIN:
            cur_top = L.Eltwise(cur_top, action_input, operation=0)
            cur_top = L.Reduction(cur_top, operation=1, axis=1)
            out = L.EuclideanLoss(cur_top, target_input, name='losslayer') 
        else:
            out = cur_top
        return out.to_proto()

    def CaffeSolver(self):
        solver_param = SolverParameter()
        solver_param.display = 0  
        solver_param.base_lr = 1e-3
        solver_param.lr_policy = 'fixed'
        # solver_param.momentum = 0.95
        # solver_param.weight_decay = 0.05
        solver_param.type = 'RMSProp'
        solver_param.random_seed = 1
        
        solver_param.train_net_param.CopyFrom(self.CaffeNet(self.batch_size, self.dim_input, self.dim_output, self.dim_hidden, TRAIN))
        solver_param.test_net_param.add().CopyFrom(self.CaffeNet(self.batch_size, self.dim_input, self.dim_output, self.dim_hidden, TEST))
        
        solver_param.test_iter.append(1)
        solver_param.test_interval = 1000000
        
        with open('/tmp/CaffeNet.prototxt','w') as f:
            f.write(MessageToString(solver_param))
        
        self.solver=caffe.get_solver(f.name)

    def TargetSolver(self):
        solver_param = SolverParameter()
        solver_param.display = 0  
        solver_param.base_lr = 1e-3
        solver_param.lr_policy = 'fixed'
        # solver_param.momentum = 0.95
        # solver_param.weight_decay = 0.05
        solver_param.type = 'RMSProp'
        solver_param.random_seed = 1
        
        solver_param.train_net_param.CopyFrom(self.CaffeNet(self.batch_size, self.dim_input, self.dim_output, self.dim_hidden, TRAIN))
        
        with open('/tmp/TargetNet.prototxt','w') as f:
            f.write(MessageToString(solver_param))
        
        self.target=caffe.get_solver(f.name)

    def UpdateTarget(self):
        # ------------------------
        # solver => target
        # ------------------------
        filename='/tmp/SolverNet.caffemodel'
        self.solver.net.save(filename)
        self.target.net.copy_from(filename)

    def Perceive(self, obs, action, next_obs, reward, done):
        self.mem_buffer.append([obs, action, next_obs, reward, done])
        if len(self.mem_buffer) > self.mem_buffer_size:
            self.mem_buffer.popleft()
        if len(self.mem_buffer) >= self.batch_size:
            index = np.random.randint(len(self.mem_buffer), size=self.batch_size)  
            minibatch_obs = np.array([self.mem_buffer[idx][0] for idx in index])
            # minibatch_action = np.zeros((self.batch_size,self.dim_output))
            # minibatch_target = np.zeros((self.batch_size,1))
            # for i, idx in enumerate(index):
            #     minibatch_action[i][self.mem_buffer[idx][1]] = 1
            #     if self.mem_buffer[idx][4]: # done
            #         minibatch_target[i][0] = self.mem_buffer[idx][3]
            #     else:
            #         minibatch_target[i][0] = self.mem_buffer[idx][3] + GAMMA*np.max(self.Eval(self.mem_buffer[idx][2])[0])
            minibatch_action = np.array([np.array([1.,0.]) if self.mem_buffer[idx][1]==0 else np.array([0.,1.]) for idx in index])
            minibatch_next_obs = np.array([self.mem_buffer[idx][2] for idx in index])
            minibatch_next_Qarray = self.Eval(minibatch_next_obs)
            minibatch_target = np.array([np.array([self.mem_buffer[idx][3]]) if self.mem_buffer[idx][4] else np.array([self.mem_buffer[idx][3]+GAMMA*np.max(minibatch_next_Qarray[i])]) for i, idx in enumerate(index)])
            self.Train(minibatch_obs, minibatch_action, minibatch_target)

    def Train(self, minibatch_obs, minibatch_action, minibatch_target):
        # print minibatch_obs
        # print minibatch_action
        # print minibatch_target
        # x=raw_input()

        blob_names = self.solver.net.blobs.keys()
        self.solver.net.blobs[blob_names[0]].data[:] = minibatch_obs
        self.solver.net.blobs[blob_names[1]].data[:] = minibatch_action
        self.solver.net.blobs[blob_names[2]].data[:] = minibatch_target

        self.solver.step(1)

        train_loss = self.solver.net.blobs['EuclideanLoss1'].data # blob_names[-2] is 'EuclideanLoss' blob

        if train_loss > 1e3:
            print train_loss

    def Eval(self, obs):
        blob_names = self.target.net.blobs.keys()
        self.target.net.blobs[blob_names[0]].data[:] = obs
        self.target.net.blobs[blob_names[1]].data[:] = 0
        self.target.net.blobs[blob_names[2]].data[:] = 0
        self.target.net.forward()
        Qarray = self.target.net.blobs['InnerProduct2'].data # blob_names[-4] is 'InnerProduct2' blob
        return Qarray

    def egreedyAct(self, obs): 
        self.epsilon -= (INITEPSILON-FINALEPSILON)/float(MAXEPISODE*MAXSTEPS/3.0)
        print "Current epsilon is {}".format(self.epsilon)
        if np.random.uniform() <= self.epsilon:
            action = self.env.action_space.sample() 
            # print 'random action: ', action
            return action
        else:
            blob_names = self.solver.test_nets[0].blobs.keys()
            self.solver.test_nets[0].blobs[blob_names[0]].data[:] = obs
            Qarray = self.solver.test_nets[0].forward()
            action = np.argmax(Qarray['InnerProduct2']) # because action space is {0, 1}
            # print 'greedy action: ', action
            return action  

    def Act(self, obs):
        blob_names = self.solver.test_nets[0].blobs.keys()
        self.solver.test_nets[0].blobs[blob_names[0]].data[:] = obs
        Qarray = self.solver.test_nets[0].forward()
        action = np.argmax(Qarray['InnerProduct2']) # because action space is {0, 1}
        # print 'greedy action: ', action
        return action  


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

# main function
if __name__ == "__main__":
    env=gym.make("CartPole-v1")
    qnet = Qnet(env)
    myPlt = IncrementalPlotting()
    for s in xrange(MAXEPISODE):
        obs = env.reset()
        for t in xrange(MAXSTEPS):
            action = qnet.egreedyAct(obs)
            next_obs, reward, done, info = env.step(action)
            qnet.Perceive(obs, action, next_obs, reward, done)
            obs = next_obs
            if done:
                # print 'Episode %d terminates at %d time step' % (s, t)
                break
        if s % TARGETUPDATEF == 0:
            qnet.UpdateTarget()
        if s % 100 == 0:
            total_reward = 0.0
            for s_test in xrange(TESTITER):
                obs = env.reset()
                for t in xrange(MAXSTEPS):
                    # env.render()
                    action = qnet.Act(obs)
                    next_obs, reward, done, info = env.step(action)
                    # time.sleep(0.01)
                    obs = next_obs
                    total_reward += reward
                    if done:
                        # print 'Episode %d terminates at %d time step' % (s, t)
                        break
            aver_reward = float(total_reward)/TESTITER
            myPlt.plot(aver_reward)
            print 'Episode %d, total reward is %.2f, average reward is %.2f' % (s,total_reward,aver_reward)
            
