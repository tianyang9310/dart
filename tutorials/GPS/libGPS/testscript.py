from Policy_Opt_Caffe import PolicyOptCaffe
import numpy as np
import math
# def StepCost(_x,_u):
#     xd = np.array([0, math.pi, 0, 0])
#     Q  = np.array([[0.01,0,0,0],
#                    [0   ,5,0,0],
#                    [0   ,0,0,0],
#                    [0   ,0,0,0]])
#     R  = np.array([1])
#     # Matrix4d Qf = Matrix4d::Identity()
#     # Qf(1,1)				= 500
#     Q = Q*0.001
#     R = R*0.001
#     return (0.5*(_x-xd).dot(Q.dot(_x-xd)) + 0.5*_u.dot(R.dot(_u)))
# 
# print StepCost(np.array([0,0,0,0]),np.array(1))

tt=PolicyOptCaffe(4,1,99,10,24)

tt.ReadX()
tt.ReadU()
tt.ReadQuu_inv()

tt.pretrain()

tt.ReadSampleSets_X()
tt.ReadSampleSets_U()
tt.ReadSampleSets_Quu_inv()
tt.ReadSampleSets_Logq()
tt.setWr()

tt.trainnet2forward()
print tt.cur_lossvalue_wo
tt.finetune()
tt.trainnet2forward()
print tt.cur_lossvalue_wo

tt.policy.act(np.array([0,0,0,0]))

