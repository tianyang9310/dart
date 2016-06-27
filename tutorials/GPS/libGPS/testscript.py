from Policy_Opt_Caffe import PolicyOptCaffe
import numpy as np
tt=PolicyOptCaffe(4,1,1999,10)

tt.ReadX()
tt.ReadU()
tt.ReadQuu_inv()

tt.pretrain()

tt.policy.act(np.array([0,0,0,0]))
