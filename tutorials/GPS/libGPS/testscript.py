from Policy_Opt_Caffe import PolicyOptCaffe

tt=PolicyOptCaffe(4,1,1999,10)

tt.ReadX()
tt.ReadU()
tt.ReadQuu_inv()

tt.pretrain()
