from NNBuilder import NNConstructor

batch_size = 25

with open('train.prototxt','w') as f:
    f.write(str(NNConstructor(7,2,50,batch_size,0)))

with open('test.prototxt','w') as f:
    f.write(str(NNConstructor(7,2,50,batch_size,1)))
