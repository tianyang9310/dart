from NNBuilder import NNConstructor

with open('train.prototxt','w') as f:
    f.write(str(NNConstructor(7,2,50,25,0)))

with open('test.prototxt','w') as f:
    f.write(str(NNConstructor(7,2,50,25,1)))
