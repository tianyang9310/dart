import numpy as np

class CaffePolicy():
    def __init__(self, test_net, var):
        self.net = test_net
        self.foo = 500
        self.var = var

    def act(self, x):
        '''
        x: state vector
        noise: is not required, which can be generated in this function
        return noised action according to x
        '''
        # casting whatever to np.array
        x = np.asarray(x)
        self.net.blobs[self.net.blobs.keys()[0]].data[:]=x
        u_mean = self.net.forward().values()[0][0]
        # If seed is None, then RandomState will try to read data from /dev/urandom (or the Windows analogue) if available or seed from the clock otherwise 
        u = np.random.multivariate_normal(u_mean,self.var)
        # u = u_mean
        u = tuple(u)
        return u

    def printFoo(self):
        print "~~~~~~~~~~~~~~~~~~~~~ Policy Net Parameters ~~~~~~~~~~~~~~~~~~~~~"
        params_names = self.net.params.keys()
        print params_names
        print self.net.params[params_names[0]][0].data[0:1,:]

