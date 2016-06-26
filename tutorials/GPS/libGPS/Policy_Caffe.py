import numpy as np

class CaffePolicy():
    def __init__(self, test_net, var):
        # self.chol_pol_covar
        self.net = test_net
        
        self.foo = 500

    def act(self, x, noise):
        '''
        x: state vector
        noise: is not requred, which can be generated in this function
        return noised action according to x
        '''
