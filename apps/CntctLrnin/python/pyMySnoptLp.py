import numpy as np
import scipy.sparse as sp
from optimize.snopt7 import SNOPT_solver

inf = 1.0e20

def mPlaceHolder(status,x,needF,needG,cu,iu,ru):
    F = []
    G = []
    return status, F, G

def MySnoptLP(A,b):
    snopt = SNOPT_solver()
    snopt.setOption('Print file','SNOPTLP.out')
    snopt.setOption('Minor print level',1)
    snopt.setOption('Summary frequency',1)
    snopt.setOption('Print frequency',1)

    row, col = A.shape

    nF = row
    ObjRow = 0
    n = col
    
    x0 = np.zeros(n)
    xlow = np.zeros(n)
    xupp = inf * np.ones(n)

    Flow = b
    Fupp = b

    snopt.setOption('Verbose',True)
    snopt.snopta(name='   snopt lp',xlow=xlow,xupp=xupp,x0=x0,
                 Flow=Flow,Fupp=Fupp,ObjRow=ObjRow,A=A,
                 usrfun=mPlaceHolder)
    print snopt.x
    print A.dot(snopt.x.reshape((col,1)))+b.reshape((col,1))

if __name__ == '__main__':
    A = np.genfromtxt('/tmp/A.csv',dtype=np.float64,delimiter=',')
    b = np.genfromtxt('/tmp/b.csv',dtype=np.float64,delimiter=',')
    #  print A
    #  print b.T
    MySnoptLP(A,-b)

