#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Demo of a histogram for 2 dimensional data as a bar graph in 3D.
'''

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

def pyHist3(x,y,name):
    x=x.astype(np.int)
    y=y.astype(np.int)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    #  x, y = np.random.rand(2, 100) * 4
    hist, xedges, yedges = np.histogram2d(x, y, bins=10, range=[[0, 9], [0, 9]])

    # Construct arrays for the anchor positions of the 16 bars.
    # Note: np.meshgrid gives arrays in (ny, nx) so we use 'F' to flatten xpos,
    # ypos in column-major order. For numpy >= 1.7, we could instead call meshgrid
    # with indexing='ij'.
    xpos, ypos = np.meshgrid(xedges[:-1] + 0.25, yedges[:-1] + 0.25)
    xpos = xpos.flatten('F')
    ypos = ypos.flatten('F')
    zpos = np.zeros_like(xpos)

    # Construct arrays with the dimensions for the 16 bars.
    dx = 0.5 * np.ones_like(zpos)
    dy = dx.copy()
    dz = hist.flatten()

    ax.bar3d(xpos, ypos, zpos, dx, dy, dz, color='b', zsort='average')

    #  plt.show()
    plt.savefig(name)

def plt1v1Hist(name,args):
    numArgs = len(args)
    for i in range(numArgs):
        for j in range(i+1,numArgs):
            pyHist3(args[i],args[j],name+'_ct_'+str(i)+'v'+str(j)+'png')
