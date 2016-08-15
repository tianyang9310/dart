import matplotlib.pyplot as plt
import numpy as np

DART_FILE_PATH = '../../build/tutorials/cartpole/'
plt.rc('text',usetex=True)
plt.close("all")
f,(ax1,ax2)=plt.subplots(1,2)


# Deal with x
# read data from file
x_dir = DART_FILE_PATH+'x.out'
with open(x_dir) as x_file:
    x=np.array([filter(None,(line.strip()).split(' ')) for line in x_file.readlines()])
    
x = x.astype(np.float)
T = len(x[0])
handle_x0,=ax1.plot(x[0],linewidth=2.0)
handle_x1,=ax1.plot(x[1],linewidth=2.0)
handle_x2,=ax1.plot(x[2])
handle_x3,=ax1.plot(x[3])
ax1.axis([0,T,-3.5,3.5])
handles = [handle_x0, handle_x1, handle_x2, handle_x3]
mLegends = [r"$x$",r"$\theta$",r"$\dot{x}$",r"$\dot{\theta}$"]
ax1.legend(handles, mLegends)

##############################################################################3j
# Deal with x
# read data from file
x_dir = DART_FILE_PATH+'u.out'
with open(x_dir) as x_file:
    u=np.array([filter(None,(line.strip()).split(' ')) for line in x_file.readlines()])
    
u=u[0]
u = u.astype(np.float)
u_max = max(u[1:-1])
handle_u,=ax2.plot(u[1:-1],linewidth=0.1)
ax2.axis([0,T,-2*u_max,2*u_max])
ax2.legend([handle_u], ['u'])
plt.savefig(DART_FILE_PATH+'x_and_u.jpg')
plt.show()
