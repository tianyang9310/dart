import matplotlib.pyplot as plt

def Plot():
    DART_FILE_PATH = '../../build/tutorials/cartpole/'
    plt.rc('text',usetex=True)
    
    # Deal with x
    # read data from file
    x_dir = DART_FILE_PATH+'x'
    with open(x_dir) as x_file:
        x=[filter(None,(line.strip()).split(' ')) for line in x_file.readlines()]
        
    T = len(x[0])
    plt.figure(1)
    handle_x0,=plt.plot(x[0],linewidth=2.0)
    handle_x1,=plt.plot(x[1],linewidth=2.0)
    handle_x2,=plt.plot(x[2])
    handle_x3,=plt.plot(x[3])
    plt.axis([0,T,-3.5,3.5])
    handles = [handle_x0, handle_x1, handle_x2, handle_x3]
    mLegends = [r"$x",r"$\theta$",r"$\dot{x}$",r"$\dot{\theta}$"]
    plt.legend(handles, mLegends)
    plt.savefig(DART_FILE_PATH+'x.jpg')
    plt.show()
    
    ##############################################################################3j
    # Deal with x
    # read data from file
    x_dir = DART_FILE_PATH+'u'
    with open(x_dir) as x_file:
        u=[filter(None,(line.strip()).split(' ')) for line in x_file.readlines()]
        
    u=u[0]
    plt.figure(2)
    handle_u,=plt.plot(u[1:-1],linewidth=0.1)
    plt.legend([handle_u], ['u'])
    plt.savefig(DART_FILE_PATH+'u.jpg')
    plt.show()
