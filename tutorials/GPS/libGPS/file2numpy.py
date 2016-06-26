import numpy as np
data_dir_path = '../../../build/tutorials/GPS/' 

def file2numpy(filename='X.numpyout'):
    data_file = data_dir_path + filename
    with open(data_file) as file:
        data=np.array([filter(None,(line.strip()).split(' ')) for line in file.readlines()])
    return data

