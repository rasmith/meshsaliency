# import libraries

from __future__ import absolute_import

import numpy as np
import scipy.io as sio
import h5py
import os
import sys
import matplotlib.pyplot as plt
import glob
import cv2

# load data

path = 'off/'
in_files = os.listdir(path)
dir_len = len(in_files)

train_im = np.zeros((dir_len*11,256,256),dtype=np.float)

for i in range(dir_len):
	for j in range(11):
		img = cv2.imread(path + in_files[i] + '/png/out'+str(j)+'.png',cv2.IMREAD_GRAYSCALE)
		img = np.expand_dims(img,0)
		img = np.array([img])
		train_im[i*11+j,:,:] = img

np.save('train_set.npy',train_im)
