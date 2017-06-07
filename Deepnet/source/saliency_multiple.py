from __future__ import absolute_import
import cv2
import os
import h5py
import numpy as np
import numpy.matlib as matlib
import scipy.io as sio
import tensorflow as tf
from functools import partial
from itertools import product
import code
from glob import glob
import warnings
import matplotlib.pyplot as plt

from keras import optimizers
from keras.models import Sequential
from keras.layers import Convolution2D, MaxPooling2D, ZeroPadding2D, AtrousConvolution2D, UpSampling2D, Deconvolution2D
from keras.layers import Convolution1D, ZeroPadding1D, MaxPooling1D
from keras.layers import Activation, Dropout, Flatten, Dense, Lambda, Reshape, Permute, Cropping2D, merge, Embedding
from keras.layers.merge import Add, Average 
from keras import backend as K
from keras.callbacks import ModelCheckpoint, LearningRateScheduler, TensorBoard
from keras.objectives import categorical_crossentropy
from keras.models import Model
from keras.layers import Input
from keras.layers import AveragePooling2D
from keras.utils.layer_utils import convert_all_kernels_in_model
from keras.utils.data_utils import get_file
from keras import backend as K
from keras.engine.topology import Layer
from keras.utils.layer_utils import convert_all_kernels_in_model
from keras.utils.data_utils import get_file
from keras.optimizers import SGD, RMSprop

train_set = '../data/train_set.npy'
sal_set = '../data/sal_set.npy'
checkpoint_file = '../models/model.hdf5'
np.random.seed(1337)
nb_epoch = 25
img_size = 256


def base_network(input_dims):
    inputs = Input(shape = input_dims)
    #layer 1
    conv1 = Convolution2D(16, 3, 3, border_mode = 'same', activation='relu')(inputs)
    conv1 = Convolution2D(16, 3, 3, border_mode = 'same', activation='relu')(conv1)
    pool1 = MaxPooling2D(pool_size=(2, 2))(conv1)
    #layer 2
    conv2 = Convolution2D(32, 3, 3, border_mode = 'same', activation='relu')(pool1)
    conv2 = Convolution2D(32, 3, 3, border_mode = 'same', activation='relu')(conv2)
    pool2 = MaxPooling2D(pool_size=(2, 2))(conv2)
    #layer 3
    conv3 = Convolution2D(64, 3, 3, border_mode = 'same', activation='relu')(pool2)
    conv3 = Convolution2D(64, 3, 3, border_mode = 'same', activation='relu')(conv3)
    pool3 = MaxPooling2D(pool_size=(2, 2))(conv3)
    #layer 4
    conv4 = Convolution2D(128, 3, 3, border_mode = 'same', activation='relu')(pool3)
    conv4 = Convolution2D(128, 3, 3, border_mode = 'same', activation='relu')(conv4)
    pool4 = MaxPooling2D(pool_size=(2, 2))(conv4)
    #layer 5
    upsm5 = UpSampling2D(size=(2,2))(pool4)
    conv5 = Convolution2D(64, 3, 3, border_mode = 'same', activation='relu')(upsm5)
    conv5 = Convolution2D(64, 3, 3, border_mode = 'same', activation='relu')(conv5)
    #layer 6
    upsm6 = UpSampling2D(size=(2,2))(conv5)
    conv6 = Convolution2D(32, 3, 3, border_mode = 'same', activation='relu')(upsm6)
    conv6 = Convolution2D(32, 3, 3, border_mode = 'same', activation='relu')(conv6)
    #layer 7
    upsm7 = UpSampling2D(size=(2,2))(conv6)
    conv7 = Convolution2D(16, 3, 3, border_mode = 'same', activation='relu')(upsm7)
    conv7 = Convolution2D(16, 3, 3, border_mode = 'same', activation='relu')(conv7)
    #layer 8
    upsm8 = UpSampling2D(size=(2,2))(conv7)
    conv8 = Convolution2D(8, 3, 3, border_mode = 'same', activation='relu')(upsm8)
    outputs = Convolution2D(1, 3, 3, border_mode = 'same', activation='relu')(conv8)

    return Model(inputs,outputs)
def train():

    imgs = np.load(train_set)
    print imgs.shape
    sals = np.load(sal_set)
    print sals.shape
    input_dim = (img_size,img_size,1)

    saliency_network = base_network(input_dim)

    input_img = Input(shape=input_dim)
    out1 = saliency_network(input_img)
    
    input_img2 = AveragePooling2D(pool_size=(2,2))(input_img)
    out2 = saliency_network(input_img2)
    out2 = UpSampling2D(size=(2,2))(out2)
    out2 = Convolution2D(1, 3, 3, border_mode = 'same', activation='relu')(out2)

    input_img3 = AveragePooling2D(pool_size=(2,2))(input_img2)
    out3 = saliency_network(input_img3)
    out3 = UpSampling2D(size=(2,2))(out3)
    out3 = Convolution2D(1, 3, 3, border_mode = 'same', activation='relu')(out3)
    out3 = UpSampling2D(size=(2,2))(out3)
    out3 = Convolution2D(1, 3, 3, border_mode = 'same', activation='relu')(out3)

    output_sal = Average()([out1,out2,out3])
    
    model = Model(input=input_img, output=output_sal)
    model.summary()

    # train
    model_checkpoint = ModelCheckpoint(checkpoint_file, monitor='loss')
    rms = RMSprop()
    model.compile(loss='mse', optimizer=rms)
    
    lr=1e-3
    for i in range(1): # num times to drop learning rate
        print('Learning rate: {0}'.format(lr))
        K.set_value(model.optimizer.lr, lr)
        model_data = np.expand_dims(imgs,axis=3)
        saliency_data = np.expand_dims(sals,axis=3)
        history = model.fit([model_data], [saliency_data], validation_split = 0.1, batch_size=128, nb_epoch=nb_epoch, callbacks=[model_checkpoint])
        lr = lr*.1
        print(history.history.keys())
        # summarize history for accuracy
        # summarize history for loss
        plt.plot(history.history['loss'])
        plt.plot(history.history['val_loss'])
        plt.title('model loss')
        plt.ylabel('loss')
        plt.xlabel('epoch')
        plt.legend(['train', 'validation'], loc='upper left')
        plt.show()
    
        

def test():

    print ("Loading Model") 
    #transformed_img = image_file.astype(np.float32)
    #transformed_img = np.expand_dims(transformed_img,axis=0)
    #print transformed_img.shape
    #img_shape = (img_size,img_size)

    #model = load_net(weights_path,num_outputs=num_outputs,input_size=img_shape)
    #model.load_weights(checkpoint_file)

 
    #print ("Predicting")
    #prediction = model.predict(transformed_img, verbose=1)
    #print prediction.shape
    #prediction = prediction[0]
    #print prediction
    #return prediction

if __name__ == '__main__':
	train()
