from __future__ import division
from keras.models import Model
from keras.layers.core import Dropout, Activation
from keras.layers import Input, merge
from keras.layers.convolutional import Convolution2D, MaxPooling2D
from keras.regularizers import l2
import keras.backend as K
import h5py
import math
from eltwise_product import EltWiseProduct

#########################################################################
# MODEL PARAMETERS														#
#########################################################################
# batch size
b_s = 10
# number of rows of input images
shape_r = 480
# number of cols of input images
shape_c = 640
# number of rows of predicted maps
shape_r_gt = int(math.ceil(shape_r / 8))
# number of cols of predicted maps
shape_c_gt = int(math.ceil(shape_c / 8))
# number of epochs
nb_epoch = 20

#########################################################################
# TRAINING SETTINGS										            	#
#########################################################################
# path of training images
imgs_train_path = '/path/to/training/images/'
# path of training maps
maps_train_path = '/path/to/training/maps/'
# number of training images
nb_imgs_train = 10000
# path of validation images
imgs_val_path = '/path/to/validation/images/'
# path of validation maps
maps_val_path = '/path/to/validation/maps/'
# number of validation images
nb_imgs_val = 5000

def padding(img, shape_r=480, shape_c=640, channels=3):
    img_padded = np.zeros((shape_r, shape_c, channels), dtype=np.uint8)
    if channels == 1:
        img_padded = np.zeros((shape_r, shape_c), dtype=np.uint8)

    original_shape = img.shape
    rows_rate = original_shape[0] / shape_r
    cols_rate = original_shape[1] / shape_c

    if rows_rate > cols_rate:
        new_cols = (original_shape[1] * shape_r) // original_shape[0]
        img = cv2.resize(img, (new_cols, shape_r))
        if new_cols > shape_c:
            new_cols = shape_c
            img_padded[:, ((img_padded.shape[1] - new_cols) // 2)
                     :((img_padded.shape[1] - new_cols) // 2 + new_cols)] = img
    else:
        new_rows = (original_shape[0] * shape_c) // original_shape[1]
        img = cv2.resize(img, (shape_c, new_rows))
        if new_rows > shape_r:
            new_rows = shape_r
            img_padded[((img_padded.shape[0] - new_rows) // 2)
                :((img_padded.shape[0] - new_rows) // 2 + new_rows), :] = img
    return img_padded


def preprocess_image(original_image):
    padded_image = padding(original_image, shape_r, shape_c, 3)
    padded_image[:, :, 0] -= 103.939
    padded_image[:, :, 1] -= 116.779
    padded_image[:, :, 2] -= 123.68
    padded_image = padded_image.transpose((2, 0, 1))


def postprocess_map(original_map):
    padded_map = padding(original_map, shape_r, shape_c, 1)
    return padded_map.astype(np.float32) / 255.0


def postprocess_prediction(pred, shape_r, shape_c):
    predictions_shape = pred.shape
    rows_rate = shape_r / predictions_shape[0]
    cols_rate = shape_c / predictions_shape[1]

    if rows_rate > cols_rate:
        new_cols = (predictions_shape[1] * shape_r) // predictions_shape[0]
        pred = cv2.resize(pred, (new_cols, shape_r))
        img = pred[:, ((pred.shape[1] - shape_c) // 2)
        :((pred.shape[1] - shape_c) // 2 + shape_c)]
    else:
        new_rows = (predictions_shape[0] * shape_c) // predictions_shape[1]
        pred = cv2.resize(pred, (shape_c, new_rows))
        img = pred[((pred.shape[0] - shape_r) // 2)
        :((pred.shape[0] - shape_r) // 2 + shape_r), :]

    return img / np.max(img) * 255


class MlnetModel(object):
    def __init__(self, vgg_weights_file, pkl_weights_file):
        self.vgg_weights_file = vgg_weights_file
        self.pkl_weights_file = pkl_weights_file

    def get_weights_vgg16(self, f, id):
        g = f['layer_{}'.format(id)]
        return [g['param_{}'.format(p)] for p in range(g.attrs['nb_params'])]

    def loss(self, y_true, y_pred):
        max_y = K.repeat_elements(K.expand_dims(K.repeat_elements(
            K.expand_dims(K.max(K.max(y_pred, axis=2), axis=2)),
            shape_r_gt, axis=-1)), shape_c_gt, axis=-1)
        return K.mean(K.square((y_pred / max_y) - y_true) / (1 - y_true + 0.1))

    def initialize(self, img_rows=480, img_cols=640,
                   downsampling_factor_net=8, downsampling_factor_product=10):
        f = h5py.File(self.vgg_weights_file)
        input_ml_net = Input(shape=(3, img_rows, img_cols))
        #########################################################
        # FEATURE EXTRACTION NETWORK              #
        #########################################################
        weights = self.get_weights_vgg16(f, 1)
        conv1_1 = Convolution2D(64, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(input_ml_net)
        weights = self.get_weights_vgg16(f, 3)
        conv1_2 = Convolution2D(64, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv1_1)
        conv1_pool = MaxPooling2D((2, 2), strides=(2, 2),
                                  border_mode='same')(conv1_2)

        weights = self.get_weights_vgg16(f, 6)
        conv2_1 = Convolution2D(128, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv1_pool)
        weights = self.get_weights_vgg16(f, 8)
        conv2_2 = Convolution2D(128, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv2_1)
        conv2_pool = MaxPooling2D((2, 2), strides=(2, 2),
                                  border_mode='same')(conv2_2)

        weights = self.get_weights_vgg16(f, 11)
        conv3_1 = Convolution2D(256, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv2_pool)
        weights = self.get_weights_vgg16(f, 13)
        conv3_2 = Convolution2D(256, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv3_1)
        weights = self.get_weights_vgg16(f, 15)
        conv3_3 = Convolution2D(256, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv3_2)
        conv3_pool = MaxPooling2D((2, 2), strides=(2, 2),
                                  border_mode='same')(conv3_3)

        weights = self.get_weights_vgg16(f, 18)
        conv4_1 = Convolution2D(512, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv3_pool)
        weights = self.get_weights_vgg16(f, 20)
        conv4_2 = Convolution2D(512, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv4_1)
        weights = self.get_weights_vgg16(f, 22)
        conv4_3 = Convolution2D(512, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv4_2)
        conv4_pool = MaxPooling2D((2, 2), strides=(1, 1),
                                  border_mode='same')(conv4_3)

        weights = self.get_weights_vgg16(f, 25)
        conv5_1 = Convolution2D(512, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv4_pool)
        weights = self.get_weights_vgg16(f, 27)
        conv5_2 = Convolution2D(512, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv5_1)
        weights = self.get_weights_vgg16(f, 29)
        conv5_3 = Convolution2D(512, 3, 3, weights=weights, activation='relu',
                                border_mode='same')(conv5_2)

        #########################################################
        # ENCODING NETWORK                    #
        #########################################################
        concatenated = merge([conv3_pool, conv4_pool, conv5_3], mode='concat',
                             concat_axis=1)
        dropout = Dropout(0.5)(concatenated)

        int_conv = Convolution2D(64, 3, 3, init='glorot_normal',
                                 activation='relu', border_mode='same')(dropout)

        pre_final_conv = Convolution2D(1, 1, 1, init='glorot_normal',
                                       activation='relu')(int_conv)

        #########################################################
        # PRIOR LEARNING                    #
        #########################################################
        rows_elt = math.ceil(
            img_rows / downsampling_factor_net) // downsampling_factor_product
        cols_elt = math.ceil(
            img_cols / downsampling_factor_net) // downsampling_factor_product
        eltprod = EltWiseProduct(init='zero',
                                 W_regularizer=l2(1 / (rows_elt * cols_elt)))(pre_final_conv)
        output_ml_net = Activation('relu')(eltprod)

        self.model = Model(input=[input_ml_net], output=[output_ml_net])
        self.model.load_weights(self.pkl_weights_file)
