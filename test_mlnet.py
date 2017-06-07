#!/usr/bin/env python3
from mlnet_model import *
m = MlnetModel('weights/vgg16_weights.h5', 'weights/mlnet_salicon_weights.pkl')
m.initialize()
