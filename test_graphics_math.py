#!/usr/bin/env python3
import numpy as np
import graphics_math as gm

eye=np.transpose(np.array([[1.0, 1.0, 1.0, 1.0]]))
at=np.zeros((4, 1))
up=np.zeros((4, 1))
up[1,0]=1.0

V=gm.lookat(eye,at,up)

np.set_printoptions(precision=3)
print("V=%s" % str(V))
print("V*eye=%s" % str(np.dot(V, eye)))
print("V*up=%s" % str(np.dot(V, up)))

width=640
height=480
P=gm.perspective(45.0, float(width)/float(height), 0.001, 100.0)
print("P=%s" % str(P))

