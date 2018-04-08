#!/usr/bin/env python2
import sys
import pickle
import numpy as np
import cv2
import math

with open(sys.argv[1], 'r') as f:
    points, m, n = pickle.load(f)

img = np.zeros((m, n, 3)).astype('uint8')

last = None
for i in range(points.shape[0]):
    (j, i) = points[i, :]
    if last:
        cv2.line(img, (j, i), last, (0, 0, 255))

        cv2.imshow('image', img)
        cv2.waitKey(int(1*math.sqrt((j-last[0])**2 + (i-last[1])**2)))
        cv2.line(img, (j, i), last, (255, 255, 255))
    last = (j, i)

print 'Done!'

cv2.waitKey(0)
