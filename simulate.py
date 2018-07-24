#!/usr/bin/env python2
import sys
import pickle
import numpy as np
import cv2
import math

with open(sys.argv[1], 'r') as f:
    points, m, n = pickle.load(f)

if len(sys.argv) > 3:
    SCALE = float(sys.argv[3])
else:
    SCALE = 1

if len(sys.argv) > 2:
    TIME = int(sys.argv[2])
else:
    TIME = 10

img = np.zeros((int(m*SCALE), int(n*SCALE), 3)).astype('uint8')

last = None
for k in range(points.shape[0]):
    (j, i) = points[k, :] * SCALE
    (j, i) = int(j), int(i)
    if last:
        # print k
        cv2.line(img, (j, i), last, (0, 0, 255))
        cv2.imshow('image', img)
        t = max(math.sqrt((j-last[0])**2 + (i-last[1])**2), 1)
        cv2.waitKey(int(TIME*t))
        cv2.line(img, (j, i), last, (255, 255, 255))
    last = (j, i)

print 'Done!'

cv2.waitKey(0)
