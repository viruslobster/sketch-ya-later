#!/usr/bin/env python2
import sys
import pickle
import numpy as np
import cv2
import serial
import python_serial
import time
NATIVE_M, NATIVE_N = 3160, 2107
ARDUINO_ADDR = '/dev/ttyACM0'
BAUD_R = 9600

with open(sys.argv[1], 'r') as f:
    points, n, m = pickle.load(f)

if NATIVE_M/float(NATIVE_N) > m / float(n):
    scale = NATIVE_N / float(n)
else:
    scale = NATIVE_M / float(m)


dx = int((NATIVE_M - m*scale)/2 + 0.5)
dy = int(NATIVE_N - n*scale)/2 + 0.5)

points = np.int32(points * scale + 0.5) + np.array(dx, dy)

ser = serial.Serial(ARDUINO_ADDR,BAUD_R)
time.sleep(python_serial.READY_HIT)
python_serial.send(ser, points.tolist())
