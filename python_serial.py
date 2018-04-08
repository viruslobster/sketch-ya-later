#!/usr/bin/env python2
import serial
import time
import struct
import array

ARDUINO_ADDR = '/dev/ttyACM0'
BAUD_R = 9600

MAX_NUM_SEND = 64

READY_MISS = 3
READY_HIT = 3

EEPROM = 2
DRAW = 1


def points_to_shortlist(point_list):
    shortlist = array.array('H')
    for point in point_list:
        shortlist.append(point[0])
        shortlist.append(point[1])
    return shortlist

def sublist_to_bstring(sublist):
    bit_string = b''
    for num in sublist:
        bit_string += struct.pack('H', num)
    return bit_string

def send(arduino, point_list):
    if len(point_list) != (len(point_list)/(MAX_NUM_SEND/2))*(MAX_NUM_SEND/2):
        point_list += [point_list[-1]] * ((len(point_list)/(MAX_NUM_SEND/2) + 1)*(MAX_NUM_SEND/2) - len(point_list))

    numlist = points_to_shortlist(point_list)

    for i in range(0, len(numlist), MAX_NUM_SEND):
        sublist = numlist[i:i+MAX_NUM_SEND]
        bit_string = sublist_to_bstring(sublist)
        data = ''
        print 'waiting for ready...'
        while data != 'ready':
            data = arduino.readline()[:-2]
            # time.sleep(READY_MISS)
        print 'ready recieved'
        # print 'data'
        # print bit_string
        print 'writting points ' + str(i) + ' to ' + str(i+MAX_NUM_SEND)
        arduino.write(bit_string)
        print 'wrote points'
        # time.sleep(READY_HIT)
    print 'done'

def main():
    #points = []
    #for i in range(0,1000):
    #    points.append((i,i))

    points = [(100,124),(212,364),(212,1220),(100,1500),(532,1500),(820,844),
    (1108,1500),(1532,1500),(1428,1220),(1428,364),(1540,124),(1092,124),
    (1092,636),(916,236),(724,236),(548,636),(548,124),(100,124)]

    ser = serial.Serial(ARDUINO_ADDR,BAUD_R)
    time.sleep(READY_HIT)

    send(ser,points)

if __name__ == '__main__':
    main()
