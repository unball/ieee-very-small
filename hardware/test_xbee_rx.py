"""Measure how many messages per second the XBee is sending."""

from __future__ import division
import serial
import struct
import time
import datetime

RESOLUTION = 1
s = serial.Serial('/dev/cu.usbserial-A600e0ti', baudrate=19200)
rcv_msgs = 0

print 'Started'

prev_time = datetime.datetime.now()

while True:
    msg = s.read()
    rcv_msgs += 1
    current_time = datetime.datetime.now()
    diff_time = current_time - prev_time

    if (diff_time.seconds == RESOLUTION):
        print '%.1f msgs/s' %(rcv_msgs/RESOLUTION)
        rcv_msgs = 0
        prev_time = current_time
