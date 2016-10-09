"""Test XBee sending messages at pre-defined speed."""

import serial
import struct
import time

SLEEP_SECONDS = 0.01

s = serial.Serial('/dev/cu.usbserial-A600e0ti', baudrate=19200)
i = 64

try:
    print 'Running'
    while True:
        s.write(struct.pack('=B', i))
        time.sleep(SLEEP_SECONDS)
        i += 1

        if i > 127:
            i = 64
except KeyboardInterrupt:
    print 'Stopping'
    s.write(struct.pack('=B', 91))
