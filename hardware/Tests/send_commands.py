import serial
import struct

s = serial.Serial('/dev/cu.usbserial-A600e0ti', baudrate=19200)

while True:
    data_to_write = struct.pack('=B', input('>> '))
    print 'Data:', data_to_write
    s.write(data_to_write)
