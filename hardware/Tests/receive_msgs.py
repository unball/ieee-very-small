import serial
import struct

s = serial.Serial('/dev/cu.usbserial-A600e0ti', baudrate=19200)

def receive_msg():
    print 'Receiving messages'
    message = []
    c = s.read()
    while c != '\n':
        message.append(c)
    message = ''.join(message)
    print 'Msg:', message
