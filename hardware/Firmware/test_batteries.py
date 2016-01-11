import serial
import struct
import time

s = serial.Serial('/dev/cu.usbserial-A600e0ti', baudrate=19200)

def create_message(robot, speed_left, speed_right):
    message = (robot << 6) | (speed_left << 3) | speed_right
    return message

def pack_byte(byte):
    return struct.pack('=B', byte)

def send_message(message):
    s.write(pack_byte(message))

if __name__ == '__main__':
    num_robots = 3
    sleep_time = 3
    min_speed = 2
    max_speed = 7
    speed = 0
    start_time = time.time()

    try:
        while True:
            for i in range(num_robots):
                message = create_message(i, speed, speed)
                print 'Message: {0:08b} {1}'.format(message, message)
                send_message(message)

            current_time = time.time() - start_time
            hours, rest = divmod(current_time, 3600)
            minutes, seconds = divmod(rest, 60)
            print '{0:02d}:{1:02d}:{2:02d}'.format(int(hours), int(minutes), int(seconds))
            time.sleep(sleep_time)

            speed += 1
            if speed > max_speed:
                speed = min_speed

    except KeyboardInterrupt:
        for i in range(num_robots):
            message = create_message(i, 3, 3)
            print 'Message: {0:08b} {1}'.format(message, message)
            send_message(message)

        current_time = time.time() - start_time
        hours, rest = divmod(current_time, 3600)
        minutes, seconds = divmod(rest, 60)
        print 'Final time: {0:02d}:{1:02d}:{2:02d}'.format(int(hours), int(minutes), int(seconds))
