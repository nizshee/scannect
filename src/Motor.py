#! /usr/bin/python3

import serial

class Motor:

    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)

    def turn(self, angle):
        ser = self.ser
        ser.write((str(angle) + '\r\n').encode('ascii'))
        response = int(ser.readline().decode('ascii'))
        ser.readline().decode('ascii')
        if response != angle:
            return False
        return True

