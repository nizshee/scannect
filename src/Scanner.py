#! /usr/bin/python3

from KinectWrapper import Device
from Motor import Motor

motor = Motor()

dev = Device(0)
dev.open()

step = 2
for angle in range(0, 360, step):
    filename = "../data/box{:03d}.pcd".format(angle)
    print(filename)
    dev.start(50) 
    dev.save(filename)
    motor.turn(-step)

dev.stop()


# print("../data/box{:02d}.pcd".format(angle))

# for i in dev.get_width():
#     for j in dev.get_heigth():
#         pc.add(())

