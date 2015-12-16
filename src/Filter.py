#! /usr/bin/python3

from PointCloud import PointCloud

start = 0
stop = 360
step = 40


# pc_next = PointCloud.read_file("../data/box005.pcd")
# pc_next = pc_next.static(mean=50, dev=1e-1)

concat = PointCloud()
for angle in range(start, stop, step):
    filename = "../data/processed/box{:03d}.pcd".format(angle)
    print(filename)
    pc = PointCloud.read_file(filename)
    concat = concat.concat(pc.voxel(leaf_x=5, leaf_y=5, leaf_z=5))
# concat = concat.static(mean=200, dev=0.5)
# concat = concat.voxel(leaf_x=15, leaf_y=15, leaf_z=15)
concat.write("../data/processed/aaa.pcd")

concat.static(mean=30, dev=3).voxel(leaf_x=7, leaf_y=7, leaf_z=7).write_normal("../data/processed/aa.pcd")