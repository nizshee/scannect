#! /usr/bin/python3

from PointCloud import PointCloud

start = 0
stop = 360
step = 10

pc_curr = PointCloud.read_file("../data/box{:03d}.pcd".format(start))
pc_curr = pc_curr.static(mean=50, dev=1e-1)
pc_curr.write("../data/processed/box{:03d}.pcd".format(start))

# pc_next = PointCloud.read_file("../data/box005.pcd")
# pc_next = pc_next.static(mean=50, dev=1e-1)

for angle in range(start + step, stop, step):
    file_fst = "../data/box{:03d}.pcd".format(angle)
    file_snd = "../data/processed/box{:03d}.pcd".format(angle)
    print(file_fst)
    pc_next = PointCloud.read_file(file_fst)
    pc_next = pc_next.static(mean=50, dev=1e-1)
    pc_curr = pc_curr.ndt(pc_next, iterations=100, step=10, resol=5.0, trans=0.08)
    pc_curr.write(file_snd)

# 24 26
# 36 38
# 28 30


# pc_next = pc_curr.ndt(pc_next, iterations=100, step=10, resol=10.0, trans=0.1)

    

# pc1 = pc1.voxel(leaf_x=0.3, leaf_y=0.3, leaf_z=0.3)
# pc2 = pc2.voxel(leaf_x=0.3, leaf_y=0.3, leaf_z=0.3)



# pc_next.write("../data/processed/box005.pcd")
