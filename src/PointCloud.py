#! /usr/bin/python3

import PointCloudWrapper as pcw

class PointCloud:

    def __init__(self, *pc):
        if pc:
            self.pc = pc[0]
        else:
            self.pc = pcw.PointCloud()

    def size(self):
        return self.pc.get_size()

    def add(self, point):
        x, y, z = point
        self.pc.add_point(x, y, z)

    def get(self, num):
        x = self.pc.get_x(num)
        y = self.pc.get_y(num)
        z = self.pc.get_z(num)
        return (x, y, z)

    def static(self, mean=50, dev=1.):
        tmp = self.pc.statistical_outlier_removal(mean, dev)
        return PointCloud(tmp)

    def voxel(self, leaf_x=0.1, leaf_y=0.1, leaf_z=0.1):
        tmp = self.pc.approximate_voxel_grid(leaf_x, leaf_y, leaf_z)
        return PointCloud(tmp)

    def icp(self, source, iterations=1000, dist=0.05, trans=1e-8, fitness=1.):
        tmp = self.pc.icp(iterations, dist, fitness, trans, source.pc)
        return PointCloud(tmp)

    def ndt(self, source, iterations=35, step=0.01, resol=1.0, trans=0.01):
        tmp = self.pc.ndt(iterations, step, resol, trans, source.pc)
        return PointCloud(tmp)

    def write_normal(self, filename, radius=10):
        self.pc.normal(filename, radius)

    def read(self, filename):
        self.pc.read_from_file(filename)

    def concat(self, pc):
        return PointCloud(self.pc.concat(pc.pc))

    @classmethod
    def read_file(cls, filename):
        tmp = PointCloud(pcw.PointCloud())
        tmp.read(filename)
        return tmp

    def write(self, filename):
        self.pc.write_to_file(filename)

# pc1 = PointCloud.read_file("../../data/file00.pcd")
# pc2 = PointCloud.read_file("../../data/file01.pcd")

# pc1 = pc1.voxel(0.0001, 0.0001, 0.0001)
# pc2 = pc2.voxel(0.0001, 0.0001, 0.0001)
# pc1 = pc1.static(dev=0.1)
# pc2 = pc2.static(dev=0.1)
# pc1.write("../../data/target.pcd")

# pc3 = pc1.ndt(pc2, iterations=100, step=0.0001, resol=0.0005, trans=0.0001)
# pc3.write("../../data/result.pcd")

# pc.read("../../data/file01.pcd")

# pc = pc.removal(mean=20, dev=0.1)
# pc = pc.grid()
# pc.write("../../data/file26new.pcd")

# print(pc.size())
# for i in range(pc.size()):
#     print(pc.get(i))
