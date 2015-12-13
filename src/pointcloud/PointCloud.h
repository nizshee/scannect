#pragma once

#include <iostream>

#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

class PointCloud {
private:
    Eigen::Matrix4f init;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;

public:
    PointCloud();
    PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr&);
    void add_point(double x, double y, double z);
    double get_x(unsigned int num);
    double get_y(unsigned int num);
    double get_z(unsigned int num);
    unsigned int get_size();
    PointCloud concat(PointCloud&);
    PointCloud normal(char*, double);
    PointCloud statistical_outlier_removal(int mean, double dev);
    PointCloud approximate_voxel_grid(double, double, double);
    PointCloud icp(int, double, double, double, PointCloud&);
    PointCloud ndt(int, double, double, double, PointCloud&);
    void read_from_file(char* filename);
    void write_to_file(char* filename);
};