#include "PointCloud.h"


PointCloud::PointCloud() {
    point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
}

PointCloud::PointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& ptr) {
    point_cloud = ptr;
}

void PointCloud::add_point(double x, double y, double z) {
    pcl::PointXYZ o;
    o.x = x;
    o.y = y;
    o.z = z;
    point_cloud->push_back(o);
}

double PointCloud::get_x(unsigned int num) {
    return point_cloud->at(num).x;
}

double PointCloud::get_y(unsigned int num) {
    return point_cloud->at(num).y;
}

double PointCloud::get_z(unsigned int num) {
    return point_cloud->at(num).z;
}

unsigned int PointCloud::get_size() {
    return point_cloud->size();
}

PointCloud& PointCloud::statistical_outlier_removal(int mean, double dev) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (point_cloud);
    sor.setMeanK (mean);
    sor.setStddevMulThresh (dev);
    sor.filter (*cloud_filtered);
    PointCloud* pc = new PointCloud(cloud_filtered);
    return *pc;
}

PointCloud& PointCloud::approximate_voxel_grid(double x, double y , double z) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setLeafSize (x, y, z);
    voxel_filter.setInputCloud (point_cloud);
    voxel_filter.filter (*cloud_filtered);
    PointCloud* pc = new PointCloud(cloud_filtered);
    return *pc;   
}

PointCloud& PointCloud::icp(int i, double d1, double d2, double d3, PointCloud& source) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
    registration.setInputSource(source.point_cloud);
    registration.setInputTarget(point_cloud);
    registration.setMaximumIterations(i);
    registration.setMaxCorrespondenceDistance (d1);
    registration.setEuclideanFitnessEpsilon (d2);
    registration.setTransformationEpsilon (d3);
    registration.align(*cloud_result);

    if(registration.hasConverged()) std::cout << "success" << std::endl;
    else std::cout << "fail" << std::endl;
    
    PointCloud* pc = new PointCloud(cloud_result);
    return *pc;
}

PointCloud& PointCloud::ndt(int i, double d1, double d2, double d3, PointCloud& source) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> registration;
    registration.setInputSource(source.point_cloud);
    registration.setInputTarget(point_cloud);
    registration.setMaximumIterations (i);
    registration.setStepSize (d1);
    registration.setResolution (d2);
    registration.setTransformationEpsilon (d3);
    registration.align(*cloud_result);

    if(registration.hasConverged()) std::cout << "success" << std::endl;
    else std::cout << "fail" << std::endl;

    PointCloud* pc = new PointCloud(cloud_result);
    return *pc;   
}

void PointCloud::read_from_file(char* filename) {
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ> (filename, *point_cloud);
}

void PointCloud::write_to_file(char* filename) {
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (filename, *point_cloud, false);
}

