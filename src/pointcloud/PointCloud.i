
%module PointCloudWrapper

%{
#define SWIG_FILE_WITH_INIT
#include "PointCloud.h"
%}

class PointCloud {
private:

public:
    PointCloud();
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