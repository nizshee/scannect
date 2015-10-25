#include <iostream>
#include "PointCloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    return 0;
    // // Objects for storing the point clouds.
    // pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZ>);
 
    // // Read two PCD files from disk.
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *sourceCloud) != 0)
    // {
    //  return -1;
    // }
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *targetCloud) != 0)
    // {
    //  return -1;
    // }
 
    // // ICP object.
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> registration;
    // registration.setInputSource(sourceCloud);
    // registration.setInputTarget(targetCloud);
    // registration.setMaxCorrespondenceDistance (0.05);
    // registration.setMaximumIterations(1000);
    // registration.setTransformationEpsilon (1e-8);
    // // registration.setEuclideanFitnessEpsilon (1);
 
    // registration.align(*finalCloud);
    // if (registration.hasConverged())
    // {
    //  pcl::io::savePCDFileASCII (argv[3], *finalCloud);
    //  std::cout << "ICP converged." << std::endl
    //            << "The score is " << registration.getFitnessScore() << std::endl;
    //  // std::cout << "Transformation matrix:" << std::endl;
    //  // std::cout << registration.getFinalTransformation() << std::endl;
    // }
    // else std::cout << "ICP did not converge." << std::endl;
}