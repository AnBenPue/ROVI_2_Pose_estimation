#ifndef UTILS_H
#define UTILS_H

// PCL headers
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// Robwork headers
#include <rw/math/Vector3D.hpp>
#include "rw/rw.hpp"

// Std headers
#include <iostream>
#include <iomanip>
#include <string>

typedef pcl::PointCloud<pcl::PointNormal> Point_Cloud_N;
typedef pcl::PointCloud<pcl::PointXYZ> Point_Cloud_XYZ;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Point_Cloud_XYZRGBA;

class Utils
{
private:
public:
    Utils();
    ~Utils();

    void step_visualization(Point_Cloud_N::Ptr &cloud_1, Point_Cloud_N::Ptr &cloud_2, std::string step_name, bool enabled);
    void step_visualization(Point_Cloud_XYZ::Ptr &cloud_1, Point_Cloud_XYZ::Ptr &cloud_2, std::string step_name, bool enabled);
    void step_visualization(Point_Cloud_XYZ::Ptr &cloud_1, Point_Cloud_N::Ptr &cloud_2, std::string step_name, bool enabled);
    void step_visualization(Point_Cloud_N::Ptr &cloud_1, Point_Cloud_N::Ptr &cloud_2, pcl::Correspondences corr, std::string step_name, bool enabled);
    void display_result_RGB(Point_Cloud_N::Ptr &object, Point_Cloud_XYZRGBA::Ptr &scene, Eigen::Matrix4f T);

    void save_data(std::string filepath, double d1, double d2, double d3, double d4, double d5, double d6);
    void save_data(std::string filepath, double d1, double d2, double d3, double d4, double d5, double d6, double d7, double d8, double d9);

    void load_object_of_interest(Point_Cloud_N::Ptr &object, std::string path);

    float vectors_angle(Eigen::Matrix4f T, pcl::ModelCoefficients::Ptr coefficients_plane);
    void progressbar(int current, int total);
};
#endif