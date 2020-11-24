#ifndef PreProcessing_H
#define PreProcessing_H

// PCL headers
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/mls.h>

// std headers
#include <iostream>
#include <iomanip>
#include <string>

// Project custom headers
#include "Utils.h"

typedef pcl::PointCloud<pcl::PointNormal> Point_Cloud_N;
typedef pcl::PointCloud<pcl::PointXYZ> Point_Cloud_XYZ;

class PreProcessing
{
private:
    Utils utilities;
    void remove_points(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, pcl::PointIndices::Ptr &inliers_plane, bool visualize);

public:
    PreProcessing();
    ~PreProcessing();

    void Filter_XYZ(Point_Cloud_XYZ::Ptr &cloud_in, Point_Cloud_XYZ::Ptr &cloud_out, bool visualize);
    void fit_plane(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, pcl::ModelCoefficients::Ptr coefficients_plane, bool visualize);
    void remove_small_clusters(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, bool visualize);
    void smoothing_cloud(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, bool visualize);
    void smoothing_cloud(Point_Cloud_XYZ::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, bool visualize);
    void downsample_pointcloud(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, float leaf_size, bool visualize);
    void compute_normals(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, int method, double magnitude);
    void compute_normals(Point_Cloud_N::Ptr &cloud_in, pcl::PointCloud<pcl::Normal>::Ptr &cloud_out, int method, double magnitude);
};

#endif