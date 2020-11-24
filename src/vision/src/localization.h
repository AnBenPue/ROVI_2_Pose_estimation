#ifndef LOCALIZATION_H
#define LOCALIZATION_H

// PCL headers
#include <pcl/common/random.h>
#include <pcl/common/transforms.h>
#include <pcl/features/spin_image.h>
#include <pcl/registration/transformation_estimation_svd.h>

// std headers
#include <iostream>
#include <iomanip>
#include <string>

// Project custom headers
#include "Utils.h"
#include "PreProcessing.h"

typedef pcl::Histogram<153> FeatureT;

class Localization
{
private:
    // TODO: Double pointer, It works now, but probably it can be done better
    pcl::visualization::PCLVisualizer::Ptr *v_global_alignment;
    pcl::visualization::PCLVisualizer::Ptr *v_local_alignment;

    inline float dist_sq(const FeatureT &query, const FeatureT &target);
    void nearest_feature(const FeatureT &query, const pcl::PointCloud<FeatureT> &target, int &idx, float &distsq, float threshold);

    void step_visualization_UPDATE(Point_Cloud_N::Ptr &cloud_1, Point_Cloud_N::Ptr &cloud_2, bool enabled, int option);

    Utils utilities;
    PreProcessing prep;

    float min_dist_nearest = FLT_MAX;
    float max_dist_nearest = FLT_MIN;
    float num_of_accepted_matches = 0.0;
    float total_dist_nearest = 0.0;

    int it_best_model_previous = 0;
    int it_best_model = 0;
    int it_local_alignment = 0;

public:
    Localization();
    ~Localization();

    void global_alignment(Point_Cloud_N::Ptr &scene, Point_Cloud_N::Ptr &object, Eigen::Matrix4f &T, const size_t iter, bool visualize);
    void local_alignment(Point_Cloud_N::Ptr &scene, Point_Cloud_N::Ptr &object, Eigen::Matrix4f &T, const size_t &iter, bool visualize);
};

#endif