
/***************************************************************************************
*    Code from the following sources has been used in this project:
*
*    Title: Pose estimation excercise (Lecture 04)
*    Author: Anders Glent 
*
***************************************************************************************/
#include "localization.h"

Localization::Localization()
{
}

Localization::~Localization()
{
}

/**
 * Update the RANSAC and ICP  visualizations
 *
 * @param cloud_1: Pointer to the pointcloud with the object.
 * @param cloud_2: Pointer to the pointcloud with the scene.
 * @param enabled:   Bollean controling if the visualization is active or not.
 * @param option: visualization to be updated.
 *                  - 1 : RANSAC
 *                  - 2 : ICP
 */

void Localization::step_visualization_UPDATE(Point_Cloud_N::Ptr &cloud_1, Point_Cloud_N::Ptr &cloud_2, bool enabled, int option)
{
    if (enabled)
    {
        switch (option)
        {
        case 1:
            // TODO: Remove double pointer, It works now, but probably it can be done better
            if (this->it_best_model != 0)
            {
                (**v_global_alignment).removePointCloud(std::to_string(this->it_best_model_previous));
                (**v_global_alignment).addPointCloud<pcl::PointNormal>(cloud_1, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_1, 0, 255, 0), std::to_string(it_best_model));
                (**v_global_alignment).spinOnce();
            }
            else
            {
                (**v_global_alignment).setCameraPosition(-0.252034, 0.341164, -0.57032, -0.128131, 0.122338, 0.0488766, -0.0385026, -0.94455, -0.326102);
                (**v_global_alignment).addPointCloud<pcl::PointNormal>(cloud_2, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_2, 255, 0, 0), "Cloud_2");
                (**v_global_alignment).spinOnce();
            }

            break;

        case 2:

            if (it_local_alignment != 0)
            {
                (**v_local_alignment).removePointCloud(std::to_string(it_local_alignment - 1));
                (**v_local_alignment).addPointCloud<pcl::PointNormal>(cloud_1, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_1, 0, 255, 0), std::to_string(it_local_alignment));
                (**v_local_alignment).spinOnce();
            }
            else
            {
                (**v_local_alignment).setCameraPosition(-0.252034, 0.341164, -0.57032, -0.128131, 0.122338, 0.0488766, -0.0385026, -0.94455, -0.326102);
                (**v_local_alignment).addPointCloud<pcl::PointNormal>(cloud_2, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_2, 255, 0, 0), "Cloud_2");
                (**v_local_alignment).spinOnce();
            }
            break;
        }
    }
}

/**
 * Applies a global alignment using SPIN features
 *
 * @param scene:  Pointer to the pointcloud with the scene.
 * @param object: Pointer to the pointcloud with the object.
 * @param T:      Transformation where the pose estimation should be saved.
 * @param iter:   Number of iterations for the RANSAC procedure.
 * @param visualize: Bollean controling if the visualization is active or not.
 */

void Localization::global_alignment(Point_Cloud_N::Ptr &scene, Point_Cloud_N::Ptr &object, Eigen::Matrix4f &T, const size_t iter, bool visualize)
{
    // RANSAC parameters:
    const float thressq = 0.005 * 0.005;
    const int ransac_model_min_points = 3;
    float min_inliers = 0.3 * object->size();
    // SPIN parameters:
    float spin_radius = 0.2;
    // Nearest feature matching:
    float threshold_nearest = 0.02;

    std::cout << "|--> Global Alignment." << std::endl;
    std::cout << "|  |--> Function parameters:" << std::endl;
    std::cout << "|  |  |--> RANSAC:" << std::endl;
    std::cout << "|  |  |  |--> Num of iterations             [ " << iter << " ]" << std::endl;
    std::cout << "|  |  |  |--> Inliers threshold             [ " << thressq << " ]" << std::endl;
    std::cout << "|  |  |  |--> Min Model points              [ " << ransac_model_min_points << " ]" << std::endl;
    std::cout << "|  |  |  |--> Min num of inliers            [ " << min_inliers << " ]" << std::endl;
    std::cout << "|  |  |--> SPIN Features:" << std::endl;
    std::cout << "|  |  |  |--> Radius                        [ " << spin_radius << " ]" << std::endl;
    std::cout << "|  |  |--> Nearest Features matching:" << std::endl;
    std::cout << "|  |     |--> Threshod                      [ " << threshold_nearest << " ]" << std::endl;

    std::cout << "|  |--> Compute SPIN features for both the Scene and the Object." << std::endl;
    pcl::PointCloud<FeatureT>::Ptr object_features(new pcl::PointCloud<FeatureT>);
    pcl::PointCloud<FeatureT>::Ptr scene_features(new pcl::PointCloud<FeatureT>);

    std::cout << "|  |  |--> Create SPIN estimation object." << std::endl;
    pcl::SpinImageEstimation<pcl::PointNormal, pcl::PointNormal, FeatureT> spin;
    spin.setRadiusSearch(spin_radius);

    spin.setInputCloud(object);
    spin.setInputNormals(object);
    spin.compute(*object_features);
    std::cout << "|  |  |--> Object features computed." << std::endl;
    spin.setInputCloud(scene);
    spin.setInputNormals(scene);
    spin.compute(*scene_features);
    std::cout << "|  |  |--> Scene features computed." << std::endl;

    std::cout << "|  |--> Find correspondences between the two poinclouds." << std::endl;
    pcl::Correspondences corr(object_features->size());
    std::cout << "|  |  |--> Object Size                      [ " << object_features->size() << " ]" << std::endl;

    for (size_t i = 0; i < object_features->size(); ++i)
    {
        corr[i].index_query = i;
        nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance, threshold_nearest);
    }

    std::cout << "|  |  |--> Threshod                         [ " << threshold_nearest << " ]" << std::endl;
    std::cout << "|  |  |--> Minimum distance                 [ " << min_dist_nearest << " ]" << std::endl;
    std::cout << "|  |  |--> Maximum distance                 [ " << max_dist_nearest << " ]" << std::endl;
    std::cout << "|  |  |--> Average distance                 [ " << total_dist_nearest / num_of_accepted_matches << " ]" << std::endl;
    std::cout << "|  |  |--> Number of matches accepted       [ " << num_of_accepted_matches << " ]" << std::endl;

    utilities.step_visualization(scene, object, corr, "Feature matching", visualize);

    std::cout << "|  |-->  RANSAC for Global alignemnt." << std::endl;
    std::cout << "|  |  |--> Create KdTree for the scene." << std::endl;
    pcl::search::KdTree<pcl::PointNormal> tree;
    tree.setInputCloud(scene);

    std::cout << "|  |  |--> Create Pointcloud and T matrix to save the best model." << std::endl;
    Point_Cloud_N::Ptr object_aligned(new Point_Cloud_N);
    Eigen::Matrix4f T_RANSAC;

    float penalty = FLT_MAX;
    size_t inliers_i = 0;
    size_t inliers = 0;
    size_t i = 1;
    float rmse = 0;
    float rmse_i = 0;

    std::cout << "|  |  |--> Create randomizer to sample correspondences." << std::endl;
    pcl::common::UniformGenerator<int> gen(0, corr.size() - 1, time(NULL));

    // TODO: Double pointer, It works now, but probably it can be done better
    v_global_alignment = new pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Global Alignment"));

    std::cout << "|  |  |--> Start RANSAC." << std::endl;

    while (inliers_i < min_inliers && i < iter)
    {
        utilities.progressbar(i, iter);

        std::vector<int> idxobj(ransac_model_min_points);
        std::vector<int> idxscn(ransac_model_min_points);

        // Sample N accepted matches
        for (int j = 0; j < ransac_model_min_points; ++j)
        {
            int idx = gen.run();

            while (corr[idx].index_match == 1)
            {
                idx = gen.run();
            }
            idxobj[j] = corr[idx].index_query;
            idxscn[j] = corr[idx].index_match;
        }

        // Estimate transformation
        pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal> est;
        est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T_RANSAC);

        // Apply T
        transformPointCloud(*object, *object_aligned, T_RANSAC);

        // Validate
        std::vector<std::vector<int>> idx;
        std::vector<std::vector<float>> distsq;
        tree.nearestKSearch(*object_aligned, std::vector<int>(), 1, idx, distsq);

        // Compute inliers and RMSE
        inliers_i = 0;
        rmse_i = 0;
        for (size_t j = 0; j < distsq.size(); ++j)
            if (distsq[j][0] <= thressq)
                ++inliers_i, rmse_i += distsq[j][0];
        rmse_i = sqrtf(rmse_i / inliers_i);

        // Evaluate a penalty function
        const float outlier_rate = 1.0f - float(inliers_i) / object->size();
        //const float penaltyi =  rmse_i ;
        const float penaltyi = outlier_rate;

        // Update result
        if (penaltyi < penalty)
        {
            transformPointCloud(*object, *object_aligned, T_RANSAC);
            step_visualization_UPDATE(object_aligned, scene, visualize, 1);

            //TODO: Revise if it can be done more efficiently
            if (this->it_best_model == 0)
            {
                this->it_best_model_previous = 0;
                this->it_best_model = i;
            }
            else
            {
                this->it_best_model_previous = it_best_model;
                this->it_best_model = i;
            }

            penalty = penaltyi;
            T = T_RANSAC;
            inliers = inliers_i;
            rmse = rmse_i;
        }
        i = i + 1;
    }
    std::cout << "|  |  |  |" << std::endl;
    std::cout << "|  |  |  |--> Inliers                       [ " << inliers << "/" << object->size() << " ]" << std::endl;
    std::cout << "|  |  |  |--> RMSE                          [ " << rmse << " ]" << std::endl;
    std::cout << "|  |  |--> Update object pointcloud with the best T estimated." << std::endl;

    transformPointCloud(*object, *object, T);
}

/**
 * Applies a local alignment using the ICP algorithm.
 *
 * @param scene:  Pointer to the pointcloud with the scene.
 * @param object: Pointer to the pointcloud with the object.
 * @param T:      Transformation where the pose estimation should be saved.
 * @param iter:   Number of iterations for the RANSAC procedure.
 * @param visualize: Bollean controling if the visualization is active or not.
 */

void Localization::local_alignment(Point_Cloud_N::Ptr &scene, Point_Cloud_N::Ptr &object, Eigen::Matrix4f &T, const size_t &iter, bool visualize)
{

    // ICP arameters:
    const float thressq = 0.01 * 0.01;

    std::cout << "|--> Local Alignment." << std::endl;
    std::cout << "|  |--> Function parameters:" << std::endl;
    std::cout << "|  |  |--> ICP:" << std::endl;
    std::cout << "|  |  |  |--> Num of iterations             [ " << iter << " ]" << std::endl;
    std::cout << "|  |  |  |--> Inliers threshold             [ " << thressq << " ]" << std::endl;

    std::cout << "|  |--> Create KdTree for the scene." << std::endl;
    pcl::search::KdTree<pcl::PointNormal> tree;
    tree.setInputCloud(scene);

    // TODO: Double pointer, It works now, but probably it can be done better
    v_local_alignment = new pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Local alignment"));
    size_t inliers = 0;
    float rmse = FLT_MAX;
    float angle_between_NN = 0.0;
    Eigen::Vector4f n;
    Eigen::Vector4f m;

    std::cout << "|  |--> ICP for local alignment." << std::endl;
    for (size_t i = 1; i < iter; ++i)
    {
        utilities.progressbar(i, iter);

        // 1) Find closest points
        std::vector<std::vector<int>> idx;
        std::vector<std::vector<float>> distsq;
        tree.nearestKSearch(*object, std::vector<int>(), 1, idx, distsq);

        // Threshold and create indices for object/scene and compute RMSE
        std::vector<int> idxobj;
        std::vector<int> idxscn;

        prep.compute_normals(object, object, 1, 3);

        for (size_t j = 0; j < idx.size(); ++j)
        {
            if (distsq[j][0] <= thressq)
            {
                n = object->at(j).getNormalVector4fMap();
                m = scene->at(idx[j][0]).getNormalVector4fMap();

                angle_between_NN = pcl::getAngle3D(m, n, true);
                if (angle_between_NN < 40 || angle_between_NN > 320)
                {
                    idxobj.push_back(j);
                    idxscn.push_back(idx[j][0]);
                }
            }
        }

        // 2) Estimate transformation
        Eigen::Matrix4f T_ICP;
        pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal> est;
        est.estimateRigidTransformation(*object, idxobj, *scene, idxscn, T_ICP);

        // 3) Apply T
        transformPointCloud(*object, *object, T_ICP);

        // 4) Update result
        T = T_ICP * T;

        step_visualization_UPDATE(object, scene, visualize, 2);
        this->it_local_alignment = i;

        // Compute inliers and RMSE
        std::vector<std::vector<int>> idx2;
        std::vector<std::vector<float>> distsq2;
        tree.nearestKSearch(*object, std::vector<int>(), 1, idx2, distsq2);
        size_t inliers_i = 0;
        float rmse_i = 0;
        for (size_t i = 0; i < distsq.size(); ++i)
            if (distsq[i][0] <= thressq)
                ++inliers_i, rmse_i += distsq[i][0];
        rmse_i = sqrtf(rmse_i / inliers_i);

        if (inliers_i > inliers)
            inliers = inliers_i;

        if (std::abs(rmse_i - rmse) < 0.00000001 && i != 1)
        {
            std::cout << "|  |  |" << std::endl;
            std::cout << "|  |  |" << rmse_i << "   " << rmse << "   " << std::endl;
            break;
        }
        else
        {
            rmse = rmse_i;
        }
    }

    std::cout << "|  |  |" << std::endl;
    std::cout << "|  |  |--> Inliers                          [ " << inliers << "/" << object->size() << " ]" << std::endl;
    std::cout << "|  |  |--> RMSE                             [ " << rmse << " ]" << std::endl;
    std::cout << "|  |--> Update object pointcloud with the best T estimated." << std::endl;
}

/**
 * Computes the nearest feature for feature matching between two pointclouds.
 *
 * @param query:  Feature to be matched.
 * @param target: Object containing all the features from the pointcloud to be matched.
 * @param idx:      
 * @param distsq:  distance between the features matched.
 * @param threshold: Matches which are further than the threshold, won't be considered as good matches..
 */

void Localization::nearest_feature(const FeatureT &query, const pcl::PointCloud<FeatureT> &target, int &idx, float &distsq, float threshold)
{
    idx = 0;
    distsq = dist_sq(query, target[0]);

    //std::cout << "Target:" << target.size() << std::endl;
    for (size_t i = 1; i < target.size(); ++i)
    {
        const float disti = dist_sq(query, target[i]);
        if (disti < distsq)
        {
            idx = i;
            distsq = disti;
        }
    }
    //std::cout << threshold << std::endl;
    if (distsq > threshold)
    {
        idx = 1;
    }
    else
    {
        //std::cout << "Matching accepted: " << distsq << std::endl;
        if (this->min_dist_nearest > distsq)
        {
            this->min_dist_nearest = distsq;
        }

        if (this->max_dist_nearest < distsq)
        {
            this->max_dist_nearest = distsq;
        }

        this->num_of_accepted_matches++;
        this->total_dist_nearest += distsq;
    }
}

/**
 * Computes the distance between two features.
 *
 * @param query:  FeatureT one.
 * @param target: FeatureT two.
 */

inline float Localization::dist_sq(const FeatureT &query, const FeatureT &target)
{
    float result = 0.0;
    for (int i = 0; i < FeatureT::descriptorSize(); ++i)
    {
        const float diff = reinterpret_cast<const float *>(&query)[i] - reinterpret_cast<const float *>(&target)[i];
        result += diff * diff;
    }

    return result;
}