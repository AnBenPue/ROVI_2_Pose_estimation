/***************************************************************************************
*    Code from the following sources has been used in this project:
*
*    Title: Filtering a PointCloud using a PassThrough filter
*    Availability: http://pointclouds.org/documentation/tutorials/passthrough.php
*
*    Title: Cylinder model segmentation
*    Availability: http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php
*
*    Title: Euclidean Cluster Extraction
*    Availability: http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
*
*    Title: Smoothing and normal estimation based on polynomial reconstruction
*    Availability: http://pointclouds.org/documentation/tutorials/resampling.php
*
***************************************************************************************/

#include "PreProcessing.h"

PreProcessing::PreProcessing()
{
    //Point_Cloud_XYZ::Ptr filtered_xyz = Filter_XYZ()
}

PreProcessing::~PreProcessing()
{
}

/**
 * Applies a passthrough filter to a pointcloud
 *
 * @param cloud_in:  Pointer to the pointcloud to be filtered.
 * @param cloud_out: Pointer to the pointcloud in which the result should be saved.
 * @param visualize: Bollean controling if the visualization is active or not.
 */

void PreProcessing::Filter_XYZ(Point_Cloud_XYZ::Ptr &cloud_in, Point_Cloud_XYZ::Ptr &cloud_out, bool visualize)
{
    std::cout << "|--> Pass trough filter in XYZ:" << std::endl;
    std::cout << "|  |--> Point cloud has " << cloud_in->points.size() << " data points." << std::endl;
    utilities.step_visualization(cloud_in, cloud_in, "Pass trough filter in XYZ: Original Cloud", visualize);

    // Parameters to modify the behaviour of this program:
    // Filter Treshold values:        X         Y          Z
    std::vector<float> lim = {-0.3, 0.1, -0.2, 0.1, 0, 0.7};

    // Variables declaration>
    Point_Cloud_XYZ::Ptr cloud_pass_z(new Point_Cloud_XYZ);
    Point_Cloud_XYZ::Ptr cloud_pass_y(new Point_Cloud_XYZ);

    //Filtering in Z:
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud_in);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(lim[4], lim[5]);
    pass_z.filter(*cloud_pass_z);
    std::cout << "|  |--> Filtering in Z, between:          [ " << lim[4] << " , " << lim[5] << " ] " << std::endl;
    std::cout << "|  |  |--> Point cloud has                [ " << cloud_pass_z->points.size() << " ] data points." << std::endl;
    utilities.step_visualization(cloud_in, cloud_pass_z, "Pass trough filter in XYZ: Z filtered", visualize);

    //Filtering in Y:
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(cloud_pass_z);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(lim[2], lim[3]);
    pass_y.filter(*cloud_pass_y);

    std::cout << "|  |--> Filtering in Y, between:          [ " << lim[2] << " , " << lim[3] << " ] " << std::endl;
    std::cout << "|  |  |--> Point cloud has                [ " << cloud_pass_y->points.size() << " ] data points." << std::endl;
    utilities.step_visualization(cloud_pass_z, cloud_pass_y, "Pass trough filter in XYZ: Y filtered", visualize);

    //Filtering in X:
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(cloud_pass_y);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(lim[0], lim[1]);
    pass_x.filter(*cloud_out);

    std::cout << "|  |--> Filtering in X, between:          [ " << lim[0] << " , " << lim[1] << " ] " << std::endl;
    std::cout << "|     |--> Point cloud has                [ " << cloud_out->points.size() << " ] data points." << std::endl;
    utilities.step_visualization(cloud_pass_y, cloud_out, "Pass trough filter in XYZ: X filtered", visualize);
}

/**
 * (1/2) Computes the normal for each point of a pointcloud
 *
 * @param cloud_in:  Pointer to the pointcloud to compute the normals.
 * @param cloud_out: Pointer to the pointcloud in which the result should be saved.
 * @param method:    Method selected for the nearenes neighbour selection for the normal computation.
 *                  - 1 : setKSearch()
 *                  - 2 : setRadiusSearch()
 * @param magnitude: Magnitude for the method selected:
*                   - method 1 : number of neighbours to be selected
 *                  - method 2 : radius of the neighbourhood 
 */

void PreProcessing::compute_normals(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, int method, double magnitude)
{

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
    pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_in);

    switch (method)
    {
    case 1:
        ne.setKSearch(magnitude);
        break;
    case 2:
        ne.setRadiusSearch(magnitude);
        break;
    default:
        break;
    }

    ne.compute(*cloud_out);
}

/**
 * (2/2) Computes the normal for each point of a pointcloud
 *
 * @param cloud_in:  Pointer to the pointcloud to compute the normals.
 * @param cloud_out: Pointer to the pointcloud in which the result should be saved.
 * @param method:    Method selected for the nearenes neighbour selection for the normal computation.
 *                  - 1 : setKSearch()
 *                  - 2 : setRadiusSearch()
 * @param magnitude: Magnitude for the method selected:
*                   - method 1 : number of neighbours to be selected
 *                  - method 2 : radius of the neighbourhood 
 */

void PreProcessing::compute_normals(Point_Cloud_N::Ptr &cloud_in, pcl::PointCloud<pcl::Normal>::Ptr &cloud_out, int method, double magnitude)
{

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
    pcl::NormalEstimation<pcl::PointNormal, pcl::Normal> ne;
    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_in);

    switch (method)
    {
    case 1:
        ne.setKSearch(magnitude);
        break;
    case 2:
        ne.setRadiusSearch(magnitude);
        break;
    default:
        break;
    }

    ne.compute(*cloud_out);
}

/**
 * Removes the specified points from a pointcloud
 *
 * @param cloud_in:  Pointer to the pointcloud to be filtered.
 * @param cloud_out: Pointer to the pointcloud in which the result should be saved.
 * @param indices:   Indices of the points to be removed.
 * @param visualize: Bollean controling if the visualization is active or not.
 */

void PreProcessing::remove_points(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, pcl::PointIndices::Ptr &indices, bool visualize)
{

    pcl::ExtractIndices<pcl::PointNormal> extract;

    extract.setInputCloud(cloud_in);
    extract.setIndices(indices);
    extract.setNegative(true);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
    extract.filter(*cloud_filtered);
    utilities.step_visualization(cloud_in, cloud_filtered, "After Removing points", visualize);
    copyPointCloud(*cloud_filtered, *cloud_out);
}

/**
 * Fits a plane to the pointcloud and removes all the inliers to this plane as well as the points under the plane.
 *
 * @param cloud_in:             Pointer to the pointcloud to be filtered.
 * @param cloud_out:            Pointer to the pointcloud in which the result should be saved.
 * @param coefficients_plane:   Pointer to the object where the coefficients of the plane fitted should be saved.
 * @param visualize:            Bollean controling if the visualization is active or not.
 */

void PreProcessing::fit_plane(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, pcl::ModelCoefficients::Ptr coefficients_plane, bool visualize)
{
    std::cout << "|--> Planar object Fitting." << std::endl;

    // Parameters to modify the behaviour of this function:
    int ransac_iterations = 100;
    float inliers_treshold = 0.07;
    float point_to_plane_minimum_distance = 0.0;
    float normalvsdistanceweight = 0.1;
    int num_of_neighbours = 50;

    std::cout << "|  |--> Function parameters:" << std::endl;
    std::cout << "|  |  |--> Ransac Parameters:" << std::endl;
    std::cout << "|  |  |  |--> Num of iterations           [ " << ransac_iterations << " ]" << std::endl;
    std::cout << "|  |  |  |--> Inliers treshold            [ " << inliers_treshold << " ]" << std::endl;
    std::cout << "|  |  |  |--> Normal vs Distance Weight   [ " << normalvsdistanceweight << " ]" << std::endl;
    std::cout << "|  |  |--> Normal Calculation:" << std::endl;
    std::cout << "|  |     |--> Num of neighbours           [ " << num_of_neighbours << " ]" << std::endl;

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
    copyPointCloud(*cloud_in, *cloud_filtered);

    std::cout << "|  |--> Computing scene normals." << std::endl;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    PreProcessing::compute_normals(cloud_filtered, cloud_normals, 1, num_of_neighbours);

    std::cout << "|  |--> Build the plane segmentation object." << std::endl;
    pcl::SACSegmentationFromNormals<pcl::PointNormal, pcl::Normal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(normalvsdistanceweight);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ransac_iterations);
    seg.setDistanceThreshold(inliers_treshold);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);

    // Obtain the plane inliers and coefficients
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);

    std::cout << "|  |--> Apply segmentation." << std::endl;
    seg.segment(*inliers_plane, *coefficients_plane);

    // Plane coefficients:
    float a = coefficients_plane->values[0];
    float b = coefficients_plane->values[1];
    float c = coefficients_plane->values[2];
    float d = coefficients_plane->values[3];

    std::cout << "|  |--> Biggest planar object found in the scene." << std::endl;
    std::cout << "|  |  |--> Number of inliers: " << inliers_plane->indices.size() << std::endl;
    std::cout << "|  |  |--> Plane coefficients: " << a << "x " << b << "y " << c << "z + " << d << " = 0 " << std::endl;

    std::cout << "|  |--> Removing points which are inliers to the planar object." << std::endl;
    PreProcessing::remove_points(cloud_filtered, cloud_filtered, inliers_plane, visualize);
    std::cout << "|  |  |--> Point cloud has " << cloud_filtered->points.size() << " data points." << std::endl;

    std::cout << "|  |--> Removing points under the plane." << std::endl;
    pcl::PointIndices::Ptr points_under_plane(new pcl::PointIndices);

    float x, y, z, Distance;
    for (uint i = 0; i < cloud_filtered->size(); i++)
    {
        x = cloud_filtered->at(i).x;
        y = cloud_filtered->at(i).y;
        z = cloud_filtered->at(i).z;

        // Distance point to plane:
        Distance = (a * x + b * y + c * z + d) / (std::sqrt(a * a + b * b + c * c));

        if (Distance < point_to_plane_minimum_distance)
        {
            points_under_plane->indices.push_back(i);
        }
    }

    PreProcessing::remove_points(cloud_filtered, cloud_filtered, points_under_plane, visualize);
    std::cout << "|     |--> Point cloud has " << cloud_filtered->points.size() << " data points." << std::endl;
    copyPointCloud(*cloud_filtered, *cloud_out);
}

/**
 * Finds the clusters of points in the pointclouds and then filters the result, keeping only those that meet certain requirements.
 *
 * @param cloud_in:             Pointer to the pointcloud to be filtered.
 * @param cloud_out:            Pointer to the pointcloud in which the result should be saved.
 * @param visualize:            Bollean controling if the visualization is active or not.
 */

void PreProcessing::remove_small_clusters(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, bool visualize)
{
    std::cout << "|--> Cluster Filtering." << std::endl;

    // Parameters to modify the behaviour of this program:
    // Clustering Parameters:
    int min_cluster_size = 1000;
    int max_cluster_size = 30000;
    float cluster_tolerance = 0.0015; // cm

    std::cout << "|  |--> Function parameters:" << std::endl;
    std::cout << "|  |  |--> Cluster Parameters:" << std::endl;
    std::cout << "|  |  |  |--> Cluster size, Max           [ " << max_cluster_size << " ]" << std::endl;
    std::cout << "|  |  |  |--> Cluster size, Min           [ " << min_cluster_size << " ]" << std::endl;
    std::cout << "|  |  |  |--> Cluster tolerance           [ " << cluster_tolerance << " ]" << std::endl;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    tree->setInputCloud(cloud_in);

    std::cout << "|  |--> Build Euclidian cluster extraction object" << std::endl;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;
    ec.setClusterTolerance(cluster_tolerance); // cm
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);

    Point_Cloud_N::Ptr cloud_filtered(new Point_Cloud_N);

    int j = 0;

    pcl::visualization::PCLVisualizer v("Cluster filtering: Different clusters");

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        Point_Cloud_N::Ptr cloud_cluster(new Point_Cloud_N);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cloud_cluster->points.push_back(cloud_in->points[*pit]);
            cloud_filtered->points.push_back(cloud_in->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = true;

        if (visualize)
        {
            v.setCameraPosition(-0.252034, 0.341164, -0.57032, -0.128131, 0.122338, 0.0488766, -0.0385026, -0.94455, -0.326102);
            v.addPointCloud<pcl::PointNormal>(cloud_cluster, pcl::visualization::PointCloudColorHandlerRandom<pcl::PointNormal>(cloud_cluster), "cloud" + j);
            v.spinOnce();
        }

        j++;
    }

    utilities.step_visualization(cloud_in, cloud_filtered, "Cluster filtering: Result", visualize);
    copyPointCloud(*cloud_filtered, *cloud_out);
}

/**
 * (1/2) Smooths the pointcloud applying Moving least squares method.
 *
 * @param cloud_in:    Pointer to the pointcloud to be filtered.
 * @param cloud_out:   Pointer to the pointcloud in which the result should be saved.
 * @param visualize:   Bollean controling if the visualization is active or not.
 */

void PreProcessing::smoothing_cloud(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, bool visualize)
{

    // Parameters to modify the behaviour of this function:
    int filter_order = 2;
    float search_radius = 0.01;

    std::cout << "|--> Smoothing Point cloud:" << std::endl;
    std::cout << "|  |--> Original Point cloud has " << cloud_in->points.size() << " data points." << std::endl;
    std::cout << "|  |--> Function parameters:" << std::endl;
    std::cout << "|  |  |--> Polynomial Order               [ " << filter_order << " ]" << std::endl;
    std::cout << "|  |  |--> Search radius                  [ " << search_radius << " ]" << std::endl;
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;

    mls.setComputeNormals(true);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

    // Set parameters
    mls.setInputCloud(cloud_in);
    mls.setPolynomialOrder(filter_order);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(search_radius);

    // Reconstruct
    mls.process(*cloud_filtered);

    utilities.step_visualization(cloud_in, cloud_filtered, "After Smoothing", visualize);
    copyPointCloud(*cloud_filtered, *cloud_out);
}

/**
 * (2/2) Smooths the pointcloud applying Moving least squares method.
 *
 * @param cloud_in:    Pointer to the pointcloud to be filtered.
 * @param cloud_out:   Pointer to the pointcloud in which the result should be saved.
 * @param visualize:   Bollean controling if the visualization is active or not.
 */

void PreProcessing::smoothing_cloud(Point_Cloud_XYZ::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, bool visualize)
{

    // Parameters to modify the behaviour of this function:
    int filter_order = 2;
    float search_radius = 0.01;

    std::cout << "|--> Smoothing Point cloud:" << std::endl;
    std::cout << "|  |--> Original Point cloud has " << cloud_in->points.size() << " data points." << std::endl;
    std::cout << "|  |--> Function parameters:" << std::endl;
    std::cout << "|  |  |--> Polynomial Order   [ " << filter_order << " ]" << std::endl;
    std::cout << "|  |  |--> Search radius      [ " << search_radius << " ]" << std::endl;

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

    // Set parameters
    mls.setInputCloud(cloud_in);
    mls.setPolynomialOrder(filter_order);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(search_radius);

    // Reconstruct
    mls.process(*cloud_filtered);

    utilities.step_visualization(cloud_in, cloud_filtered, "After Smoothing", visualize);
    copyPointCloud(*cloud_filtered, *cloud_out);
}

/**
 * Downsample the pointcloud using a VoxelGrid filter and an specific leaf size.
 *
 * @param cloud_in:  Pointer to the pointcloud to be filtered.
 * @param cloud_out: Pointer to the pointcloud in which the results will be saved.
 * @param leaf_size: Float with the leaf size used in the filter.
 */

void PreProcessing::downsample_pointcloud(Point_Cloud_N::Ptr &cloud_in, Point_Cloud_N::Ptr &cloud_out, float leaf_size, bool visualize)
{
    std::cout << "|--> Downsample Point cloud with leaf_size [ " << leaf_size << " ] " << std::endl;
    std::cout << "|  |--> Original Point cloud has " << cloud_in->points.size() << " data points." << std::endl;
    pcl::VoxelGrid<pcl::PointNormal> avg;
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointNormal>);

    avg.setInputCloud(cloud_in);
    avg.setLeafSize(leaf_size, leaf_size, leaf_size);
    avg.filter(*cloud_filtered);
    std::cout << "|  |--> Downsampled Point cloud has " << cloud_filtered->points.size() << " data points." << std::endl;
    utilities.step_visualization(cloud_in, cloud_filtered, "After Downsampling", visualize);
    copyPointCloud(*cloud_filtered, *cloud_out);
}
