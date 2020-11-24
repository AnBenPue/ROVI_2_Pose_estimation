#include "Utils.h"

Utils::Utils()
{
}

Utils::~Utils()
{
}

/**
 * (1/4) Visualization of a step in the localization process.
 *
 * @param cloud_1:   Pointer to the pointcloud containing the Object.
 * @param cloud_2:   Pointer to the pointcloud containing the Scene.
 * @param step_name: String with the desired name for the visualization window.
 * @param enabled:   Bollean controling if the visualization is active or not.
 */

void Utils::step_visualization(Point_Cloud_N::Ptr &cloud_1, Point_Cloud_N::Ptr &cloud_2, std::string step_name, bool enabled)
{
    if (enabled)
    {
        pcl::visualization::PCLVisualizer v(step_name);
        v.setCameraPosition(-0.252034, 0.341164, -0.57032, -0.128131, 0.122338, 0.0488766, -0.0385026, -0.94455, -0.326102);
        v.addPointCloud<pcl::PointNormal>(cloud_1, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_1, 255, 0, 0), "Cloud_1");
        v.addPointCloud<pcl::PointNormal>(cloud_2, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_2, 0, 255, 0), "Cloud_2");
        v.spinOnce();
    }
}

/**
 * (2/4) Visualization of a step in the localization process. The difference with (1/3) is the type of pointcloud.
 *
 * @param cloud_1:   Pointer to the pointcloud containing the Object.
 * @param cloud_2:   Pointer to the pointcloud containing the Scene.
 * @param step_name: String with the desired name for the visualization window.
 * @param enabled:   Bollean controling if the visualization is active or not.
 */

void Utils::step_visualization(Point_Cloud_XYZ::Ptr &cloud_1, Point_Cloud_N::Ptr &cloud_2, std::string step_name, bool enabled)
{
    if (enabled)
    {
        pcl::visualization::PCLVisualizer v(step_name);
        v.setCameraPosition(-0.252034, 0.341164, -0.57032, -0.128131, 0.122338, 0.0488766, -0.0385026, -0.94455, -0.326102);
        v.addPointCloud<pcl::PointXYZ>(cloud_1, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_1, 255, 0, 0), "Cloud_1");
        v.addPointCloud<pcl::PointNormal>(cloud_2, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_2, 0, 255, 0), "Cloud_2");
        v.spinOnce();
    }
}

/**
 * (3/4) Visualization of a step in the localization process. In addition, show the correspondences between bouth clouds.
 *
 * @param cloud_1:   Pointer to the pointcloud containing the Object.
 * @param cloud_2:   Pointer to the pointcloud containing the Scene.
 * @param corr:      Correspondences between the two pointclouds.
 * @param step_name: String with the desired name for the visualization window.
 * @param enabled:   Bollean controling if the visualization is active or not.
 */

void Utils::step_visualization(Point_Cloud_N::Ptr &cloud_1, Point_Cloud_N::Ptr &cloud_2, pcl::Correspondences corr, std::string step_name, bool enabled)
{
    if (enabled)
    {
        pcl::visualization::PCLVisualizer v(step_name);
        v.setCameraPosition(-0.252034, 0.341164, -0.57032, -0.128131, 0.122338, 0.0488766, -0.0385026, -0.94455, -0.326102);
        v.addPointCloud<pcl::PointNormal>(cloud_1, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_1, 255, 0, 0), "cloud_1");
        v.addPointCloud<pcl::PointNormal>(cloud_2, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(cloud_2, 0, 255, 0), "cloud_2");
        v.addCorrespondences<pcl::PointNormal>(cloud_2, cloud_1, corr, 1);
        v.spinOnce();
    }
}

/**
 * (4/4) Visualization of a step in the localization process.
 *
 * @param cloud_1:   Pointer to the pointcloud containing the Object.
 * @param cloud_2:   Pointer to the pointcloud containing the Scene.
 * @param step_name: String with the desired name for the visualization window.
 * @param enabled:   Bollean controling if the visualization is active or not.
 */

void Utils::step_visualization(Point_Cloud_XYZ::Ptr &cloud_1, Point_Cloud_XYZ::Ptr &cloud_2, std::string step_name, bool enabled)
{
    if (enabled)
    {
        pcl::visualization::PCLVisualizer v(step_name);
        v.setCameraPosition(-0.252034, 0.341164, -0.57032, -0.128131, 0.122338, 0.0488766, -0.0385026, -0.94455, -0.326102);
        v.addPointCloud<pcl::PointXYZ>(cloud_1, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_1, 255, 0, 0), "Cloud_1");
        v.addPointCloud<pcl::PointXYZ>(cloud_2, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_2, 0, 255, 0), "Cloud_2");
        v.spinOnce();
    }
}

/**
 * (1/2) Saves variables values to a file.
 *
 * @param filepath: Path to the .txt file, if it doesn't exist, it will be created.
 * @param d1-d9:    Variables values to be saved.
 */

void save_data(std::string filepath, double d1, double d2, double d3, double d4, double d5, double d6, double d7, double d8, double d9)
{
    std::ofstream file;
    //can't enable exception now because of gcc bug that raises ios_base::failure with useless message
    //file.exceptions(file.exceptions() | std::ios::failbit);
    file.open(filepath, std::ios::out | std::ios::app);
    if (file.fail())
        throw std::ios_base::failure(std::strerror(errno));

    //make sure write fails with exception if something is wrong
    file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);

    file.seekp(0, ios::end);
    size_t size = file.tellp();
    if (size == 0)
    {

        std::cout << "File: " << filepath << " wasn't initialized, initializing now." << std::endl;
        file << " Data =[";
    }

    file << d1 << "," << d2 << "," << d3 << "," << d4 << "," << d5 << "," << d6 << "," << d7 << "," << d8 << "," << d9 << ";"
         << "\n";
}

/**
 * (2/2) Saves variables values to a file.
 *
 * @param filepath: Path to the .txt file, if it doesn't exist, it will be created.
 * @param d1-d6:    Variables values to be saved.
 */

void Utils::save_data(std::string filepath, double d1, double d2, double d3, double d4, double d5, double d6)
{
    std::ofstream file;
    //can't enable exception now because of gcc bug that raises ios_base::failure with useless message
    //file.exceptions(file.exceptions() | std::ios::failbit);
    file.open(filepath, std::ios::out | std::ios::app);
    if (file.fail())
        throw std::ios_base::failure(std::strerror(errno));

    //make sure write fails with exception if something is wrong
    file.exceptions(file.exceptions() | std::ios::failbit | std::ifstream::badbit);

    file.seekp(0, ios::end);
    size_t size = file.tellp();
    {
        std::cout << " File: " << filepath << " wasn't initialized, initializing now." << std::endl;
        file << " Data =[";
    }

    file << d1 << "," << d2 << "," << d3 << "," << d4 << "," << d5 << "," << d6 << ";"
         << "\n";
}

/**
 * Loads a pointcloud contained in a .pcd file.
 *
 * @param object: Pointer to the pointcloud in where the object has to be saved.
 * @param path:   String containing the path to the object. 
 */

void Utils::load_object_of_interest(Point_Cloud_N::Ptr &object, std::string path)
{
    if (!pcl::io::loadPCDFile(path, *object))
    {
        std::cout << "--> File (.pcd) loaded correctly (Piece)" << std::endl;
        std::cout << "----> PointCloud has: " << object->points.size() << " data points." << std::endl;
    }
    else
    {
        std::cout << "--> Couldn't load file (object.pcd) at:  " << path << std::endl;
    }
}

/**
 * Computes the angle between the z axis of a transformation matrix and a plane.
 *
 * @param T:                  Transformation matrix representing the pose estimated, regarding the camera frame.
 * @param coefficients_plane: coeficients defining the plane. 
 */

float Utils::vectors_angle(Eigen::Matrix4f T, pcl::ModelCoefficients::Ptr coefficients_plane)
{

    rw::math::Vector3D<float> Plane_normal(coefficients_plane->values[0], coefficients_plane->values[1], coefficients_plane->values[2]);
    rw::math::Vector3D<float> Piece_Z_axis(T(0, 2), T(1, 2), T(2, 2));

    float angle = rw::math::angle(Plane_normal, Piece_Z_axis);

    return angle;
}

/**
 * Displays the original pointcloud and the objcet transformed onto the estimated pose.
 *
 * @param object: Pointer to the pointcloud containing the object.
 * @param scene:  Pointer to the pointcloud containing the scene.
 * @param T:      Transformation matrix representing the pose estimated, regarding the camera frame.
 */

void Utils::display_result_RGB(Point_Cloud_N::Ptr &object, Point_Cloud_XYZRGBA::Ptr &scene, Eigen::Matrix4f T)
{

    // Apply transformation to the object:
    Point_Cloud_N::Ptr object_transformed(new Point_Cloud_N);
    transformPointCloud(*object, *object_transformed, T);

    pcl::visualization::PCLVisualizer v("Final result with the estimated pose");
    v.setCameraPosition(-0.252034, 0.341164, -0.57032, -0.128131, 0.122338, 0.0488766, -0.0385026, -0.94455, -0.326102);
    v.addPointCloud<pcl::PointXYZRGBA>(scene, "Scene");
    v.addPointCloud<pcl::PointNormal>(object_transformed, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(object_transformed, 255, 0, 0), "object_transformed");
    v.spin();
}

/**
 * Prints a progress bar based on total value and current progress.
 *
 * @param current: Current progress.
 * @param total:   Total value.
 */

void Utils::progressbar(int current, int total)
{
    current = current * 100 / total/2;
    if (current > 100)
        current = 100;
    std::cout << "          [";

    for (int i = 0; i < 100/2; i++)
        if (i < current)
            std::cout << '=';
        else if (i == current)
            std::cout << '>';
        else
            std::cout << ' ';
    std::cout << "] " << current*2 << " % "
              << " Total [ " << total << " ] ";
    std::cout.flush();
    std::cout << std::endl;

    // TODO: this won't work everywhere, the idea is to reset the bash cursor to the begining of the line to overwrite it insted of adding a new one
    if (current != total)
    {
        std::system("printf \"\033[A\"");
    }
}
