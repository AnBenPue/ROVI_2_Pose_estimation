// ROS headers
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Project custom headers
#include "PreProcessing.h"
#include "localization.h"
#include "Utils.h"

// std headers
#include <iostream>

// namspace
using namespace sensor_msgs;

typedef pcl::PointCloud<pcl::PointNormal> Point_Cloud_N;
typedef pcl::PointCloud<pcl::PointXYZ> Point_Cloud_XYZ;

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_XYZRGBA(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::visualization::PCLVisualizer *v1;

void callback(const sensor_msgs::PointCloud2 pCloud)
{
  // new cloud formation
  pcl::fromROSMsg(pCloud, *scene_XYZRGBA);
  v1->updatePointCloud(scene_XYZRGBA, "scene");
  v1->setCameraPosition(-0.252034, 0.341164, -0.57032, -0.128131, 0.122338, 0.0488766, -0.0385026, -0.94455, -0.326102);
}

static void show_usage(std::string name)
{
  std::cout << "Usage: "
            << "<Path to pcd file> <option(s)>\n"
            << "This program estimates the position of the piece in the scene.\n"
            << "Options:\n"
            << "\t-h,--help \tShow this help message\n"
            << "\t-s,--save \t Save file eg: -c -s test.pcd\n"
            << "\t-l,--load \t Load file eg: -l Pointcloud_samples/test.pcd\n"
            << "\t-c,--camera \t enable ros node and real time camera "
            << std::endl;
}
int main(int argc, char **argv)
{
  bool save_file = false;
  bool load_file = false;
  bool enable_camera = false;

  std::string load_filename, save_filename;

  PreProcessing Pre_processor;
  Localization localizer;
  Utils utilities;

  Point_Cloud_XYZ::Ptr scene_XYZ(new Point_Cloud_XYZ);
  Point_Cloud_N::Ptr scene(new Point_Cloud_N);
  Point_Cloud_N::Ptr object(new Point_Cloud_N);
  Point_Cloud_N::Ptr object_final(new Point_Cloud_N);

  for (int i = 1; i < argc; ++i)
  {
    std::string arg = argv[i];
    if ((arg == "-h") || (arg == "--help"))
    {
      show_usage(argv[0]);
      return 0;
    }
    else if ((arg == "-s") || (arg == "--save"))
    {
      save_file = true;
      save_filename = argv[i + 1];
      cout << "|--> Saving of the Point Cloud enabled" << endl;
    }
    else if ((arg == "-l") || (arg == "--load"))
    {
      load_file = true;
      load_filename = argv[i + 1];
      cout << "|--> Loading of the Point Cloud enabled" << endl;
    }
    else if ((arg == "-c") || (arg == "--camera"))
    {
      enable_camera = true;
      utilities.load_object_of_interest(object, "./Utilities/Piece.pcd");
      cout << "|--> ROS node and Camera Point Cloud enabled" << endl;
    }
  }

  if (enable_camera)
  {
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    ros::Rate rate(30); // frequency of operation

    // subscribe
    ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, callback);
    // ros::Subscriber sub2 = nh.subscribe()
    //ros::Publisher pub = nh.advertise<std_msgs::String>("object_pose",100);

    //ros::spinOnce();
    v1 = new pcl::visualization::PCLVisualizer("Print scene");
    v1->addPointCloud<pcl::PointXYZRGBA>(scene_XYZRGBA, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA>(scene_XYZRGBA, 255, 0, 0), "scene");
    while (!v1->wasStopped())
    {
      ros::spinOnce();
      rate.sleep();
      v1->spinOnce();
      //viewer_local->spinOnce (100);
    }
    // // Shutdown subscriber
    sub.shutdown();
    if (save_file)
    {
      if (!pcl::io::savePCDFileASCII(save_filename, *scene_XYZRGBA))
      {
        std::cout << "|--> File (.pcd) containing the scene was saved correctly." << std::endl;
        std::cout << "|  |--> PointCloud has: " << scene_XYZRGBA->points.size() << " data points." << std::endl;
        return 0;
      }
      else
      {
        std::cout << "|--> ERROR: Couldn't save File (.pcd) containing the scene." << std::endl;
        return 0;
      }
    }
  }
  else
  {
    if (!pcl::io::loadPCDFile(load_filename, *scene_XYZRGBA))
    {
      std::cout << "|--> File (.pcd) containing the scene was loaded correctly." << std::endl;
      std::cout << "|  |--> PointCloud has: " << scene_XYZRGBA->points.size() << " data points." << std::endl;
      // Removing the extension:
      size_t lastindex = load_filename.find_last_of(".");
      load_filename = load_filename.substr(0, lastindex);
      utilities.load_object_of_interest(object, "./Utilities/Piece.pcd");
    }
    else
    {
      std::cout << "|--> ERROR: Couldn't load File (.pcd) containing the scene." << std::endl;
      return 0;
    }
  }

  // Clone clouds for later use
  copyPointCloud(*object, *object_final);
  copyPointCloud(*scene_XYZRGBA, *scene_XYZ);

  // Passthrough filtering
  Pre_processor.Filter_XYZ(scene_XYZ, scene_XYZ, true);
  copyPointCloud(*scene_XYZ, *scene);

  // Compute normals for the scene
  Pre_processor.compute_normals(scene, scene, 1, 3.0);

  // Downsample cloud
  Pre_processor.downsample_pointcloud(scene, scene, 0.001, true);

  // Planar filtering
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  Pre_processor.fit_plane(scene, scene, coefficients_plane, true);

  // Cluster filtering
  Pre_processor.remove_small_clusters(scene, scene, true);

  // Smoothing using MLS
  //Pre_processor.smoothing_cloud(object, object, true);
  //Pre_processor.smoothing_cloud(scene, scene, true);

  // Downsample both Pointlcouds
  Pre_processor.downsample_pointcloud(object, object, 0.005, true);
  Pre_processor.downsample_pointcloud(scene, scene, 0.005, true);

  // Compute normals for both Pointlcouds
  Pre_processor.compute_normals(object, object, 1, 3.0);
  Pre_processor.compute_normals(scene, scene, 1, 3.0);

  // Global alignment
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  localizer.global_alignment(scene, object, T, 1000, true);

  // Local alignemnt
  localizer.local_alignment(scene, object, T, 500, true);

  // Show results
  utilities.display_result_RGB(object_final, scene_XYZRGBA, T);

  // Convert T
  rw::math::Rotation3D<float> tempR(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2));
  rw::math::Quaternion<float> tempQ(tempR);
    //rw::math::Vector3D<double> tempP(T(0,3),T(1,3),T(2,3));
  std::cout << "|--> Estimated pose." << std::endl;
  std::cout << "|  |--> Rotation Matrix." << std::endl;
  std::cout << "|  | " << std::setprecision(2) << std::setw(7) << T(0, 0) << std::setw(7) << T(0, 1) << std::setw(7) << T(0, 2) << std::endl;
  std::cout << "|  | " << std::setw(7) << T(1, 0) << std::setw(7) << T(1, 1) << std::setw(7) << T(1, 2) << std::endl;
  std::cout << "|  | " << std::setw(7) << T(2, 0) << std::setw(7) << T(2, 1) << std::setw(7) << T(2, 2) << std::endl;
  std::cout << "|  | " << std::endl;
  std::cout << "|  |--> Quaternions." << std::endl;
  std::cout << "|  | " << std::setw(7) << tempQ.getQx() << std::setw(7) << tempQ.getQy() << std::setw(7) << tempQ.getQz() << std::setw(7) << tempQ.getQw() << std::endl;
  std::cout << "|  | " << std::endl;
  std::cout << "|  |--> Translation." << std::endl;
  std::cout << "|  | " << std::setw(7) << T(0, 3) << std::setw(7) << T(1, 3) << std::setw(7) << T(2, 3) << std::endl;


  //ros::spin();
  /*if(enable_camera)
  {
      tf2_ros::StaticTransformBroadcaster static_broadcaster;

      geometry_msgs::TransformStamped static_tf;

      static_tf.header.stamp = ros::Time::now();
      static_tf.header.frame_id = "ObjectPose";
      static_tf.child_frame_id = "Pose_for_robot";
      static_tf.transform.translation.x = T(0, 3);
      static_tf.transform.translation.y = T(1, 3);
      static_tf.transform.translation.z = T(2, 3);
      static_tf.transform.rotation.x = tempQ.getQx();
      static_tf.transform.rotation.y = tempQ.getQy();
      static_tf.transform.rotation.z = tempQ.getQz();
      static_tf.transform.rotation.w = tempQ.getQw();
      ros::Rate rate(30);
      while (ros::ok())
      {
      // pub.publish(T);
      static_broadcaster.sendTransform(static_tf);
      ros::spinOnce();
      rate.sleep();
      }
  }*/

  return 0;
}