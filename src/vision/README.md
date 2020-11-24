## Realsense D435 Installation:
Additional steps for the camera installation.

* [Github](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) - Install librealsense packages.
	
* Rgbd error:
```
	sudo apt-get install ros-melodic-rgbd-launch
```
* [minsung](http://www.minsung.org/2018/08/intel-realsense-d435-setup-on-ubuntu-16-04/) - Installing realsense SDK.
* [Github](https://github.com/IntelRealSense/librealsense/issues/2846) - Error: Failed to unload module videobuf2_core.


## Vision Package:
Running the package (Make sure that the camera node is on):
```
rosrun vision pose_estimation -c
```
Currently the package contains the following utilities:

* **pose_estimation** : runs the pose estimation process.
* **PLY_2_PCL** : Convert a .ply file to .pcd file.
* **Filter_XYZ** : Applies a spatial filter to the point cloud, removing the points which arent in between certain values.
* **Table_Markers_detection**: Applies a series of filters to find the screw holes on the table (NOT USED)
* **fit_plane** : Fits a plane in the point cloud. Used to find the table in the cloud.
* **remove_small_clusters** : Clusters the point cloud and afterwards removes the clusters which don't meet certain requirements.
* **locate_piece** : Applies Global alignmenr in order to find the piece  in the point clouds. After this, it applies local alignment to correct the pose estimation.

## Possible problems:
Make sure the CMakelists.txt in vision package contains the correct PCL verion and directory of the current computer.
``` 
cd src/vision/
gedit CMakeLists.txt 
```
* Hemanth:
 	- set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8 /usr/include/eigen3/)
* Antoni:
	 - set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8 /usr/include/eigen3/)
