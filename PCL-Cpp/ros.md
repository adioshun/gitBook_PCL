# PCL-C++& ROS

## 1. msg 송수신     

> [How to use a PCL tutorial in ROS](http://wiki.ros.org/cn/pcl/Tutorials)





 
   
```cpp
$ cd ~/catkin_src/src/
$ catkin_create_pkg my_pcl_tutorial pcl pcl_ros roscpp sensor_msgs  #CMakeList.txt 자동 생성 
```

코드 작성 

```python 
// src/example.cpp

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>



// Topics
static const std::string SUBSCRIBER_TOPIC = "/velodyne_points";
static const std::string PUBLISH_TOPIC = "test";

// ROS Publisher
ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  ## 메시지 종류
  # sensor_msgs::PointCloud — ROS message (deprecated)
  # sensor_msgs::PointCloud2 — 기본 메시지, ROS message
  # pcl::PCLPointCloud2 — PCL data structure mostly for compatibility with ROS (I think)
  # pcl::PointCloud<T> — 기본 데이터 포맷, standard PCL data structure
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*input, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);
  
  // Do data processing here...
  #output = *input;
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (SUBSCRIBER_TOPIC, 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> (PUBLISH_TOPIC, 1);

  // Spin
  ros::spin ();
}
```



###### CMakeLists.txt
```
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_conversions
    pcl_ros
  )
  
  catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp
                 pcl_conversions
                 pcl_ros
  )
  
  add_executable(roscpp_pcl_example src/roscpp_pcl_example.cpp)
  target_link_libraries(roscpp_pcl_example ${catkin_LIBRARIES})
  
```

###### package.xml

```xml
  <build_depend>roscpp</build_depend>
  <build_depend>pcl_conversions</build_depend>
  <build_depend>pcl_ros</build_depend>
  <build_depend>libpcl-all-dev</build_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>pcl_conversions</run_depend>
  <run_depend>pcl_ros</run_depend>
  <run_depend>libpcl-all</run_depend>



```


###### RUN 

```
catkin_make --directory ~/catkin_ws --pkg ground
source ~/devel/setup.sh
rosrun my_pcl_tutorial example input:=/narrow_stereo_textured/points2
```





