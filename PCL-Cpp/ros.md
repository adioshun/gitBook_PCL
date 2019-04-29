## C++ Example    

> [How to use a PCL tutorial in ROS](http://wiki.ros.org/cn/pcl/Tutorials)


ubuntu 16기준 
cd ~/catkin_src/src/
catkin_create_pkg my_pcl_tutorial pcl pcl_ros roscpp sensor_msgs

```cpp
// src/example.cpp

#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;
  std::cout << "test" << std::endl;
  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_test", 1);

  // Spin
  ros::spin ();
}


// CMakeLists.txt

catkin_package() //[rosrun] Couldn't find executable named example below에러시 
add_executable(example src/example.cpp) 
target_link_libraries(example ${catkin_LIBRARIES})
```

cd ~/catkin_src
catkin_make

run : rosrun test example






