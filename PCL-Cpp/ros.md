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
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>       // std::cout
#include <typeinfo>       // operator typeid

ros::Publisher pub;

typedef pcl::PointXYZRGBA              PointXYZRGBA;

void 
cloud_cb (const sensor_msgs::PointCloud2 msg)
{
  std::cout << "msg is: " << typeid(msg).name() << '\n';

  //ros_pcl2 to pcl2
  //http://www.pcl-users.org/ROS-PointCloud2-to-CloudXYZRGBA-td4039503.html
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(msg, pcl_pc);  
  std::cout << "cloud is: " << typeid(msg).name() << '\n';

  //pcl2 to pclxyzrgba
  pcl::PointCloud<PointXYZRGBA> input_cloud;
  pcl::fromPCLPointCloud2(pcl_pc, input_cloud);

  // Publish the data
  sensor_msgs::PointCloud2 output;
  output = msg;  
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
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

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





