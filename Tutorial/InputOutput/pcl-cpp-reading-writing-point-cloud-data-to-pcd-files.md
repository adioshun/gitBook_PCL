# [Reading Point Cloud data from PCD files](http://www.pointclouds.org/documentation/tutorials/reading_pcd.php#reading-pcd)


















---

# [Writing Point Cloud data to PCD files](http://www.pointclouds.org/documentation/tutorials/writing_pcd.php#writing-pcd)








--- 

# Conversion 



```cpp
// Converting pcl::PCLPointCloud2 to pcl::PointCloud and reverse
#include <pcl/conversions.h>
pcl::PCLPointCloud2 point_cloud2;
pcl::PointCloud<pcl::PointXYZ> point_cloud;
pcl::fromPCLPointCloud2( point_cloud2, point_cloud);
pcl::toPCLPointCloud2(point_cloud, point_cloud2);

//Converting sensor_msgs::PCLPointCloud2 to sensor_msgs::PointCloud and reverse

#include <sensor_msgs/point_cloud_conversion.h>
sensor_msgs::PointCloud2 point_cloud2;
sensor_msgs::PointCloud point_cloud;

sensor_msgs::convertPointCloudToPointCloud2(point_cloud, point_cloud2);
sensor_msgs::convertPointCloud2ToPointCloud(point_cloud2, point_cloud);


```