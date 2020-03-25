# [How to use iterative closest point](http://pointclouds.org/documentation/tutorials/iterative_closest_point.php#iterative-closest-point)


This document demonstrates using the Iterative Closest Point algorithm in your code which can determine if one PointCloud is just a rigid transformation of another by minimizing the distances between the points of two pointclouds and rigidly transforming them.

The code


```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

// How to use iterative closest point
// http://pointclouds.org/documentation/tutorials/iterative_closest_point.php#iterative-closest-point

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile ("bun0.pcd", *cloud_in);
  pcl::io::loadPCDFile ("bun4.pcd", *cloud_out);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  
  pcl::PointCloud<pcl::PointXYZ> Final;   
  icp.align(Final);

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<   // 정확히 정합되면 1(True)
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;                // 변환 행렬 출력 

 return (0);
}

```
