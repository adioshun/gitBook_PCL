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
  pcl::PointCloud<pcl::PointXYZ> Final;   

  pcl::io::loadPCDFile ("bun0.pcd", *cloud_in);
  pcl::io::loadPCDFile ("bun4.pcd", *cloud_out);

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> reg;
  reg.setInputSource(cloud_in);
  reg.setInputTarget(cloud_out); 
  reg.setMaximumIterations (50);              // 최대 수행 횟수 Set the maximum number of iterations (criterion 1)
  reg.setTransformationEpsilon (1e-8);        // 이전 Transformation과의 최대 변화량 Set the transformation epsilon (criterion 2)
  reg.setMaxCorrespondenceDistance (0.05);    // Set the max correspondence distance to 5cm 
                                              // (e.g., correspondences with higher distances will be ignored)
  //reg.setEuclideanFitnessEpsilon (1);       // Set the euclidean distance difference epsilon (criterion 3)
  reg.align(Final);

  std::cout << "has converged:" << reg.hasConverged() << " score: " <<   // 정확히 정합되면 1(True)
  reg.getFitnessScore() << std::endl;
  
  Eigen::Matrix4f transformation = reg.getFinalTransformation ();
  std::cout << transformation << std::endl;                // 변환 행렬 출력 

 return (0);
}

```

### IterativeClosestPointNonLinear

```cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h> //IterativeClosestPointNonLinear

// https://github.com/otherlab/pcl/blob/master/test/registration/test_registration.cpp

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> Final;   

  pcl::io::loadPCDFile ("bun0.pcd", *cloud_in);
  pcl::io::loadPCDFile ("bun4.pcd", *cloud_out);

  pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> reg;
  reg.setInputCloud (cloud_in);
  reg.setInputTarget (cloud_out);
  reg.setMaximumIterations (50);
  reg.setTransformationEpsilon (1e-8);
  reg.align(Final);

  std::cout << "has converged:" << reg.hasConverged() << " score: " <<   // 정확히 정합되면 1(True)
  reg.getFitnessScore() << std::endl;
  
  Eigen::Matrix4f transformation = reg.getFinalTransformation ();
  std::cout << transformation << std::endl;                // 변환 행렬 출력 

 return (0);
}
```
