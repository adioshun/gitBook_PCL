# [How to use Normal Distributions Transform](http://pointclouds.org/documentation/tutorials/normal_distributions_transform.php#normal-distributions-transform)


Normal Distributions Transform (NDT)알고리즘을 사용하여서 두 점군간 강체 변환을 결정하는 방법을 살펴 본다. `In this tutorial we will describe how to use the Normal Distributions Transform (NDT) algorithm to determine a rigid transformation between two large point clouds, both over 100,000 points. `

NDT는 정합 알고리즘 이다. : The NDT algorithm is a registration algorithm that uses standard optimization techniques applied to statistical models of 3D points to determine the most probable registration between two point clouds. 

자세한 내용은 다음 논문을 차고 하기 바란다. `For more information on the inner workings of the NDT algorithm, see Dr. Martin Magnusson’s doctoral thesis, `

> “The Three-Dimensional Normal Distributions Transform – an Efficient Representation for Registration, Surface Analysis, and Loop Detection.”

```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h> // NormalDistributionsTransform

/* ---[ */
int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> Final;   

  pcl::io::loadPCDFile ("bun0.pcd", *cloud_in);
  pcl::io::loadPCDFile ("bun4.pcd", *cloud_out);

  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> reg;
  reg.setStepSize (0.05);
  reg.setResolution (0.025f);
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
