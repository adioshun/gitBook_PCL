# [Extracting indices from a PointCloud](http://pointclouds.org/documentation/tutorials/extract_indices.php)


- [How to use Random Sample Consensus model](http://pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus) : 모델이 주어진 상태에서 RANSAC을 이용하여 파라미터 찾아 inlider 찾기

- [Projecting points using a parametric model](http://pointclouds.org/documentation/tutorials/project_inliers.php#project-inliers) : 모델과, 파라미터(coefficients)가 주어진 상태에서 RANSAC을 이용하여 inlier찾기 

- [Extracting indices from a PointCloud](http://pointclouds.org/documentation/tutorials/extract_indices.php): 

In this tutorial we will learn how to use an :pcl:`ExtractIndices <pcl::ExtractIndices>` filter to extract a subset of points from a point cloud based on the indices output by a segmentation algorithm.

In order to not complicate the tutorial, the segmentation algorithm is not explained here. Please check the Plane model segmentation tutorial for more information.

```cpp

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop.pcd", *cloud);
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  int i = 0, nr_points = (int) cloud->points.size ();
  // While 30% of the original cloud is still there
  while (cloud->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    pcl::PCDWriter writer2;
    writer2.write<pcl::PointXYZRGB> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud.swap (cloud_f);
    i++;
  }

  return (0);
}
```

---

# 간단 버젼 

```cpp
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int
main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("tabletop_passthrough.pcd", *cloud);
  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;


 seg.setInputCloud (cloud);
 seg.segment (*inliers, *coefficients);

 // Extract the inliers
 extract.setInputCloud (cloud);
 extract.setIndices (inliers);
 extract.setNegative (false);//true
 extract.filter (*cloud_p);
 std::cerr << "Filtered : " << cloud_p->width * cloud_p->height << " data points." << std::endl;

 std::stringstream ss;
 ss << "RANSAC_plane.pcd";
 pcl::PCDWriter writer2;
 writer2.write<pcl::PointXYZRGB> (ss.str (), *cloud_p, false);

 

  return (0);
}


```