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
  //pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud_filtered);
  std::cerr << "PointCloud : " << cloud_filtered->points.size() << " data points." << std::endl;



  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional 
  //http://answers.ros.org/question/55223/what-does-setoptimizecoefficients-do-in-sacsegmentation/
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
   
  
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }

  return (0);
}
```