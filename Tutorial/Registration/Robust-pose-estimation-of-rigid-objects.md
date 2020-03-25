# [Robust pose estimation of rigid objects](http://pointclouds.org/documentation/tutorials/alignment_prerejective.php#alignment-prerejective)

In this tutorial, we show how to find the alignment pose of a rigid object in a scene with clutter and occlusions.

```cpp

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// Types
//typedef pcl::PointNormal PointNT;
//typedef pcl::PointCloud<pcl::PointNormal> PointCloudT;
//typedef pcl::FPFHSignature33 FeatureT;
//typedef pcl::FPFHEstimationOMP<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33> FeatureEstimationT;
//typedef pcl::PointCloud<pcl::FPFHSignature33> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> ColorHandlerT;

const float leaf = 0.005f;

// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{
  // Point clouds
  pcl::PointCloud<pcl::PointNormal>::Ptr object (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr object_aligned (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr scene (new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features (new pcl::PointCloud<pcl::FPFHSignature33>);
  
  pcl::io::loadPCDFile<pcl::PointNormal> ("chef.pcd", *object);
  pcl::io::loadPCDFile<pcl::PointNormal> ("rs1.pcd", *scene);

  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<pcl::PointNormal> grid;
  const float leaf = 0.005f;                 //5 mm
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);
   
  // Estimate normals for scene  매칭작업에서 사용할 Feature 계산에 필요한 Normal 생성 
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<pcl::PointNormal,pcl::PointNormal> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (scene);
  nest.compute (*scene);
  
  // Estimate features  Fast Point Feature Histogram (FPFH)생성 
  pcl::console::print_highlight ("Estimating features...\n");
  pcl::FPFHEstimationOMP<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33> fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);
  
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<pcl::PointNormal,pcl::PointNormal,pcl::FPFHSignature33> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  align.align (*object_aligned);   // 정합 실시 

  
  if (align.hasConverged ())  // If a pose with enough inliers was found (more than 25 % of the total number of object points)
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
    

  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return (1);
  }
  
  return (0);
}


```
