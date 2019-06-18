# [Smoothing and normal estimation based on polynomial reconstruction](http://pointclouds.org/documentation/tutorials/resampling.php#moving-least-squares)


```cpp
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

int
main (int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile ("table_scene_lms400_downsampled.pcd", *cloud);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  // the first template type is used for the input and output cloud. 
  // Only the XYZ dimensions of the input are smoothed in the output.
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
   // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (mls_points);

  // Save output
  pcl::io::savePCDFile ("table_scene_lms400_upsampled.pcd", mls_points);
}
```


---

# [PCL/OpenNI tutorial 2: Cloud processing (Surface smoothing)](http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing\_\(basic\))



```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>

int
main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// The output will also contain the normals.
	pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	// Smoothing object (we choose what point types we want as input and output).
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
	filter.setInputCloud(cloud);
	// Use all neighbors in a radius of 3cm.
	filter.setSearchRadius(0.03);
	// If true, the surface and normal are approximated using a polynomial estimation
	// (if false, only a tangent one).
	filter.setPolynomialFit(true);
	// We can tell the algorithm to also compute smoothed normals (optional).
	filter.setComputeNormals(true);
	// kd-tree object for performing searches.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
	filter.setSearchMethod(kdtree);

	filter.process(*smoothedCloud);
}
```