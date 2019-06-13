# Decomposition

When I talk about decomposition, I mean the process of creating an ordered, organized data structure from the points of the cloud, with some purpose in mind like making further analysis easier, or performing filters.

- KdTree
- Octree


---

# Point Cloud Compression

Point clouds consist of huge data sets describing three dimensional points associated with additional information such as distance, color, normals, etc. Additionally, they can be created at high rate and therefore occupy a significant amount of memory resources. Once point clouds have to be stored or transmitted over rate-limited communication channels, methods for compressing this kind of data become highly interesting. The Point Cloud Library provides point cloud compression functionality. It allows for encoding all kinds of point clouds including “unorganized” point clouds that are characterized by non-existing point references, varying point size, resolution, density and/or point ordering. Furthermore, the underlying octree data structure enables to efficiently merge point cloud data from several sources.






---
# [Point Cloud Compression](http://pointclouds.org/documentation/tutorials/compression.php#octree-compression)



> http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing_(basic)



```cpp

#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>

int
main(int argc, char** argv)
{
// Objects for storing the point clouds.
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr decompressedCloud(new pcl::PointCloud<pcl::PointXYZ>);

// Read a PCD file from disk.
pcl::io::loadPCDFile<pcl::PointXYZ> ("tabletop.pcd", *cloud);

// Octree compressor object.
// Check /usr/include/pcl-<version>/pcl/compression/compression_profiles.h for more profiles.
// The second parameter enables the output of information.
pcl::io::OctreePointCloudCompression<pcl::PointXYZ> octreeCompression(pcl::io::MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR, true);
// Stringstream that will hold the compressed cloud.
std::stringstream compressedData;

// Compress the cloud (you would save the stream to disk).
octreeCompression.encodePointCloud(cloud, compressedData);

// Decompress the cloud.
octreeCompression.decodePointCloud(compressedData, decompressedCloud);

// Display the decompressed cloud.
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Octree compression"));
viewer->addPointCloud<pcl::PointXYZ>(decompressedCloud, "cloud");
while (!viewer->wasStopped())
{
viewer->spinOnce(100);
boost::this_thread::sleep(boost::posix_time::microseconds(100000));
}
}

```


---

- [A Survey of Compression Strategies for 3D Point Clouds](https://www.youtube.com/watch?v=zgziFNTQ_ZE&feature=youtu.be): Youtube, 9:41

- [Compressing Point Clouds with Point Cloud Library (PCL)](https://www.youtube.com/watch?v=DLvO0b2NBXE&feature=youtu.be): Youtube, 2:02


