# [Point Cloud Compression](http://pointclouds.org/documentation/tutorials/compression.php#octree-compression)





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
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

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