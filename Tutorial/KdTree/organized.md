```cpp

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/filters/filter.h>

// http://docs.ros.org/hydro/api/agile_grasp/html/hands__test_8cpp_source.html
// http://www.pcl-users.org/Re-Eucledian-clustering-bug-gt-OrganizedNeighbor-search-bug-td4033983.html
int
  main (int argc, char** argv)
{
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile ("region_growing_rgb_tutorial.pcd", *cloud);
	std::cout << "input size: " << cloud->points.size () << std::endl;
	
	
	// Remove NAN 
	std::vector<int> indices;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::removeNaNFromPointCloud(*cloud, *outputCloud, indices);
	std::cout << "Number of NAN: " << cloud->points.size () - outputCloud->points.size () << std::endl;
	
	//organized_neighbor
	pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr organized_neighbor(
	new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>());
	
	double taubin_radius = 0.03; // radius of curvature-estimation neighborhood
	std::vector<float> nn_dists;
	std::vector<int> nn_indices;
	
	int sample_index = 0; //  int sample_index = 0;
	if (cloud->isOrganized())
		std::cout << "isOrganized"  << std::endl;
	else
		std::cout << "Not Organized"  << std::endl;
	
	organized_neighbor->setInputCloud(cloud);
	organized_neighbor->radiusSearch(cloud->points[sample_index], taubin_radius, nn_indices, nn_dists);
	
	
	
	
	return (0);
}

```
