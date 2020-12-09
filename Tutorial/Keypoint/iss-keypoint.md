










```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>

int
main(int argc, char** argv)
{
	// Objects for storing the point cloud and the keypoints.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	
  	pcl::io::loadPCDFile<pcl::PointXYZRGB> ("cloud_bin_1.pcd", *cloud);
	std::cout << "size:" << cloud->points.size() << std::endl;

	double resolution = 0.0058329; //computeCloudResolution(cloud); //0.00408779
	std::cout << "resolution:" << resolution<< std::endl; 

	pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector; 
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	iss_detector.setSearchMethod(kdtree);
	iss_detector.setSalientRadius (6 * resolution);//support_radius); 
	iss_detector.setNonMaxRadius (4 * resolution);//nms_radius); 
	iss_detector.setInputCloud (cloud); 
	iss_detector.compute (*keypoints);
	std::cout << "Filtered :" << keypoints->width * keypoints->height  << std::endl;  
}

```

---

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/iss_3d.h>

// This function by Tommaso Cavallari and Federico Tombari, taken from the tutorial
// http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (! pcl_isfinite((*cloud)[i].x))
			continue;

		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	return resolution;
}

int
main(int argc, char** argv)
{
	// Objects for storing the point cloud and the keypoints.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>);
	
  	pcl::io::loadPCDFile<pcl::PointXYZRGB> ("cloud_bin_1.pcd", *cloud);
	std::cout << "size:" << cloud->points.size() << std::endl;

     // 시각적 확인을 위해 색상 통일 (255,255,255)
	for (size_t i = 0; i < cloud->points.size(); ++i){
		cloud->points[i].r = 255;
		cloud->points[i].g = 255;
		cloud->points[i].b = 255;
	}


	// ISS keypoint detector object.
	
	//pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
	
	//detector.setInputCloud(cloud);
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	//detector.setSearchMethod(kdtree);
	double resolution = 0.0058329; //computeCloudResolution(cloud); //0.00408779
	

	std::cout << "resolution:" << resolution<< std::endl; 
	// Set the radius of the spherical neighborhood used to compute the scatter matrix.
	//detector.setSalientRadius(6 * resolution);
	// Set the radius for the application of the non maxima supression algorithm.
	//detector.setNonMaxRadius(4 * resolution);
	// Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
	//detector.setMinNeighbors(5);
	// Set the upper bound on the ratio between the second and the first eigenvalue.
	//detector.setThreshold21(0.975);
	// Set the upper bound on the ratio between the third and the second eigenvalue.
	//detector.setThreshold32(0.975);
	// Set the number of prpcessing threads to use. 0 sets it to automatic.
	//detector.setNumberOfThreads(4);
	
	//detector.compute(*keypoints);



	//pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZ>()); 

	pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector; 
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	iss_detector.setSearchMethod(kdtree);
	iss_detector.setSalientRadius (6 * resolution);//support_radius); 
	iss_detector.setNonMaxRadius (4 * resolution);//nms_radius); 
	iss_detector.setInputCloud (cloud); 
	iss_detector.compute (*keypoints);
	std::cout << "Filtered :" << keypoints->width * keypoints->height  << std::endl;  


	/*
	for (size_t i = 0; i < keypoints.points.size (); ++i)
	{
	keypoints.points[i].r = 255;
	keypoints.points[i].g = 0;
	keypoints.points[i].b = 0;
	}
	*/
	/*
	if (octree.voxelSearch(searchPoint, pointIdxVec))
	{
		//시각적 확인을 위하여 색상 변경 (255,0,0)
		for (size_t i = 0; i < pointIdxVec.size(); ++i){
			cloud->points[pointIdxVec[i]].r = 255;
			cloud->points[pointIdxVec[i]].g = 0;
			cloud->points[pointIdxVec[i]].b = 0;
		}		
	}
  */
	
	


}
```
