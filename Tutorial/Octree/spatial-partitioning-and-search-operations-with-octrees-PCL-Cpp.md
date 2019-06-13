# [Spatial Partitioning and Search Operations with Octrees](http://pointclouds.org/documentation/tutorials/octree.php#octree-search)

옥트리는 3D데이터 처리를 위한 트리형태의 데이터 구조체이다. `An octree is a tree-based data structure for managing sparse 3-D data. `

각 내부 노드는 8개의 자식 노드를 가진다. `Each internal node has exactly eight children. `

In this tutorial we will learn how to use the octree for spatial partitioning and neighbor search within pointcloud data. 

Particularly, we explain how to perform a “Neighbors within Voxel Search”, the “K Nearest Neighbor Search” and “Neighbors within Radius Search”.

## Code 


### 1. 옥트리 구조체 생성 


```cpp
float resolution = 128.0f;

pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);

octree.setInputCloud (cloud);
octree.addPointsFromInputCloud ();
 ```

L2 : Then we create an octree instance which is initialized with its resolution. 
	- This octree keeps a vector of point indices within its leaf nodes. 
	- The **resolution parameter** describes the length of the smallest voxels at lowest octree level. 
	- The depth of the octree is therefore a function of the resolution as well as the spatial dimension of the pointcloud. 
	- If a bounding box of the pointcloud is know, it should be assigned to the octree by using the `defineBoundingBox` method. 


L3~L4 : Then we assign a pointer to the PointCloud and add all points to the octree.



### 2. Neighbors within Voxel Search 

searchPoint와 동일한 복셀에 속하는 점들의 inx값 탐지 

```cpp
  std::vector<int> pointIdxVec;

  if (octree.voxelSearch (searchPoint, pointIdxVec))
  {
    std::cout << "Neighbors within voxel search at (" << searchPoint.x 
     << " " << searchPoint.y 
     << " " << searchPoint.z << ")" 
     << std::endl;
              
    for (size_t i = 0; i < pointIdxVec.size (); ++i)
   std::cout << "    " << cloud->points[pointIdxVec[i]].x 
       << " " << cloud->points[pointIdxVec[i]].y 
       << " " << cloud->points[pointIdxVec[i]].z << std::endl;
  }
```

점군과 Octree를 연동한 후에는 검색 작업이 가능하다. `Once the PointCloud is associated with an octree, we can perform search operations.`

The fist search method used here is “Neighbors within Voxel Search”. 
- It assigns the search point to the corresponding leaf node voxel and returns a vector of point indices. 
- These indices relate to points which fall within the same voxel. 
- The distance between the search point and the search result depend therefore on the resolution parameter of the octree.

### 3. K nearest neighbor search

searchPoint에서 가까운 K개의 점과 거리 정보 

```cpp

  // K nearest neighbor search

  int K = 10;

  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;

  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

  if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
  {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
  }
 
 ```
 
Next, a K nearest neighbor search is demonstrated. In this example, K is set to 10. 

탐색 결과를 두개의 벡터에 저장 `The “K Nearest Neighbor Search” method writes the search results into two separate vectors. `
- The first one, `pointIdxNKNSearch`, 탐색 결과  will contain the search result (indices referring to the associated PointCloud data set). 
- 이웃 점과의 거리 `The second vector holds corresponding squared distances between the search point and the nearest neighbors.`


### 4. Neighbors within Radius Search


searchPoint에서 지정된 반경(`radius`) 안의 점과 리 정보 

```cpp
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

  std::cout << "Neighbors within radius search at (" << searchPoint.x 
      << " " << searchPoint.y 
      << " " << searchPoint.z
      << ") with radius=" << radius << std::endl;


  if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
  {
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  }
```

The “Neighbors within Radius Search” works very similar to the “K Nearest Neighbor Search”. 

탐색 결과를 두개의 벡터에 저장 `Its search results are written to two separate vectors describing point indices and squares search point distances.`


---
# [PCL Series 3 - Use of Octree- Search ](https://blog.csdn.net/qq_22170875/article/details/84844385)

```cpp

#include <pcl/io/pcd_io.h>  //文件输入输出
#include <pcl/octree/octree_search.h>  //octree相关定义
#include <pcl/visualization/cloud_viewer.h>  //vtk可视化相关定义
#include <pcl/point_types.h>  //点类型相关定义

#include <iostream>
#include <vector>

int main()
{
	//1.读取点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}

	//2.原始点云着色
	for (size_t i = 0; i < cloud->points.size(); ++i){
		cloud->points[i].r = 255;
		cloud->points[i].g = 255;
		cloud->points[i].b = 255;
	}

	//3.创建Octree实例对象
	float resolution = 0.03f;  //设置octree体素分辨率
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution); //建立octree对象
	octree.setInputCloud(cloud); //传入需要建立kdtree的点云指针
	octree.addPointsFromInputCloud();  //构建Octree

	//3.1.体素内近邻搜索
	pcl::PointXYZRGB searchPoint1 = cloud->points[1250]; //设置查找点
	std::vector<int> pointIdxVec;  //保存体素近邻搜索的结果向量
	if (octree.voxelSearch(searchPoint1, pointIdxVec))
	{
		std::cout << "Neighbors within voxel search at (" << searchPoint1.x
			<< " " << searchPoint1.y
			<< " " << searchPoint1.z << ")"
			<< std::endl;

		//给查找到的近邻点设置颜色
		for (size_t i = 0; i < pointIdxVec.size(); ++i){
			cloud->points[pointIdxVec[i]].r = 255;
			cloud->points[pointIdxVec[i]].g = 0;
			cloud->points[pointIdxVec[i]].b = 0;
		}		
	}

	//3.2.K近邻搜索
	pcl::PointXYZRGB searchPoint2 = cloud->points[3000]; //设置查找点
	int K = 200;
	std::vector<int> pointIdxNKNSearch; //保存K近邻点的索引结果
	std::vector<float> pointNKNSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方

	std::cout << "K nearest neighbor search at (" << searchPoint2.x
		<< " " << searchPoint2.y
		<< " " << searchPoint2.z
		<< ") with K=" << K << std::endl;

	if (octree.nearestKSearch(searchPoint2, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{   //给查找到的近邻点设置颜色
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
			cloud->points[pointIdxNKNSearch[i]].r = 0;
			cloud->points[pointIdxNKNSearch[i]].g = 255;
			cloud->points[pointIdxNKNSearch[i]].b = 0;
		}	
	}
	std::cout << "K = 200近邻点个数：" << pointIdxNKNSearch.size() << endl;

	//3.3.半径内近邻搜索
	pcl::PointXYZRGB searchPoint3 = cloud->points[6500]; //设置查找点
	std::vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
	std::vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
	float radius = 0.02; //设置查找半径范围

	std::cout << "Neighbors within radius search at (" << searchPoint3.x
		<< " " << searchPoint3.y
		<< " " << searchPoint3.z
		<< ") with radius=" << radius << std::endl;

	if (octree.radiusSearch(searchPoint3, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{    //给查找到的近邻点设置颜色
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
			cloud->points[pointIdxRadiusSearch[i]].r = 0;
			cloud->points[pointIdxRadiusSearch[i]].g = 0;
			cloud->points[pointIdxRadiusSearch[i]].b = 255;
		}		
	}
	std::cout << "半径0.02近邻点个数： " << pointIdxRadiusSearch.size() << endl;

	//4.显示点云
	pcl::visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(cloud);

	system("pause");
	return 0;
}

```

![](https://img-blog.csdnimg.cn/20181205215523236.JPG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIyMTcwODc1,size_16,color_FFFFFF,t_70)