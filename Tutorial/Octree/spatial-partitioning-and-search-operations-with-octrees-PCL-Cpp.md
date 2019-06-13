# [Spatial Partitioning and Search Operations with Octrees](http://pointclouds.org/documentation/tutorials/octree.php#octree-search)

옥트리는 3D데이터 처리를 위한 트리형태의 데이터 구조체이다. `An octree is a tree-based data structure for managing sparse 3-D data. `

각 내부 노드는 8개의 자식 노드를 가진다. `Each internal node has exactly eight children. `

In this tutorial we will learn how to use the octree for spatial partitioning and neighbor search within pointcloud data. 

Particularly, we explain how to perform a “Neighbors within Voxel Search”, the “K Nearest Neighbor Search” and “Neighbors within Radius Search”.


## Additional Details

PCL에는 여러개의 Octree가 존재 하며 leaf노드의 특징에 따라 다르다. `Several octree types are provided by the PCL octree component. They basically differ by their individual leaf node characteristics.`

- OctreePointCloudPointVector (equal to OctreePointCloud): This octree can hold a list of point indices at each leaf node.

- OctreePointCloudSinglePoint: This octree class hold only a single point indices at each leaf node. Only the most recent point index that is assigned to the leaf node is stored.

- OctreePointCloudOccupancy: This octree does not store any point information at its leaf nodes. It can be used for spatial occupancy checks.

- OctreePointCloudDensity: This octree counts the amount of points within each leaf node voxel. It allows for spatial density queries.

빠른 속도를 원한다면,  If octrees needs to be created at high rate, please have a look at the octree double buffering implementation ( Octree2BufBase class ). 
- This class keeps two parallel octree structures in the memory at the same time. 
- 배경탐지에도 활용된다. In addition to search operations, this also enables spatial change detection. 
- Furthermore, an advanced memory management reduces memory allocation and deallocation operations during the octree building process. 
- The double buffering octree implementation can be assigned to all OctreePointCloud classes via the template argument “OctreeT”.

All octrees support serialization and deserialization of the octree structure and the octree data content.

---

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


