# [Spatial change detection on unorganized point cloud data](http://pointclouds.org/documentation/tutorials/octree_change.php#octree-change-detection)

An octree is a tree-based data structure for organizing sparse 3-D data. 

In this tutorial we will learn how to use the octree implementation for detecting spatial changes between multiple unorganized point clouds which could vary in size, resolution, density and point ordering. 

반복적으로 Octree구조를 비교 하여 변경점 탐지 `By recursively comparing the tree structures of octrees, spatial changes represented by differences in voxel configuration can be identified. `

성능 향상을 위해 **double buffering** 사용 `Additionally, we explain how to use the pcl octree “double buffering” technique allows us to efficiently process multiple point clouds over time.`

## code 

```cpp
// Add points from cloudA to octree
octree.setInputCloud (cloudA);
octree.addPointsFromInputCloud ();

// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
octree.switchBuffers ();

// Add points from cloudB to octree
octree.setInputCloud (cloudB);
octree.addPointsFromInputCloud ();

std::vector<int> newPointIdxVector;

// Get vector of point indices from octree voxels which did not exist in previous buffer
octree.getPointIndicesFromNewVoxels (newPointIdxVector);

```

Point cloud cloudA is our reference point cloud and the octree structure describe its spatial distribution.

 
The class OctreePointCloudChangeDetector inherits from class Octree2BufBase which enables to keep and manage two octrees in the memory at the same time. 

In addition, it implements a memory pool that reuses already allocated node objects and therefore reduces expensive memory allocation and deallocation operations when generating octrees of multiple point clouds. 

By calling “octree.switchBuffers()”, we reset the octree class while keeping the previous octree structure in memory.

