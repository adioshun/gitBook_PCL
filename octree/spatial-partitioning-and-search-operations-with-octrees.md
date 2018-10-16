# Spatial Partitioning and Search Operations with Octrees

    Cpp : http://pointclouds.org/documentation/tutorials/octree.php#octree-search    
    python : https://github.com/strawlab/python-pcl/blob/master/examples/official/octree/octree_search.py

An octree is a tree-based data structure for managing sparse 3-D data. 

Each internal node has exactly eight children. 

In this tutorial we will learn how to use the octree for spatial partitioning and neighbor search within pointcloud data. 

Particularly, we explain how to perform a “Neighbors within Voxel Search”, the “K Nearest Neighbor Search” and “Neighbors within Radius Search”.





```python
resolution = 0.5#128.0f
octree = cloud.make_octreeSearch(resolution)
octree.add_points_from_input_cloud()
```

Then we create an octree instance which is initialized with its resolution. 
This octree keeps a vector of point indices within its leaf nodes. 
The **resolution parameter** describes the length of the smallest voxels at lowest octree level. 
The depth of the octree is therefore a function of the resolution as well as the spatial dimension of the pointcloud. 
If a bounding box of the pointcloud is know, it should be assigned to the octree by using the defineBoundingBox method. 
Then we assign a pointer to the PointCloud and add all points to the octree.

> The resolution parameter describes the length of the smallest voxels at lowest octree level