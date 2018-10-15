# Spatial Partitioning and Search Operations with Octrees

    Cpp : http://pointclouds.org/documentation/tutorials/octree.php#octree-search    
    python : https://github.com/strawlab/python-pcl/blob/master/examples/official/octree/octree_search.py

An octree is a tree-based data structure for managing sparse 3-D data. 

Each internal node has exactly eight children. 

In this tutorial we will learn how to use the octree for spatial partitioning and neighbor search within pointcloud data. 

Particularly, we explain how to perform a “Neighbors within Voxel Search”, the “K Nearest Neighbor Search” and “Neighbors within Radius Search”.