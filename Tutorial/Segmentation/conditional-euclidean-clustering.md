# [Conditional Euclidean Clustering](http://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.php#conditional-euclidean-clustering)

> https://github.com/strawlab/python-pcl/blob/master/examples/official/Segmentation/conditional_euclidean_clustering_172.txt

Conditional Euclidean segmentation works the same way as the standard one seen above, with one exception. 

Apart from the distance check, points need also to meet a special, custom requirement for them to be added to a cluster.

This condition is user-specified. It boils down to this: for every pair of points (the first one, the seed, is being processed in this moment, the second one, candidate, is a neighbor of the former that is being tested) a custom function will be called. 

This function has a certain signature: it receives

1. a copy of the two points so we can perform our own tests, and
2. the squared distance

and returns a boolean value. If the value is true, the candidate may be added to the cluster. If false, it will not, even if it passed the distance check.

The condition that is implemented above (checking if the candidate's Y coordinate is lower than the seed's) does not make a lot of sense, but I just wanted you to understand how the method works.


---
This tutorial describes how to use the  `pcl::ConditionalEuclideanClustering`  class: 
- A segmentation algorithm that clusters points based on Euclidean distance and a user-customizable condition that needs to hold.

This class uses the same greedy-like / region-growing / flood-filling approach that is used in  [Euclidean Cluster Extraction],  [Region growing segmentation]  and  [Color-based region growing segmentation]. 

장점 : The advantage of using this class over the other classes is that the constraints for clustering (pure Euclidean, smoothness, RGB) are now customizable by the user. 

단점 Some disadvantages include: 
- no initial seeding system, 
- no over- and under-segmentation control, 
- and the fact that calling a conditional function from inside the main computational loop is less time efficient.


# Theoretical Primer

The  [Euclidean Cluster Extraction]  and  [Region growing segmentation]  tutorials already explain the region growing algorithm very accurately. 

The only addition to those explanations is that the condition that needs to hold for a neighbor to be merged into the current cluster, can now be fully customized.

As a cluster grows, it will evaluate the user-defined condition between points already inside the cluster and nearby candidate points. 

The candidate points (nearest neighbor points) are found using a Euclidean radius search around each point in the cluster. 

For each point within a resulting cluster, the condition needed to hold with at least one of its neighbors and NOT with all of its neighbors.

The Conditional Euclidean Clustering class can also automatically filter clusters based on a size constraint. 

The clusters classified as too small or too large can still be retrieved afterwards.



--- 

```python

import pcl
cloud_in = pcl.load_XYZI('./examples/pcldata/tutorials/Statues_4.pcd')

# // Downsample the cloud using a Voxel Grid class
vg = cloud_in.make_VoxelGrid()
vg.set_LeafSize (80.0, 80.0, 80.0)
vg.set_DownsampleAllData (True)
cloud_out = vg.filter ()
print('>> Done: ' + tt.toc () + ' ms, ' + cloud_out.size + ' points\n')


# // Set up a Normal Estimation class and merge data in cloud_with_normals
ne = cloud_out.make_NormalEstimation()
ne.set_SearchMethod (search_tree);
ne.set_RadiusSearch (300.0)
cloud_with_normals = ne.compute ()
print(">> Done: ' + ' ms\n')

#코드 미완성?
```
