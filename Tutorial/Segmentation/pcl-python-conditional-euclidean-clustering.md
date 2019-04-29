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
