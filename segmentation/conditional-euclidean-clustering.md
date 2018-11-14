# [Conditional Euclidean Clustering](http://pointclouds.org/documentation/tutorials/conditional_euclidean_clustering.php#conditional-euclidean-clustering)

> https://github.com/strawlab/python-pcl/blob/master/examples/official/Segmentation/conditional_euclidean_clustering_172.txt


가장 간단한 방법으로 두 점사이의 거리 정보를 이용한다. `Euclidean segmentation is the simplest of all. It checks the distance between two points. `
- 거리가 특정 값보다 작다면 같은 클러스터로 간주 한다. `If it is less than a threshold, both are considered to belong in the same cluster. `

It works like a flood fill algorithm:
1. a point in the cloud is "marked" as "chosen" for the cluster.
2. Then, it spreads like a virus to all other points that are near enough, and from those to even more points, until none new can be added.
3. Then, a new cluster is initialized, and the procedure starts again with the remaining unmarked points.


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
