# [Euclidean Cluster Extraction](http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction)

In this tutorial we will learn how to extract Euclidean clusters with the pcl::EuclideanClusterExtraction class. 

Plane model segmentation은 다른 곳에서 별도 확인 바람 `In order to not complicate the tutorial, certain elements of it such as the plane segmentation algorithm, will not be explained here. Please check the Plane model segmentation tutorial for more information.`


## Theoretical Primer

A clustering method needs to divide an unorganized point cloud model P into smaller parts so that the overall processing time for P is significantly reduced. 

A simple data clustering approach in an Euclidean sense can be implemented by making use of a 3D grid subdivision of the space using fixed width boxes, or more generally, an octree data structure. 

This particular representation is very fast to build and is useful for situations where either a volumetric representation of the occupied space is needed, or the data in each resultant 3D box (or octree leaf) can be approximated with a different structure. 

In a more general sense however, we can make use of nearest neighbors and implement a clustering technique that is essentially similar to a flood fill algorithm.

Let’s assume we have given a point cloud with a table and objects on top of it. We want to find and segment the individual object point clusters lying on the plane. 

Assuming that we use a Kd-tree structure for finding the nearest neighbors, the algorithmic steps for that would be (from [RusuDissertation]):

> 자세한 수행 절차는 홈페이지 확인 


## code 

```cpp 
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

ec.setClusterTolerance (0.02); // 2cm
ec.setMinClusterSize (100);
ec.setMaxClusterSize (25000);
ec.setSearchMethod (tree);
ec.setInputCloud (cloud_filtered);
ec.extract (cluster_indices);
```

1. Here we are creating a EuclideanClusterExtraction object with point type PointXYZ since our point cloud is of type PointXYZ. 

2. We are also setting the parameters and variables for the extraction. Be careful setting the right value for setClusterTolerance(). 
- 값이 작으면, If you take a very small value, it can happen that an actual object can be seen as multiple clusters. 
- 값이 크면, On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster. 
- 테스트 해보며 찾아야 함. So our recommendation is to just test and try out which value suits your dataset.

최소 포인트수와 최대 포인트 수 지정 We impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points.

Now we extracted the clusters out of our point cloud and saved the indices in cluster_indices. 

To separate each cluster out of the vector<PointIndices> we have to iterate through cluster_indices, create a new PointCloud for each entry and write all points of the current cluster in the PointCloud.