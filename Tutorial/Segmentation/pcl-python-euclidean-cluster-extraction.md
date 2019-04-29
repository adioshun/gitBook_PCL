# [Euclidean Cluster Extraction](http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction)


> https://github.com/strawlab/python-pcl/blob/master/examples/official/Segmentation/cluster_extraction.py

가장 간단한 방법으로 두 점사이의 거리 정보를 이용한다. `Euclidean segmentation is the simplest of all. It checks the distance between two points. `
- 거리가 특정 값보다 작다면 같은 클러스터로 간주 한다. `If it is less than a threshold, both are considered to belong in the same cluster. `

It works like a flood fill algorithm:
1. a point in the cloud is "marked" as "chosen" for the cluster.
2. Then, it spreads like a virus to all other points that are near enough, and from those to even more points, until none new can be added.
3. Then, a new cluster is initialized, and the procedure starts again with the remaining unmarked points.





```python 
import numpy as np
import pcl


cloud = pcl.load('./examples/pcldata/tutorials/table_scene_lms400.pcd')

#   // Create the filtering object: downsample the dataset using a leaf size of 1cm
vg = cloud.make_voxel_grid_filter()
vg.set_leaf_size (0.01, 0.01, 0.01)
cloud_filtered = vg.filter ()

#   // Create the segmentation object for the planar model and set all the parameters
seg = cloud.make_segmenter()
seg.set_optimize_coefficients (True)
seg.set_model_type (pcl.SACMODEL_PLANE)
seg.set_method_type (pcl.SAC_RANSAC)
seg.set_MaxIterations (100)
seg.set_distance_threshold (0.02)


i = 0
nr_points = cloud_filtered.size


# Creating the KdTree object for the search method of the extraction
tree = cloud_filtered.make_kdtree()


ec = cloud_filtered.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance (0.02)
ec.set_MinClusterSize (100)
ec.set_MaxClusterSize (25000)
ec.set_SearchMethod (tree)
cluster_indices = ec.Extract()

print('cluster_indices : ' + str(cluster_indices.count) + " count.")
cloud_cluster = pcl.PointCloud()

for j, indices in enumerate(cluster_indices):
    # cloudsize = indices
    print('indices = ' + str(len(indices)))
    # cloudsize = len(indices)
    points = np.zeros((len(indices), 3), dtype=np.float32)
    # points = np.zeros((cloudsize, 3), dtype=np.float32)
    
    # for indice in range(len(indices)):
    for i, indice in enumerate(indices):
        # print('dataNum = ' + str(i) + ', data point[x y z]: ' + str(cloud_filtered[indice][0]) + ' ' + str(cloud_filtered[indice][1]) + ' ' + str(cloud_filtered[indice][2]))
        # print('PointCloud representing the Cluster: ' + str(cloud_cluster.size) + " data points.")
        points[i][0] = cloud_filtered[indice][0]
        points[i][1] = cloud_filtered[indice][1]
        points[i][2] = cloud_filtered[indice][2]

    cloud_cluster.from_array(points)
    ss = "cloud_cluster_" + str(j) + ".pcd";
    pcl.save(cloud_cluster, ss)

```

--- 

# Clustering

> [GitBook 코드 모음](https://adioshun.gitbooks.io/pcl/content/pcl/5-clustering.html)

## 1. 유클리드 클러스터링

> https://github.com/yulivee/RoboND-Perception-Project/blob/master/pr2_robot/scripts/perception_functions.py

```python 

# Euclidean Clustering
def euclid_cluster(cloud):
    white_cloud = XYZRGB_to_XYZ(cloud) # Apply function to convert XYZRGB to XYZ
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(3000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    return cluster_indices, white_cloud


def cluster_mask(cluster_indices, white_cloud):
    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([
                                            white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float( cluster_color[j] )
                                           ])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    return cluster_cloud
```


---

## 참고 

- [[youtube] Real Time Clustering of point clouds using Euclidean Clustering](https://www.youtube.com/watch?v=_suyKDMEdro) : kinetc, rviz 이용 


- [유클리드 거리 기반 3차원 포인트 클라우드 세그먼테이션](http://daddynkidsmakers.blogspot.com/2015/08/3.html)



