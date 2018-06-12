# clustering

## 1. Eucliean Cluster 



> 중요 : colorlist is a static variable you should define it outside the function getcolorlist, I defined it after if _name == ‘__main‘: get_color_list.color_list = []

```python



def do_euclidean_clustering(white_cloud):
    '''
    :param cloud_objects:
    :return: cluster cloud and cluster indices
    '''
    tree = white_cloud.make_kdtree()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(20000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    return cluster_cloud,cluster_indices
    
# Euclidean Clustering


white_cloud= XYZRGB_to_XYZ(cloud_objects)
cluster_cloud,cluster_indices = do_euclidean_clustering(white_cloud)
```



---


Yuchao's blogspot
```python
# Euclidean Clustering
white_cloud = XYZRGB_to_XYZ(white_cloud) # <type 'pcl._pcl.PointCloud'>
tree = white_cloud.make_kdtree() # <type 'pcl._pcl.KdTree'>
ec = white_cloud.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance(0.02) # for hammer
ec.set_MinClusterSize(10)
ec.set_MaxClusterSize(250)
ec.set_SearchMethod(tree)
cluster_indices = ec.Extract() # indices for each cluster (a list of lists)
# Assign a color to each cluster
cluster_color = get_color_list(len(cluster_indices))
color_cluster_point_list = []
for j, indices in enumerate(cluster_indices):
for i, indice in enumerate(indices):
color_cluster_point_list.append([white_cloud[indice][0], white_cloud[indice][1], white_cloud[indice][2], rgb_to_float(cluster_color[j])])
# Create new cloud containing all clusters, each with unique color
cluster_cloud = pcl.PointCloud_PointXYZRGB()
cluster_cloud.from_list(color_cluster_point_list)
# publish to cloud
ros_cluster_cloud = pcl_to_ros(cluster_cloud)
# save to local
pcl.save(cluster_cloud, 'cluster.pcd')
```

분리수행

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


cluster_indices, white_cloud = euclid_cluster(white_cloud)
get_color_list.color_list = []


```





잡음 포인트 : 하얀색 / 핵심 샘플은 크게 / 경계 포인트는 작게



eps 증가 -> 하나의 클러스터에 더 많은 포인트 포함 -> 클러스터가 커지게 하지만 여러 클러스터 하나로 합친다.



min_samples 증가 -> 핵심 포인트 수가 줄어 들며 -> 잡음 포인트 증가



eps 매개변수는 가까운 포인트의 범위를 결정하기 때문에 더 중요



eps 줄이면 -> 어떤 포인트든 핵심 포인트 x -> 모든 포인트 잡음 됨



eps 크게하면 -> 모든 포인트가 단 하나의 클러스터에 속하게 됨



min_samples - 덜 조밀한 지역에 있는 포인트들이 잡음 포인트가 될 것인지, 아니면 하나의 클러스터가 될 것인지 결정하는 역할



min_samples 증가 -> min_samples의 수보다 작은 클러스터들은 잡음 포인트가 된다.



min_samples : 클러스터의 최소 크기 정한다.



dbscan : 클러스터 개수 지정 x ->  적절한 eps 로 정한다.



적절한 eps 정하는 방법 : 1. StandardScaler 2. MinMaxScaler