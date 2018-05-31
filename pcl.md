# PCL

[Community](http://www.pcl-users.org/), [설치(gitbook)](https://adioshun.gitbooks.io/system_setup/content/08a-pcl-setup.html)

![image](https://user-images.githubusercontent.com/17797922/39513568-cc50661e-4dc2-11e8-8e9e-0e9e1e5747ad.png)

> PCL API Documentation [Html](http://docs.pointclouds.org/trunk/), [pdf](http://www.pointclouds.org/assets/pdf/pcl_icra2011.pdf)

## 1. Modules (Library)

Module common : common data structures, computing distances/norms, means and covariances, angular conversions, geometric transformations

[Module filters](http://docs.pointclouds.org/trunk/group__filters.html): outlier and noise removal mechanisms (eg. PassThrough, voxel grid)
- [How 3D Features work in PCL](http://pointclouds.org/documentation/tutorials/how_features_work.php)
- [Filtering a PointCloud using a PassThrough filter](http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough): simple filtering along a specified dimension – that is, cut off values that are either inside or outside a given user range.
- [Downsampling a PointCloud using a VoxelGrid filter](http://pointclouds.org/documentation/tutorials/voxel_grid.php#voxelgrid): reduce the number of points – a point cloud dataset, using a voxelized grid approach

- [Projecting points using a parametric model](http://pointclouds.org/documentation/tutorials/project_inliers.php#project-inliers): project points onto a parametric model (e.g., plane, sphere, etc)
- [Extracting indices from a PointCloud](http://pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices): extract a subset of points by a **segmentation algorithm**
- [Removing outliers using a StatisticalOutlierRemoval filter](http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal): remove noisy measurements, e.g. outliers

- [Removing outliers using a Conditional or RadiusOutlier removal](http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers): remove outliers by  ConditionalRemoval filter & RadiusOutlierRemoval filter 


[Module features](http://docs.pointclouds.org/trunk/group__features.html): data structures and mechanisms for 3D feature estimation from point cloud data

[Module keypoints](http://docs.pointclouds.org/trunk/group__keypoints.html): keypoint detection algorithms 

[Module registration](http://docs.pointclouds.org/trunk/group__registration.html):  Combining several datasets into a global consistent model is usually performed using a technique called **registration**

[Module kdtree](http://docs.pointclouds.org/trunk/group__kdtree.html): kd-tree data-structure, Allows for fast nearest neighbor searches. 

[Module octree](http://docs.pointclouds.org/trunk/group__octree.html): efficient methods for creating a hierarchical tree data structure from point cloud data
- [Point Cloud Compression](http://pointclouds.org/documentation/tutorials/compression.php#octree-compression): 
- The `pcl_octree` implementation provides efficient nearest neighbor search routines, such as 
    - "Neighbors within Voxel Search”, 
    - “K Nearest Neighbor Search” 
    - “Neighbors within Radius Search”. 

[Module segmentation](http://docs.pointclouds.org/trunk/group__segmentation.html):  algorithms for segmenting a point cloud into distinct clusters
- [Euclidean Cluster Extraction](http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction)
- [Plane model segmentation](http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation)


[Module sample_consensus](http://docs.pointclouds.org/trunk/group__sample__consensus.html): SAmple Consensus (SAC) methods like `RANSAC` / models like `planes` and `cylinders`
- [How to use Random Sample Consensus model](http://pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus)


[Module surface](http://docs.pointclouds.org/trunk/group__surface.html): deals with reconstructing the original surfaces from 3D scans(eg. hull, a mesh representation or a smoothed/resampled)

[Module recognition](http://docs.pointclouds.org/trunk/group__recognition.html): algorithms used for Object Recognition 
- [3D Object Recognition based on Correspondence Grouping](http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping):use **Correspondence Grouping algorithms** in order to **cluster** the set of point-to-point correspondences 

[Module io](http://docs.pointclouds.org/trunk/group__io.html): reading and writing point cloud data (PCD) files
- The PCD (Point Cloud Data) file format
- Reading PointCloud data from PCD files
- Writing PointCloud data to PCD files
- The OpenNI Grabber Framework in PCL
- Grabbing point clouds from Ensenso cameras

[Module visualization](http://docs.pointclouds.org/trunk/group__visualization.html): visualize the results


Module Range Image : depth map
- [How to create a range image from a point cloud](http://pointclouds.org/documentation/tutorials/range_image_creation.php#range-image-creation)
- [How to extract borders from range images](http://pointclouds.org/documentation/tutorials/range_image_border_extraction.php#range-image-border-extraction): ?NOT Lidar data?


Module common : common data structures, computing distances/norms, means and covariances, angular conversions, geometric transformations

Module search : methods for searching for nearest neighbors using different data structures


## 2. Tools

###### Official Tools of PCL


- pcl_viewer: a quick way for visualizing PCD (Point Cloud Data) files

- pcd2ply: converts PCD (Point Cloud Data) files to the PLY format

- octree_viewer: allows the visualization of octrees


###### PCl Tools

`sudo apt-get install pcl-tools` [[github]](https://github.com/ryanfb/pcl-tools)
- pcd2ply - convert PCD files to PLY files
- ply2pcd - convert PLY files to PCD files
- statistical_removal - statistical outliers removal
- pcd_viewer - PCL PCD viewer with added 
- pcdnormal2ply - convert PCD files with normals to PLY files
- normal_estimation_omp - estimate normals in a PCD using PCL's OpenMP implementation


###### pcl_helper.py

[github](https://github.com/udacity/RoboND-Perception-Exercises#documentation-for-pcl_helperpy), [Code](https://github.com/udacity/RoboND-Perception-Exercises/tree/master/Exercise-2/sensor_stick/scripts)



`random_color_gen()` : Generates a random set of r,g,b values
- Return: a 3-tuple with r,g,b values (range 0-255)

`ros_to_pcl(sensor_msgs/PointCloud2)` : Converts sensor_msgs/PointCloud2 to XYZRGB Point Cloud
- Return: pcl.PointCloud_PointXYZRGB

`pcl_to_ros(pcl.PointCloud_PointXYZRGB)`: Converts XYZRGB Point Cloud to sensor_msgs/PointCloud2
- Return: sensor_msgs/PointCloud2

`XYZRGB_to_XYZ(XYZRGB_cloud)`: Converts XYZRGB Point Cloud to XYZ Point CLoud
- Return: pcl.PointCloud

`XYZ_to_XYZRGB(XYZ_cloud, color)`:Takes a 3-tuple as color and adds it to XYZ Point Cloud
- Return: pcl.PointCloud_PointXYZRGB

`rgb_to_float(color)`:Converts 3-tuple color to a single float32
- Return: rgb packed as a single float32

`get_color_list(cluster_count)` : Creates a list of 3-tuple (rgb) with length of the list = cluster_count
- Return: get_color_list.color_list








## 3. Projects

- [Cluster Recognition and 6DOF Pose Estimation using VFH descriptors](http://pointclouds.org/documentation/tutorials/vfh_recognition.php#vfh-recognition)

- [Detecting people and their poses using PointCloud Library](http://pointclouds.org/documentation/tutorials/gpu_people.php#gpu-people): RGB-D??

- [RoboND-Perception-Exercises](https://github.com/udacity/RoboND-Perception-Exercises): Udacity Nanodegree program, python

---
## PCD 읽기




```python
import pcl
import numpy as np
p = pcl.PointCloud(np.array([[1, 2, 3], [3, 4, 5]], dtype=np.float32))
seg = p.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
indices, model = seg.segment()
```


```python
import pcl
p = pcl.PointCloud()
p.from_file("table_scene_lms400.pcd")
fil = p.make_statistical_outlier_filter()
fil.set_mean_k (50)
fil.set_std_dev_mul_thresh (1.0)
fil.filter().to_file("inliers.pcd")
```


> [How to extract depth information from the 3D point cloud data?](https://stackoverflow.com/questions/42430479/how-to-extract-depth-information-from-the-3d-point-cloud-data) : 포인트클라우드에서 depth 계산법 

---

# The Point Cloud Data

PCD files contain two sections:

- a human-readable header that defines the number, size, dimensionality and data type of the point cloud;
- a data section which can be in ASCII or a binary, non-human-readable format.





[The PCD (Point Cloud Data) file format](http://pointclouds.org/documentation/tutorials/pcd_file_format.php)

```
# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH 213
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS 213
DATA ascii
0.93773 0.33763 0 4.2108e+06
0.90805 0.35641 0 4.2108e+06
0.81915 0.32 0 4.2108e+06
0.97192 0.278 0 4.2108e+06
0.944 0.29474 0 4.2108e+06
0.98111 0.24247 0 4.2108e+06
0.93655 0.26143 0 4.2108e+06
0.91631 0.27442 0 4.2108e+06
0.81921 0.29315 0 4.2108e+06
0.90701 0.24109 0 4.2108e+06
0.83239 0.23398 0 4.2108e+06
0.99185 0.2116 0 4.2108e+06
0.89264 0.21174 0 4.2108e+06
```

## 1. 개요 

> [Ronny Restrepo](http://ronny.rest/tutorials/module/pointclouds_01/point_cloud_data/)

|The Point Cloud Data|이미지 데이터와 비교 |
|-|-|
|![](http://i.imgur.com/Bc13san.png)|![](http://i.imgur.com/smzFU5N.png)|



- 포인트 클라우드 데이터는 보통 `N x 3` Numpy 배열로 표현 된다. 
    - 각 N 줄은 하나의 점과 맵핑이 되며 
    - 3(x,y,z) 정보를 가지고 있다. 

- Lidar 센서에서 수집한 정보의 경우는 `reflectance`라는 정보가 추가되어 `N x 4` Numpy 배열이 된다. 
    - reflectance : 반사 시간 정보로 보면 된다. 


이미지 데이터
- 항상 양수 이다. 
- 기준점은 왼쪽 위부터 이다. 
- 좌표값은 정수(integer)이다. 

포인트 클라우드 데이터 
- 양수/음수 이다. 
- 좌표값은 real numbered이다. 
- The positive x axis represents forward.
- The positive y axis represents left.
- The positive z axis represents up.

## 2. Creating Birdseye View of Point Cloud Data

> 참고 : Height의 Level별 값 추출 (Height as Channels), [Creating Height Slices of Lidar Data](http://ronny.rest/blog/post_2017_03_27_lidar_height_slices/)

In order to create a birds eye view image, the relevant axes from the point cloud data will be the x and y axes.

![](http://i.imgur.com/cHsb48Y.png)

조심해야 할점 
- the x, and y axes mean the opposite thing.
- The x, and y axes point in the opposite direction.
- You have to shift the values across so that (0,0) is the smallest possible value in the image.


|- [Creating Birdseye View of Point Cloud Data 코드 및 설명(python)](http://ronny.rest/blog/post_2017_03_26_lidar_birds_eye/), [gist](https://gist.github.com/adioshun/12873804f472080c612e506310674797)|
|-|

> [참고] cpp로 작성한 코드 : [mjshiggins's github](https://github.com/mjshiggins/ros-examples)



## 3. Creating 360 degree Panoramic Views

- Project the `points in 3D` space into `cylindrical surface`

- LiDAR센서의 특징에 따라 설정값이 달라 진다. 
    - `h_res`: Horizontal resolution
    - `v_res`: vertical resolution

```python
# KTTI dataset = Velodyne HDL 64E 
## A vertical field of view of 26.9 degrees, at a resolution of 0.4 degree intervals. The vertical field of view is broken up into +2 degrees above the sensor, and -24.9 degrees below the sensor.
## A horizontal field of view of 360 degrees, at a resolution of 0.08 - 0.35 (depending on the rotation rate)
## Rotation rate can be selected to be betwen 5-20Hz.
## http://velodynelidar.com/docs/datasheet/63-9194%20Rev-E_HDL-64E_S3_Spec%20Sheet_Web.pdf

# Resolution and Field of View of LIDAR sensor
h_res = 0.35         # horizontal resolution, assuming rate of 20Hz is used 
v_res = 0.4          # vertical res
v_fov = (-24.9, 2.0) # Field of view (-ve, +ve) along vertical axis
v_fov_total = -v_fov[0] + v_fov[1] 
```

|- [Creating 360 degree Panoramic Views코드 및 설명(matplotlib)](http://ronny.rest/blog/post_2017_03_25_lidar_to_2d/)<br>- [Creating 360 degree Panoramic Views코드 및 설명(numpy)](http://ronny.rest/blog/post_2017_04_03_point_cloud_panorama/)|
|-|




---
# LAS 포맷

- [정의](https://www.asprs.org/committee-general/laser-las-file-format-exchange-activities.html)
- the most commonly used binary point data exchange format
- The LAS file format is a public file format for the interchange of 3-dimensional point cloud data data between data users
- supports the exchange of any 3-dimensional x,y,z tuplet
- [What Lidar processing tools are available in Python?](https://gis.stackexchange.com/questions/88322/what-lidar-processing-tools-are-available-in-python)
- 항공용 Lidar 데이터에서 주로 사용하는지 확인 필요

# pcap to LAS

> LAS format - the most commonly used binary point data exchange format

- (1) I suggest you create many many txt files containing the points in xyz or xyzi a layout (i = intensity). Maybe one file per driveline. (벨로뷰의 csv저장 기능 활용)
- (2) Then you convert each of them to the LAZ format with txt2las using the '-parse xyz' or the '-parse xyzi' option.
- (3) Then you tile the resulting LAZ files into buffered tiles that have less than, say 10 million, points per tile with lastile. `lastile -v -i drivelines/*.laz -merged -tile_size 100 -buffer 5 -odir raw_tiles -o gmu.laz`
- (4) Then you refine tiles to 10 million points per tile if needed
`lastile -v -i raw_tiles/*.laz -refine_tiles 10000000 -cores 4`
- (5) And then - starting from the tiles in the tiles_raw folder - you run one of those many tile-based batch processing pipelines for ground classification and DTM generation described previously in numerous tutorials, videos, and forum entries.









