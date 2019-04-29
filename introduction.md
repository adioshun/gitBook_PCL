API설명 상세 : https://nlesc.github.io/python-pcl/

[Community](http://www.pcl-users.org/), [설치(gitbook)](https://adioshun.gitbooks.io/system_setup/content/08a-pcl-setup.html)

![image](https://user-images.githubusercontent.com/17797922/39513568-cc50661e-4dc2-11e8-8e9e-0e9e1e5747ad.png)

> PCL API Documentation [Html](http://docs.pointclouds.org/trunk/), [pdf](http://www.pointclouds.org/assets/pdf/pcl_icra2011.pdf)

## 1. Modules (Library)

pcl_filters - 3D 점군 데이터에서 이상값과 노이즈 제거 등의 필터링
pcl_features - 점군 데이터로부터 3D 특징 추정 (feature estimation) 을 위한 수많은 자료 구조와 방법들 (surface normals, curvatures, boundary point estimation, moment invariants, principal curvatures, PFH & FPFH descriptors, spin images, integral images, NARF descriptors, RIFT, RSD, VFH, SHOT 등)
pcl_keypoints - Keypoint (or interest point) 을 검출하는 알고리즘 구현 (BRISK, Harris Corner, NARF, SIFT, SUSAN 등)
pcl_registration - 여러 데이터셋을 합쳐 큰 모델로 만드는 registration 작업 (ICP 등)
pcl_kdtree - 빠른 최근거리 이웃을 탐색하는 FLANN 라이브러리를 사용한 kdtree 자료 구조
pcl_octree - 점군 데이터로부터 계층 트리 구조를 구성하는 방법
pcl_segmentation - 점군으로부터 클러스터들로 구분하는 알고리즘들
pcl_sample_consensus - 선, 평면, 실린더 등의 모델 계수 추정을 위한 RANSAC 등의 알고리즘들
pcl_surface - 3D 표면 복원 기법들 (meshing, convex hulls, Moving Least Squares 등)
pcl_range_image - range image (or depth map) 을 나타내고 처리하는 방법
pcl_io - OpenNI 호환 depth camera 로부터 점군 데이터를 읽고 쓰는 방법
pcl_visualization - 3D 점군 데이터를 처리하는 알고리즘의 결과를 시각화
[출처] PCL 설치 및 테스트 (키넥트 개발 Open Kinect 코리아 커뮤니티) |작성자 매발톱



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


