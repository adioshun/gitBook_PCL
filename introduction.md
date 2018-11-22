API설명 상세 : https://nlesc.github.io/python-pcl/

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


