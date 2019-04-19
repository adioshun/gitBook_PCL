# Summary

## Point Cloud Lib.

* [README](README.md)
* [what is PointCLoud](what-is-pointcloud.md)
* [References](references.md)

## PCL-Python

* [README](pcl/README.md)
* [0-설치](pcl/installation.md)
* [1-File I/O](pcl/1-file-io.md)
* [2-Numpy](pcl/2-numpy.md)
* [3-Downsampling](pcl/3-downsampling.md)
* [4-Filter](pcl/4-filter.md)
* [5-Clustering](pcl/5-clustering.md)
* [6-Feature](pcl/6-feature.md)
  * [surface\_normal](pcl/6-feature/surfacenormal.md)
* [8- ROS](pcl/8-ros.md)
* [helper](pcl/helper.md)

## PCL-C++

* [README](pcl-cpp/README.md)

## Open3D

* [README](open3d/README.md)
* [0-설치](open3d/installation.md)
* [1-File I/O](open3d/1-file-io.md)
* [2- Numpy ](open3d/2-numpy.md)
* [3-Downsampling](open3d/3-downsampling.md)
* [4-Filtering](open3d/4-filtering.md)
* [9-Visulization](open3d/9-visulization.md)
* [8-ROS](open3d/8-ros.md)

## PCL Tutorial

* [Introduction](introduction.md)
* [Basic\_Usage](basicusage.md)
  * [Using a matrix to transform a point cloud](basicusage/using-a-matrix-to-transform-a-point-cloud.md)
* [I\_O](Tutorial/InputOutput/README.md)
  * [\[PCL-Cpp\] Concatenate the points of two Point Clouds](Tutorial/InputOutput/pcl-cpp-concatenate-the-points-of-two-point-clouds.md)
  * [\[PCL-Cpp\] Reading Writing Point Cloud data to PCD files](Tutorial/InputOutput/pcl-cpp-reading-writing-point-cloud-data-to-pcd-files.md)

---

* [Feature](feature.md)
  * [Estimating Surface Normals in a PointCloud](feature/estimating-surface-normals-in-a-pointcloud.md)
  * [How 3D Features work in PCL](feature/how-3d-features-work-in-pcl.md)
  * [Normal Estimation Using Integral Images](feature/normal-estimation-using-integral-images.md)
  * [Point Feature Histograms \(PFH\) descriptors](feature/point-feature-histograms-pfh-descriptors.md)
  * [Estimating VFH signatures for a set of points](feature/estimating-vfh-signatures-for-a-set-of-points.md)
  * [Moment of inertia and eccentricity based descriptors](feature/moment-of-inertia-and-eccentricity-based-descriptors.md)
  * [RoPs feature](feature/rops-feature.md)
* [Filtering](filtering.md)
  * [Filtering a PointCloud using a PassThrough filter](filtering/filtering-a-pointcloud-using-a-passthrough-filter.md)
  * [Downsampling a PointCloud using a VoxelGrid filter](filtering/downsampling-a-pointcloud-using-a-voxelgrid-filter.md)
  * [Removing outliers using a StatisticalOutlierRemoval filter](filtering/removing-outliers-using-a-statisticaloutlierremoval-filter.md)
  * [Extracting indices from a PointCloud](filtering/extracting-indices-from-a-pointcloud.md)
  * [Removing outliers using a Conditional or Radius Outlier removal](filtering/removing-outliers-using-a-conditional-or-radius-outlier-removal.md)
* [Registration](Tutorial/registration/README.md)
  * [How to use iterative closest point](Tutorial/registration/iterative-closest-point.md)
  * [The PCL Registration API](Tutorial/registration/the-pcl-registration-api.md)
  * [\[PCL-Cpp\] How to use Normal Distributions Transform](Tutorial/registration/how-to-use-normal-distributions-transform.md)
  * [\[Open3D\] ICP registration](Tutorial/registration/open3d-icp-registration.md)
  * [\[Open3D\] Global registration](Tutorial/registration/open3d-global-registration.md)
* [KdTree](kdtree.md)
  * [How to use a KdTree to search](how-to-use-a-kdtree-to-search.md)
* [Octree](octree.md)
  * [Spatial Partitioning and Search Operations with Octrees](octree/spatial-partitioning-and-search-operations-with-octrees.md)
  * [Spatial change detection on unorganized point cloud data](octree/spatial-change-detection-on-unorganized-point-cloud-data.md)
  * [Point Cloud Compression](octree/point-cloud-compression.md)
* [Segmentation](segmentation.md)
  * [DBSCAN\_Cluster](segmentation/dbscancluster.md)
  * [Euclidean Cluster Extraction](segmentation/euclidean-cluster-extraction.md)
  * [Region growing segmentation](segmentation/region-growing-segmentation.md)
  * [Conditional Euclidean Clustering](segmentation/conditional-euclidean-clustering.md)
* Surface
  * [Smoothing and normal estimation based on polynomial reconstruction](smoothing-and-normal-estimation-based-on-polynomial-reconstruction.md)
  * [Fast triangulation of unordered point clouds](fast-triangulation-of-unordered-point-clouds.md)
* [Visualization](visualization.md)
  * [Tool\_CloudCompare](visualization/toolcloudcompare.md)
  * [Tool\_ParaView with PCL](visualization/toolparaview-with-pcl.md)
  * [lib\_k3d](visualization/libk3d.md)
  * [The CloudViewer](visualization/visualizing-point-clouds.md)
  * [PCLVisualizer](visualization/pclvisualizer.md)
  * [PCL Visualization overview](visualization/pcl-visualization-overview.md)
* [Tracking object in real time](tracking-object-in-real-time.md)

## 참고

* [Normal Estimation](normal-estimation.md)
* [RANSAC](ransac.md)
* [CCL](ccl.md)
* [Hough Transform](hough-transform.md)
* [DBSCAN](dbscan.md)
  * [OPTICS](dbscan/optics.md)
* [Numpy](https://legacy.gitbook.com/book/adioshun/python_snippet/edit#/edit/master/packagenumpy.md?_k=s830r0)

## snippet

* [README](snippet/README.md)
* [ROS-to-array-PCD-pickle](snippet/ros-to-array.md)
* [Top-View](snippet/top-view.md)
* [Surrount-view](snippet/surround-view.md)
* [배경제거](snippet/bg-removal.md)

## 다른 패키지들

* [패키지/설치](installation.md)
* [pyPCD\(/w PCL\)](pypcd.md)
  * [1-File I/O](1-file-io.md)
  * [2-Numpy](2-numpy.md)
  * [8-ROS](8-ros.md)
* [ROS](ros.md)
* [pytncloud](pytncloud.md)
  * [1-File I/O](pytncloud/1-file-io.md)
  * [2-Numpy](pytncloud/2-numpy.md)
  * [9-Visualization](pytncloud/9-visualization.md)
* [pyPCL\(/w PCL\)](pypcl.md)
  * [1-File I/O](pypcl/1-file-io.md)
  * [6-Feature](pypcl/6-feature.md)
  * [5-Clustering](pypcl/5-clustering.md)
* [LasPy](laspy.md)
  * [9-Visualization](laspy/9-visualization.md)
* [pyDriver](pydriver.md)
* [ecto\_PCL](ectopcl.md)
* [PCLpy](pclpy.md)
  * [5-Clustering](5-clustering.md)
  * [6-Feature](6-feature.md)

