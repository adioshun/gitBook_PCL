# Summary

## Point Cloud Lib.

* [README](README.md)
* [what is PointCLoud](what-is-pointcloud.md)
* [References](references.md)

## PCL-Python

* [README](pcl/README.md)
* [0-설치](pcl/installation.md)
* [2-Numpy](pcl/2-numpy.md)
* [3-Downsampling](pcl/3-downsampling.md)
* [4-Filter](pcl/4-filter.md)
* [5-Clustering](pcl/5-clustering.md)
* [8- ROS](pcl/8-ros.md)
* [helper](pcl/helper.md)

## PCL-C++

* [README](pcl-cpp/README.md)

## Open3D

* [README](Open3D/README.md)
* [0-설치](Open3D/installation.md)
* [2- Numpy ](Open3D/2-numpy.md)
* [3-Downsampling](Open3D/3-downsampling.md)
* [4-Filtering](Open3D/4-filtering.md)
* [8-ROS](Open3D/8-ros.md)
* [9-Visulization](Open3D/9-visulization.md)

## PCL Tutorial

* [README](Tutorial/README.md)
* [Introduction](introduction.md)
* [Basic Usage](Tutorial/Basic-Usage/README.md)
  * [Using a matrix to transform a point cloud](Tutorial/Basic-Usage/using-a-matrix-to-transform-a-point-cloud.md)
* [I\_O](Tutorial/InputOutput/README.md)
  * [\[PCL-Cpp\] Concatenate the points of two Point Clouds](Tutorial/InputOutput/pcl-cpp-concatenate-the-points-of-two-point-clouds.md)
  * [\[PCL-Cpp\] Reading Writing Point Cloud data to PCD files](Tutorial/InputOutput/pcl-cpp-reading-writing-point-cloud-data-to-pcd-files.md)
  * [\[PCL-Python\] Reading Writing Point Cloud data](Tutorial/InputOutput/pcl-python-reading-writing-point-cloud-data.md)
  * [\[Open3D\] Reading Writing Point Cloud data ](Tutorial/InputOutput/open3d-reading-writing-point-cloud-data.md)
* [Feature](feature.md)
  * [Estimating Surface Normals in a PointCloud](feature/estimating-surface-normals-in-a-pointcloud.md)
  * [How 3D Features work in PCL](feature/how-3d-features-work-in-pcl.md)
  * [Normal Estimation Using Integral Images](feature/normal-estimation-using-integral-images.md)
  * [Point Feature Histograms \(PFH\) descriptors](feature/point-feature-histograms-pfh-descriptors.md)
  * [Estimating VFH signatures for a set of points](feature/estimating-vfh-signatures-for-a-set-of-points.md)
  * [Moment of inertia and eccentricity based descriptors](feature/moment-of-inertia-and-eccentricity-based-descriptors.md)
  * [RoPs feature](feature/rops-feature.md)

---

* [Feature](Tutorial/Feature/README.md)
  * [\[Open3D\] Vertex normal estimation](Tutorial/Feature/open3d-vertex-normal-estimation.md)
  * [\[PCL-Python\] Surface Normal](Tutorial/Feature/pcl-python-surface-normal.md)

---

* [Filtering](filtering.md)
  * [Filtering a PointCloud using a PassThrough filter](filtering/filtering-a-pointcloud-using-a-passthrough-filter.md)
  * [Downsampling a PointCloud using a VoxelGrid filter](filtering/downsampling-a-pointcloud-using-a-voxelgrid-filter.md)
  * [Removing outliers using a StatisticalOutlierRemoval filter](filtering/removing-outliers-using-a-statisticaloutlierremoval-filter.md)
  * [Extracting indices from a PointCloud](filtering/extracting-indices-from-a-pointcloud.md)
  * [Removing outliers using a Conditional or Radius Outlier removal](filtering/removing-outliers-using-a-conditional-or-radius-outlier-removal.md)
* [Registration](Tutorial/Registration/README.md)
  * [How to use iterative closest point](Tutorial/Registration/iterative-closest-point.md)
  * [The PCL Registration API](Tutorial/Registration/the-pcl-registration-api.md)
  * [\[PCL-Cpp\] How to use Normal Distributions Transform](Tutorial/Registration/how-to-use-normal-distributions-transform.md)
  * [\[Open3D\] ICP registration](Tutorial/Registration/open3d-icp-registration.md)
  * [\[Open3D\] \(Fast\) Global registration](Tutorial/Registration/open3d-global-registration.md)
  * [\[Open3D\] Colored point cloud registration](Tutorial/Registration/open3d-colored-point-cloud-registration.md)
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

