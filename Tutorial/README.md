이곳은 [PCL Tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#)의 내용을 간략하게 번역 정리 하기 위해 만들었습니다. 

> Radu Bogdan Rusu, Steve Cousins,"[3D is here: Point Cloud Library (PCL)](http://www.pointclouds.org/assets/pdf/pcl_icra2011.pdf)", ICRA2011

## PCL Modules 

- pcl_filters - 3D 점군 데이터에서 이상값과 노이즈 제거 등의 필터링
- pcl_features - 점군 데이터로부터 3D 특징 추정 (feature estimation) 을 위한 수많은 자료 구조와 방법들 
- pcl_keypoints - Keypoint (or interest point) 을 검출하는 알고리즘 구현 (BRISK, Harris Corner, NARF, SIFT, SUSAN 등)
- pcl_registration - 여러 데이터셋을 합쳐 큰 모델로 만드는 registration 작업 (ICP 등)
- pcl_kdtree - 빠른 최근거리 이웃을 탐색하는 FLANN 라이브러리를 사용한 kdtree 자료 구조
- pcl_octree - 점군 데이터로부터 계층 트리 구조를 구성하는 방법
- pcl_segmentation - 점군으로부터 클러스터들로 구분하는 알고리즘들
- pcl_sample_consensus - 선, 평면, 실린더 등의 모델 계수 추정을 위한 RANSAC 등의 알고리즘들
- pcl_surface - 3D 표면 복원 기법들 (meshing, convex hulls, Moving Least Squares 등)
- pcl_range_image - range image (or depth map) 을 나타내고 처리하는 방법
- pcl_io - OpenNI 호환 depth camera 로부터 점군 데이터를 읽고 쓰는 방법
- pcl_visualization - 3D 점군 데이터를 처리하는 알고리즘의 결과를 시각화

---

* [Keypoint](Keypoint/README.md)
  * [ISS Keypoint](Keypoint/iss-keypoint.md)
  * [NARF Keypoint](Keypoint/narf-keypoint.md)

* [Feature](Feature/README.md)
  * [How 3D Features work in PCL](Feature/how-3d-features-work-in-pcl.md)
  * [Estimating Surface Normals in a PointCloud](Feature/Estimating-surface-normals-in-a-pointcloud.md)
    * [Ref # Normal Estimation ](Feature/Normal-Estimation.md)
    * [Code # Noraml and Noral Histogram(Python)](Feature/python-pcl-normal-histogram.md)
    * [Code # Vertex normal estimation-Open3](Feature/open3d-vertex-normal-estimation.md)
  * [Normal Estimation Using Integral Images](Feature/Normal-Estimation-Using-Integral-Images.md)
  * Point Feature Histograms (PFH) descriptors
    * [PFH Descriptors-PCL-CPP](Feature/pcl-cpp-pfh-descriptors.md)
  * Fast Point Feature Histograms (FPFH) descriptors
  * [Estimating VFH signatures for a set of points-PCL-Python](Feature/pcl-python-estimating-vfh-signatures-for-a-set-of-points.md)
  * How to extract NARF Features from a range image
  * [Moment of inertia and eccentricity based descriptors](Feature/Moment-of-inertia-and-eccentricity-based-descriptors.md)
    * [Code # Moment of inertia and eccentricity based descriptors-PCL-Python](Feature/pcl-python-moment-of-inertia-and-eccentricity-based-descriptors.md)
  * RoPs (Rotational Projection Statistics) feature
    * [Code # RoPs feature-PCL-Python](Feature/pcl-python-rops-feature.md)
  * Globally Aligned Spatial Distribution (GASD) descriptors
  * [Ref # Summary of Descriptor](ref_1.md) 


* [Filtering](Filtering/README.md)
  * [Projecting points using a parametric model-PCL-Cpp](Filtering/pcl-cpp-projecting-points-using-a-parametric-model.md)
  * [Downsampling a PointCloud using a VoxelGrid filter-PCL-Cpp](Filtering/pcl-cpp-downsampling-a-pointcloud-using-a-voxelgrid-filter.md)
  * [Extracting indices from a PointCloud-PCL-Cpp](Filtering/pcl-cpp-extracting-indices-from-a-pointcloud.md)
  * [Extracting indices from a PointCloud-PCL-Python](Filtering/pcl-python-extracting-indices-from-a-pointcloud.md)
* [Registration](Registration/README.md)
  * [The PCL Registration API](Registration/the-pcl-registration-api.md)
  * [How to use iterative closest point](Registration/iterative-closest-point.md)
  * [How to incrementally register pairs of clouds](Registration/How-to-incrementally-register-pairs-of-clouds.md)
  * [Interactive Iterative Closest Point](Registration/Interactive-Iterative-Closest-Point.md)
  * [How to use Normal Distributions Transform](Registration/How-to-use-Normal-Distributions-Transform.md)
  * [Robust pose estimation of rigid objects](Registration/Robust-pose-estimation-of-rigid-objects.md)
  * [\[PCL-Cpp\] How to use Normal Distributions Transform](Registration/how-to-use-normal-distributions-transform.md)
  * [\[Open3D\] ICP registration](Registration/open3d-icp-registration.md)
  * [\[Open3D\] \(Fast\) Global registration](Registration/open3d-global-registration.md)
  * [\[Open3D\] Colored point cloud registration](Registration/open3d-colored-point-cloud-registration.md)
  * [\[PCL-Cpp\] Fuse two pointcloud ](Registration/pcl-cpp-fuse-two-pointcloud.md)
* [KdTree](KdTree/README.md)
  * [brute_force](KdTree/brute_force.md)
  * [organized](KdTree/organized.md)
  * [How to use a KdTree to search](KdTree/how-to-use-a-kdtree-to-search-PCL-Cpp.md)
  * [performance_compare_brute_force_kd_tree](KdTree/performance_compare_brute_force_kd_tree.md)
* [Octree](Octree/README.md)
  * [Spatial Partitioning and Search Operations with Octrees-PCL-Cpp](Octree/spatial-partitioning-and-search-operations-with-octrees-PCL-Cpp.md)
  * [Spatial Partitioning and Search Operations with Octrees-PCL-Python](Octree/spatial-partitioning-and-search-operations-with-octrees-PCL-Python.md)
  * [Spatial change detection on unorganized point cloud data-PCL-Cpp](Octree/spatial-change-detection-on-unorganized-point-cloud-data-PCL-Cpp.md)
  * [Spatial change detection on unorganized point cloud data-PCL-Python](Octree/spatial-change-detection-on-unorganized-point-cloud-data-PCL-Python.md)
  * [Point Cloud Compression-PCL-Cpp](Octree/point-cloud-compression-PCL-Cpp.md)
* [Sample Consensus](SampleConsensus/README.md)
  * [How to use Random Sample Consensus model \(100%\)](SampleConsensus/how-to-use-random-sample-consensus-model.md)
* [Segmentation](Segmentation/README.md)
  * [\[PCL-Cpp\] Identifying ground returns using Progressive Morphological Filter segmentation](Segmentation/pcl-cpp-identifying-ground-returns-using-progressive-morphological-filter-segmentation.md)
  * [\[Matlab\] segmentGroundFromLidarData](Segmentation/matlab-segmentgroundfromlidardata.md)

  * [Region growing segmentation](Segmentation/region-growing-segmentation.md)
  * [\[PCL-Python\] Progressive Morphological Filter segmentation](Segmentation/pcl-python-progressive-morphological-filter-segmentation.md)
  * [\[PCL-Cpp\] Plane model segmentation](Segmentation/pcl-cpp-plane-model-segmentation.md)

  * [Euclidean Cluster Extraction](Segmentation/euclidean-cluster-extraction.md)
    * [Conditional Euclidean Clustering](Segmentation/conditional-euclidean-clustering.md)
* [Surface](Surface/README.md)
  * [Smoothing and normal estimation based on polynomial reconstruction](Surface/Smoothing-and-normal-estimation-based-on-polynomial-reconstruction.md)
  * [Fast triangulation of unordered point clouds](Surface/Fast-triangulation-of-unordered-point-clouds.md)
* [Visualization](visualization.md)
* [Tracking object in real time](tracking-object-in-real-time.md)
* [Range Image](RangeImage/How-to-create-a-range-image-from-a-point-cloud.md)
* [Recognition](Recognition/README.md)
  * [3D Object Recognition based on Correspondence Grouping](Recognition/3D-Object-Recognition-based-on-Correspondence-Grouping.md)

