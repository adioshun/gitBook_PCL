# [The PCL Registration API](http://pointclouds.org/documentation/tutorials/registration_api.php#registration-api)




## 1. 개요 

- 정의 : The problem of consistently aligning various 3D point cloud data views into a complete model is known as **registration**
- 목적 :  find the **relative positions** and **orientations** of the separately acquired views in a **global coordinate** framework
- 분류 
    - Global Registration : 초기값 불필요 
    - Local Registration : 초기값 필요(Global Regstration사용), eg. ICP registration 
- 절차 : subsequent processing steps such as **segmentation** and object **reconstruction** can be applied.
- The algorithmic work in the PCL registration library is motivated by 
    - 대응점 찾기 `finding correct point correspondences in the given input datasets, `
    - 강체 변환(회전, 이동) 예측 `and estimating rigid transformations that can rotate and translate each individual dataset into a consistent global coordinate framework.`
- 필요 기능 `PCL contains a set of powerful algorithms that allow`
    - 대응점 평가 방법` the estimation of multiple sets of correspondences, `
    - 나쁜 대응점 제거 방 `as well as methods for rejecting bad correspondences, `
    - 변환 파악 `and estimating transformations in a robust manner from them. `
    
    
    
    
## 2. An overview of pairwise registration

- 목표 : transformation matrix (4x4) 계산
    - representing the rotation and translation

![](http://pointclouds.org/documentation/tutorials/_images/block_diagram_single_iteration.jpg)

절차 The computational steps for two datasets are straightforward:

- from a set of points, identify interest points (i.e., keypoints) that best represent the scene in both datasets;
- at each keypoint, compute a feature descriptor;
- from the set of feature descriptors together with their XYZ positions in the two datasets, 
    - estimate a set of correspondences, based on the similarities between features and positions;
- given that the data is assumed to be noisy, not all correspondences are valid, so reject those bad correspondences that contribute negatively to the registration process;
- from the remaining set of good correspondences, estimate a motion transformation.


## 3. Registration modules

### 3.1 Keypoints

keypoint is an interest point that has a “special property” in the scene
- 코너 

There are a number of different keypoints available in PCL like 
- NARF
- SIFT
- FAST

### 3.2 Feature descriptors

Based on the keypoints found we have to extract [features](http://www.pointclouds.org/documentation/tutorials/how_features_work.php)

Again there is a number of feature options to choose from, for example 
- NARF
- FPFH
- BRIEF
- SIFT

### 3.3 Correspondences estimation

Depending on the feature type we can use different methods to find the correspondences.

#### A. For point matching 

using the points’ xyz-coordinates as features
- brute force matching,
- kd-tree nearest neighbor search (FLANN),
- searching in the image space of organized data, and
- searching in the index space of organized data.

#### B. For feature matching 

not using the points’ coordinates, but certain features
- brute force matching and
- kd-tree nearest neighbor search (FLANN).


### 3.4 Correspondences rejection

Since wrong correspondences can negatively affect the estimation of the final transformation, they need to be rejected.

방법 
- RANSAC
- trimming down the amount and using only a certain percent of the found correspondences


### 3.5 Transformation estimation

compute the transformation.

- evaluate some error metric based on correspondence
- estimate a (rigid) transformation between camera poses (motion estimate) and minimize error metric
- optimize the structure of the points
    - Examples: - SVD for motion estimate; - Levenberg-Marquardt with different kernels for motion estimate;
- use the rigid transformation to rotate/translate the source onto the target, and potentially run an internal ICP loop with either all points or a subset of points or the keypoints
- iterate until some convergence criterion is met


- eg. Normal Distributions Transform (NDT) 

## 4. Example pipelines


### 4.1 Iterative Closest Point

- Search for correspondences.
- Reject bad correspondences.
- Estimate a transformation using the good correspondences.
- Iterate.

### 4.2 Feature based registration

- use SIFT Keypoints (pcl::SIFT…something)
- use FPFH descriptors (pcl::FPFHEstimation) at the keypoints 
    - see our tutorials for that, [like](http://www.pointclouds.org/media/rss2011.html)
- get the FPFH descriptors and estimate correspondences using `pcl::CorrespondenceEstimation`
- reject bad correspondences using one or many of the `pcl::CorrespondenceRejectionXXX methods`
- finally get a transformation as mentioned above




```cpp
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;
PointCloud<PointXYZ>::Ptr src, tgt;

// https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/sources/registration_api/example2.cpp

int
main (int argc, char** argv)
{
pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ> ("bun0.pcd", *src);
pcl::io::loadPCDFile<pcl::PointXYZ> ("bun4.pcd", *tgt);

// Get an uniform grid of keypoints  //다운 샘플링 
PointCloud<PointXYZ>::Ptr keypoints_src (new PointCloud<PointXYZ>), 
                         keypoints_tgt (new PointCloud<PointXYZ>);
UniformSampling<PointXYZ> uniform;
uniform.setRadiusSearch (1);  // 1m
uniform.setInputCloud (src);
uniform.filter (*keypoints_src);
uniform.setInputCloud (tgt);
uniform.filter (*keypoints_tgt);
print_info ("- Found %lu and %lu keypoints for the source and target datasets.\n", keypoints_src->points.size (), keypoints_tgt->points.size ());


// Compute normals for all points keypoint
PointCloud<Normal>::Ptr normals_src (new PointCloud<Normal>), 
                         normals_tgt (new PointCloud<Normal>);
NormalEstimation<PointXYZ, Normal> normal_est;
normal_est.setInputCloud (src);
normal_est.setRadiusSearch (0.5);  // 50cm
normal_est.compute (*normals_src);
normal_est.setInputCloud (tgt);
normal_est.compute (*normals_tgt);
print_info ("- Estimated %lu and %lu normals for the source and target datasets.\n", normals_src->points.size (), normals_tgt->points.size ());



// Compute FPFH features at each keypoint
PointCloud<FPFHSignature33>::Ptr fpfhs_src (new PointCloud<FPFHSignature33>), 
                              fpfhs_tgt (new PointCloud<FPFHSignature33>);
FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est;
fpfh_est.setInputCloud (keypoints_src);
fpfh_est.setInputNormals (normals_src);
fpfh_est.setRadiusSearch (1); // 1m
fpfh_est.setSearchSurface (src);
fpfh_est.compute (*fpfhs_src);

fpfh_est.setInputCloud (keypoints_tgt);
fpfh_est.setInputNormals (normals_tgt);
fpfh_est.setSearchSurface (tgt);
fpfh_est.compute (*fpfhs_tgt);



// Find correspondences between keypoints in FPFH space
CorrespondencesPtr all_correspondences (new Correspondences), 
                    good_correspondences (new Correspondences);

//findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);
CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> est;
est.setInputCloud (fpfhs_src);
est.setInputTarget (fpfhs_tgt);
est.determineReciprocalCorrespondences (*all_correspondences);

// Reject correspondences based on their XYZ distance
CorrespondenceRejectorDistance rej;
rej.setInputCloud<PointXYZ> (keypoints_src);
rej.setInputTarget<PointXYZ> (keypoints_tgt);
rej.setMaximumDistance (1);    // 1m
rej.setInputCorrespondences (all_correspondences);
rej.getCorrespondences (*good_correspondences);


for (int i = 0; i < good_correspondences->size (); ++i)
std::cerr << good_correspondences->at (i) << std::endl;

// Obtain the best transformation between the two sets of keypoints given the remaining correspondences
// Compute the best transformtion
Eigen::Matrix4f transform;
TransformationEstimationSVD<PointXYZ, PointXYZ> trans_est;
trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transform);


std::cerr << transform << std::endl;
// Transform the data and write it to disk
PointCloud<PointXYZ> output;
transformPointCloud (*src, output, transform);

savePCDFileBinary ("source_transformed.pcd", output);
}

```









