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














