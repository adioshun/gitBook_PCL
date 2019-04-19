# [The PCL Registration API](http://pointclouds.org/documentation/tutorials/registration_api.php#registration-api)




## 1. 개요 

- 정의 : The problem of consistently aligning various 3D point cloud data views into a complete model is known as **registration*
- 목적 :  find the **relative positions** and **orientations** of the separately acquired views in a **global coordinate** framework
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




