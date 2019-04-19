# [The PCL Registration API](http://pointclouds.org/documentation/tutorials/registration_api.php#registration-api)




## 개요 

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
    
    
    
    
## An overview of pairwise registration

