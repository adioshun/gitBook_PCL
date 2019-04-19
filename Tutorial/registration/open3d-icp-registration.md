# [[Open3D] ICP registration](http://www.open3d.org/docs/tutorial/Basic/icp_registration.html)


입력 `The input are `
    - two point clouds 
    - an initial transformation : usually obtained by a [global registration algorithm](http://www.open3d.org/docs/tutorial/Advanced/global_registration.html#global-registration)
출력 `The output is `
    - a refined transformation 

        
In this tutorial, we show two ICP variants, 
- the point-to-point ICP 
- the point-to-plane ICP : Normal 정보 사용, 더 빠르

In general, the ICP algorithm iterates over two steps:
- Find correspondence set K={(p,q)} from target point cloud P, and source point cloud Q transformed with current transformation matrix T.
- Update the transformation T by minimizing an objective function E(T) defined over the correspondence set K.

여러 변형들은 E(T)가 다르다. `Different variants of ICP use different objective functions  E(T)`

- Point-to-point ICP에서는 아래 object를 사용 였다. 

![](https://i.imgur.com/QGKX26b.png)

```
Paul J. Besl and Neil D. McKay, A Method for Registration of 3D Shapes, PAMI, 1992.
```

- Point-to-plane ICP에서는 아래 object를 사용 하였다. 

![](https://i.imgur.com/LwRWt4P.png)
- $$n_p$$ is the normal of point $$p$$.
```
Rusinkiewicz and M. Levoy. Efficient variants of the ICP algorithm. In 3-D Digital Imaging and Modeling, 2001.
```

> P2Plane방식이 p2point방식 보다 빠른 convergence speed를 보였다. 




  
```
Rusinkiewicz, S. & M. Levoy. (2001). Efficient variants of the ICP algorithm. In 3-D Digital Imaging and Modeling.
```
Iterative nearest point (ICP) is an algorithm used to minimize the difference between two point clouds.

(If the wheel odometry for particularly slippery terrain unreliable) ICP is or reconfigure the 2D or 3D surfaces from various scan, or position estimate in the robot, optimal path planning a stand or, It is often used for joint alignment of bone models.

ICP is often used to reconstruct 2D or 3D surfaces from different scans, to localize robots and achieve optimal path planning


## 0. 시각화 함수 

```python
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])
```


## 1. Input 

```python 

# 파일 읽기
source = read_point_cloud("../../TestData/ICP/cloud_bin_0.pcd")
target = read_point_cloud("../../TestData/ICP/cloud_bin_1.pcd")

# 초기값 설정 
threshold = 0.02
trans_init = np.asarray( #usually obtained by a [global registration algorithm]
            [[0.862, 0.011, -0.507,  0.5],
            [-0.139, 0.967, -0.215,  0.7],
            [0.487, 0.255,  0.835, -1.4],
            [0.0, 0.0, 0.0, 1.0]])

#시각화 
draw_registration_result(source, target, trans_init)

# Initial alignment
evaluation = evaluate_registration(source, target, threshold, trans_init)


#결과 출력 
print(evaluation)
"""
Initial alignment
RegistrationResult with fitness = 0.174723, inlier_rmse = 0.011771,
and correspondence_set size of 34741
Access transformation to get result.
"""
```

`evaluate_registration` calculates two main metrics. 
- `fitness` : measures the overlapping area (# of inlier correspondences / # of points in target). Higher the better. 
- `inlier_rmse` : measures the RMSE of all inlier correspondences. Lower the better.


## 2. Point-to-point ICP / Point-to-plane ICP

여러 변형들은 E(T)가 다르다.
 


```python 
#Apply point-to-point ICP
reg_p2p = registration_icp(source, target, threshold, trans_init,
        TransformationEstimationPointToPoint(),
        ICPConvergenceCriteria(max_iteration = 2000))

# Apply point-to-plane ICP 
reg_p2l = registration_icp(source, target, threshold, trans_init,
        TransformationEstimationPointToPlane())

print(reg_p2p)
"""
RegistrationResult with fitness = 0.372450, inlier_rmse = 0.007760,
and correspondence_set size of 74056
Access transformation to get result.
"""

# Transformation 출력 
print(reg_p2p.transformation)
"""
[[ 0.83924644  0.01006041 -0.54390867  0.64639961]
 [-0.15102344  0.96521988 -0.21491604  0.75166079]
 [ 0.52191123  0.2616952   0.81146378 -1.50303533]
 [ 0.          0.          0.          1.        ]]
"""

#시각화 
draw_registration_result(source, target, reg_p2p.transformation)
```


Function `registration_icp` takes a parameter and runs point-to-point ICP to obtain results.
- parameter 
    - TransformationEstimationPointToPoint() : compute the residuals and Jacobian matrices of the point-to-point ICP objective

    - ICPConvergenceCriteria : 실행횟수, By default, runs until convergence or reaches a maximum number of iterations (30 by default)







---

---

```python
#Data read
source = read_point_cloud ("bun.pcd")
target = read_point_cloud ("bun 045.pcd")

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_kp = voxel_down_sample(pcd, voxel_size)
    radius_normal = voxel_size * 2
    viewpoint = np.array([0.,0.,100.], dtype='float64')
    estimate_normals(pcd_kp, KDTreeSearchParamHybrid(radius = radius_normal, max_nn = 30))
    orient_normals_towards_camera_location( pcd_kp, camera_location = viewpoint )
    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = compute_fpfh_feature(pcd_kp,
    KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))
    return pcd_kp, pcd_fpfh

def execute_global_registration(
    source_kp, target_kp, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    result = registration_ransac_based_on_feature_matching(
    source_kp, target_kp, source_fpfh, target_fpfh,
    distance_threshold,
    TransformationEstimationPointToPoint(False), 4,
    [CorrespondenceCheckerBasedOnEdgeLength(0.9),
    CorrespondenceCheckerBasedOnDistance(distance_threshold)],
    RANSACConvergenceCriteria(40000, 500))
    return result

def refine_registration(source, target, trans, voxel_size):
    distance_threshold = voxel_size * 0.4
    result = registration_icp(source, target,
    distance_threshold,
    trans,
    TransformationEstimationPointToPlane())
    return result

# Key point detection and feature extraction
voxel_size = 0.01
source_kp, source_fpfh = preprocess_point_cloud (source, voxel_size)
target_kp, target_fpfh = preprocess_point_cloud (target, voxel_size)

Posture estimation by #RANSAC
result_ransac = execute_global_registration (source_kp, target_kp,
source_fpfh, target_fpfh, voxel_size)
# Fine modification by # ICP
result_icp = refine_registration (source, target,
result_ransac.transformation, voxel_size)
# Displaying results
draw_registration_result (source, target, result_icp.transformation)
```

> [SSII2018TS: 3D object detection and its application to robot vision](https://www.slideshare.net/SSII_Slides/3d-101077557) 58slide