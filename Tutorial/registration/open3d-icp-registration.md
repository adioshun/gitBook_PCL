# [[Open3D] ICP registration](http://www.open3d.org/docs/tutorial/Basic/icp_registration.html)


- 입력 `The input are `
    - two point clouds 
    - an initial transformation : usually obtained by a [global registration algorithm](http://www.open3d.org/docs/tutorial/Advanced/global_registration.html#global-registration)
- 출력 `The output is `
    - a refined transformation 

        
- In this tutorial, we show two ICP variants, 
    - the point-to-point ICP 
    - the point-to-plane ICP
    
```
Rusinkiewicz, S. & M. Levoy. (2001). Efficient variants of the ICP algorithm. In 3-D Digital Imaging and Modeling.
```
Iterative nearest point (ICP) is an algorithm used to minimize the difference between two point clouds.

(If the wheel odometry for particularly slippery terrain unreliable) ICP is or reconfigure the 2D or 3D surfaces from various scan, or position estimate in the robot, optimal path planning a stand or, It is often used for joint alignment of bone models.

ICP is often used to reconstruct 2D or 3D surfaces from different scans, to localize robots and achieve optimal path planning





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