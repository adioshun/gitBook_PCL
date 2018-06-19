# File I/O

## 1. Read / Write

```python 
from open3d import *

print("Testing IO for point cloud ...")
pcd = read_point_cloud("../../TestData/fragment.pcd")

print(pcd)

write_point_cloud("copy_of_fragment.pcd", pcd)

```

The supported extension names are: pcd, ply, xyz, xyzrgb, xyzn, pts.


## 2. Iterative Closest Point

What is ICP: a method of superimposing two point clouds

추후 참고 : http://robonchu.hatenablog.com/entry/2018/02/25/121024

    In this tutorial we show the ICP (iteration nearest neighbor) registration algorithm. Over the years, it has become a pillar of geometric alignment process in both research and business. Input is an initial transformation to roughly align two point clouds, the point cloud of the source to the target point cloud. The output is a sophisticated transformation that closely aligns the two point clouds of input. The helper function draw_registration_resultvisualizes the alignment during the alignment process. In this tutorial we show two ICP variants: point-to-point ICP and point-to-plane ICP (Rusinkiewicz & Levoy 2001).
    
    Rusinkiewicz, S. & M. Levoy. (2001). Efficient variants of the ICP algorithm. In 3-D Digital Imaging and Modeling.
    
    Note: For ICP Https://En.Wikipedia.Org/wiki/Iterative_closest_point reference
    
    Iterative nearest point (ICP) is an algorithm used to minimize the difference between two point clouds. (If the wheel odometry for particularly slippery terrain unreliable) ICP is or reconfigure the 2D or 3D surfaces from various scan, or position estimate in the robot, optimal path planning a stand or, It is often used for joint alignment of bone models.
    
    
    Point-to-point ICP 
    Point-to-plane ICP 



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