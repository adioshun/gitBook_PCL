# [[Open3D] Global registration](http://www.open3d.org/docs/tutorial/Advanced/global_registration.html#global-registration)

- 분류 
    - Global Registration : 초기값 불필요 
    - Local Registration : 초기값 필요(Global Regstration사용), eg. ICP registration 

- Open3D에서는 좀더 빠른 수행을 위해 Fast Global registration을 제공 한다. 

## 1. Input

```python 
def prepare_dataset(voxel_size):
    print(":: Load two point clouds and disturb initial pose.")
    source = read_point_cloud("../../TestData/ICP/cloud_bin_0.pcd")
    target = read_point_cloud("../../TestData/ICP/cloud_bin_1.pcd")
    
    
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0],
                            [1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0]])
    
    source.transform(trans_init) 
    
    draw_registration_result(source, target, np.identity(4))
```

- They are misaligned with an identity matrix as transformation


## 2. Extract geometric feature

```python 

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    
    #다운 샘플
    pcd_down = voxel_down_sample(pcd, voxel_size)

    #노멀 계산 estimate normals
    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    estimate_normals(pcd_down, KDTreeSearchParamHybrid(
            radius = radius_normal, max_nn = 30))

    #FPFH feature 계산 
    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = compute_fpfh_feature(pcd_down,
            KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))
```

- The FPFH feature is a 33-dimensional vector that describes the local geometric property of a point.

## 3. RANSAC

```python 
def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    
    result = registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            distance_threshold,
            TransformationEstimationPointToPoint(False), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(0.9), #제거 알고리즘 
            CorrespondenceCheckerBasedOnDistance(distance_threshold)], #제거 알고리즘 
            RANSACConvergenceCriteria(4000000, 500)) #최대 RASAC 횟수 & 최대 확인 횟
```

`registration_ransac_based_on_feature_matching()`
- 파라미터 
    - RANSACConvergenceCriteria : the maximum number of RANSAC iterations & the maximum number of validation steps.

- 절차 : In each RANSAC iteration, `ransac_n` 
    - random points are picked from the source point cloud. 
    - Their corresponding points in the target point cloud are detected by querying the nearest neighbor in the 33-dimensional FPFH feature space. 
    - A pruning step takes fast **pruning algorithms** to quickly reject false matches early.
    - Only matches that pass the pruning step are used to compute a transformation, which is validated on the entire point cloud. 

```
- 제거 알고리즘`pruning algorithms`:
    - `CorrespondenceCheckerBasedOnDistance` checks if aligned point clouds are close (less than specified threshold).
    - `CorrespondenceCheckerBasedOnEdgeLength` checks if the lengths of any two arbitrary edges (line formed by two vertices) individually drawn from source and target correspondences are similar.
    - `CorrespondenceCheckerBasedOnNormal` considers vertex normal affinity of any correspondences. 
        - It computes dot product of two normal vectors. It takes radian value for the threshold.
```


## 4. Local refinement

성능 문제로 Global Registration은 다운샘플링된 상태로 진행 되고, 결과 또한 정확히 맞지는 않는다. P2Plane ICP를 통해 Refine 절차를 수행 한다. `For performance reason, the global registration is only performed on a heavily down-sampled point cloud. The result is also not tight. We use Point-to-plane ICP to further refine the alignment.`

> ICP(=Local) registration으로 바로 넘어 가도 될듯 

```python 
def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = registration_icp(source, target, distance_threshold,
            result_ransac.transformation,
            TransformationEstimationPointToPlane())
```





