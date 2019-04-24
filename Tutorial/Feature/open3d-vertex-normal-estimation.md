# [[Open3D] Vertex normal estimation](http://www.open3d.org/docs/tutorial/Basic/pointcloud.html#vertex-normal-estimation)


## 1. 계산 

```python 
print("Recompute the normal of the downsampled point cloud")

#finds adjacent points and calculate the principal axis of the adjacent points using covariance analysis.
estimate_normals(downpcd, 
    search_param = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30)) 
    # search radius and maximum nearest neighbor
    
draw_geometries([downpcd])
 ```
 
 
 ## 2. 접근 
 
 ```python
print("Print a normal vector of the 0th point")
print(downpcd.normals[0])

#>Print a normal vector of 0th point
#>[-0.27566603 -0.89197839 -0.35830543]

# np변환 

print("Print the normal vectors of the first 10 points")
np.asarray(downpcd.normals)[:10,:]

```