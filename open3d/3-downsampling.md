## Voxelization

```python
print("Downsample the point cloud with a voxel of 0.05")
downpcd = voxel_down_sample(pcd, voxel_size = 0.05)
```



## Vertex normal estimation

```python
print("Recompute the normal of the downsampled point cloud")
estimate_normals(downpcd, search_param = KDTreeSearchParamHybrid(
        radius = 0.1, max_nn = 30))
draw_geometries([downpcd])
print("")
```

Normal manipulation is possible with "n", "+", "-" on the screen

        estimate_normals computes normal for every point. The function finds adjacent points and calculate the principal axis of the adjacent points using covariance analysis.
        
        The function takes an instance of KDTreeSearchParamHybrid class as an argument. The two key arguments radius = 0.1 and max_nn = 30 specifies search radius and maximum nearest neighbor. It has 10cm of search radius, and only considers up to 30 neighbors to save computation time.
        

