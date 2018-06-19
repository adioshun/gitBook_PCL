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
        
        Recompute the normal of the downsampled point cloud

        estimate_normalsCalculates the normals of all points. This function finds neighboring points using covariance analysis and computes the principal axes of neighboring points.
        
        This function KDTreeSearchParamHybridtakes an instance of a class as an argument. radius = 0.1And max_nn = 30two important argument that is used to specify a search radius and up to the nearest neighbor. This has a search radius of 10 cm, considering only up to 30 neighborhoods in order to save calculation time.
        
        The covariance analysis algorithm generates two mutually opposite vectors as normal candidates. If you do not know the global structure of the geometry, neither is correct. This is a problem known as the normal direction problem. Open3D attempts to orient the normals so that they are aligned with the original normals when normals are present. Otherwise, Open3D makes random guesses. If the direction is important, you need to call an orientation function such as  orient_normals_to_align_with_directionand orient_normals_towards_camera_location.
        
        draw_geometriesTo visualize the point cloud nand press to display the normal of the point. -You +can control the length of the normal by using keys and keys.

