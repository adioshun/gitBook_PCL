## [Working with NumPy](http://www.open3d.org/docs/tutorial/Basic/working_with_numpy.html)


### From NumPy to Open3D


```
# Pass xyz to Open3D.PointCloud.points and visualize
pcd = PointCloud()
pcd.points = Vector3dVector(xyz)
write_point_cloud("../../TestData/sync.ply", pcd)

```


### From Open3D to NumPy

```python
    pcd_load = read_point_cloud("../../TestData/sync.ply")
    xyz_load = np.asarray(pcd_load.points)
    
```



    
