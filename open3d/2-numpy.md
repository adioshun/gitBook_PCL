## [Working with NumPy](http://www.open3d.org/docs/tutorial/Basic/working_with_numpy.html)


### From NumPy to Open3D


```python
# generate some neat n times 3 matrix using a variant of sync function
x = np.linspace(-3, 3, 401)
mesh_x, mesh_y = np.meshgrid(x,x)
z = np.sinc((np.power(mesh_x,2)+np.power(mesh_y,2)))
xyz = np.zeros((np.size(mesh_x),3))
xyz[:,0] = np.reshape(mesh_x,-1)
xyz[:,1] = np.reshape(mesh_y,-1)
xyz[:,2] = np.reshape(z,-1)
print('xyz')
print(xyz)

# Pass xyz to Open3D.PointCloud and visualize
pcd = PointCloud()
pcd.points = Vector3dVector(xyz)
write_point_cloud("../../TestData/sync.ply", pcd)

```


### From Open3D to NumPy

```python
pcd_load = read_point_cloud("../../TestData/sync.ply")
xyz_load = np.asarray(pcd_load.points)
```



    
