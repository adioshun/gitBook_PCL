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