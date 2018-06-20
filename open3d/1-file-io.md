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


