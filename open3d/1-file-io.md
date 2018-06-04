```python 
from open3d import *

print("Testing IO for point cloud ...")
pcd = read_point_cloud("../../TestData/fragment.pcd")

print(pcd)

write_point_cloud("copy_of_fragment.pcd", pcd)

```