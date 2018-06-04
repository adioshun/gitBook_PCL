```python
from __future__ import print_function
import pcl
import numpy as np


#p = pcl.PointCloud()
#p.from_file("test.pcd") # Deprecated; use pcl.load instead.

pc = pcl.load("sample_sec.pcd") #cloud = pcl.load_XYZRGB('sample_table.pcd')
pa = pc.to_array()

print(type(pc))
print(type(pa))

print(pa.shape)
print(pa.ndim)
print(pa.dtype)

print(pa)

pc.width


cloud = pcl.load('sample_sec.pcd')
print ('Loaded ' + str(cloud.width * cloud.height) + ' data points from test_pcd.pcd with the following fields: ')
for i in range(0, cloud.size):
print ('x: ' + str(cloud[i][0]) + ', y : ' + str(cloud[i][1]) + ', z : ' + str(cloud[i][2]))



pcl.save(cloud, 'test.pcd') pcl.save_XYZRGBA(pc, 'test.pcd')

```

