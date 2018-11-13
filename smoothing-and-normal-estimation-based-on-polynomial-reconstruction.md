# [Smoothing and normal estimation based on polynomial reconstruction](http://pointclouds.org/documentation/tutorials/resampling.php#moving-least-squares)


> Upsampleing의 방법으로 활용 가능 
> - https://github.com/strawlab/python-pcl/blob/master/examples/official/Surface/resampling.py

This tutorial explains how a Moving Least Squares (MLS) surface reconstruction method can be used to smooth and resample noisy data. 






---

## Code (python)

```python

import numpy as np
import pcl
import random

cloud = pcl.load('bun0.pcd')
print('cloud(size) = ' + str(cloud.size))

# Create a KD-Tree
tree = cloud.make_kdtree()

# Output has the PointNormal type in order to store the normals calculated by MLS
mls = cloud.make_moving_least_squares()
# // Set parameters
mls.set_Compute_Normals (True)
mls.set_polynomial_fit (True)
mls.set_Search_Method (tree)
mls.set_search_radius (0.03) # Use all neighbors in a radius of 3cm.
print('set parameters')

# // Reconstruct
mls_points = mls.process ()
print('cloud(size) = ' + str(mls_points.size))

pcl.save_PointNormal(mls_points, 'bun0-mls.pcd')
```

> 노이즈 제거후 Upsampling을 수행 하므로, 제거된 노이즈가 많을경우 포인트 수는 오히려 감소 할수 있음 