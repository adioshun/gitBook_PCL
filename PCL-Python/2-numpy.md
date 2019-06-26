```python
import numpy as np
import pcl
p = pcl.PointCloud(10)  # "empty" point cloud
a = np.asarray(p)       # NumPy view on the cloud
a[:] = 0                # fill with zeros
print(p[3])             # prints (0.0, 0.0, 0.0)
a[:, 0] = 1             # set x coordinates to 1
print(p[3])             # prints (1.0, 0.0, 0.0)

```


---

# Kitti Bin Read

```python 
# from Bin 
raw_lidar = np.fromfile('/datasets/testing/velodyne/000001.bin', dtype=np.float32).reshape((-1, 4))
#points = np.fromfile(v_path, dtype=np.float32, count=-1).reshape([-1, num_features]) #SECOND code 
# to Bin 
pc = pcl.load("./sample_lcas.pcd")
pa = pc.to_array()
pa.tofile('sample_lcas.bin')

# to Bin
pc = pcl.load("/workspace/_rosbag/office_bg_2018_10_22_pcd/1540261303.747807979.pcd")
pc_rgb=XYZ_to_XYZRGB(pc,[0,0,0] )
pa = pc_rgb.to_array()

pa.tofile('/workspace/_pcd/1540261303.747807979.bin')
```