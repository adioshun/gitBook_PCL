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



## 1. From pc to arr

pa = pc.to_array()

## 2. From arr to pc

### XYZ
pc = pcl.PointCloud(array)


### XYZRGB

```
p = pcl.PointCloud_PointXYZRGBA()
p.from_array(np.array(concate, dtype=np.float32))
```


```
pcl_xyzrgb = pcl_helper.XYZ_to_XYZRGB(pcl_xyz, [255,255,255])       
        
out_ros_msg = pcl_helper.pcl_to_ros(pcl_xyzrgb) #PCL을 ROS 메시지로 변경 
pub_out = rospy.Publisher("/velodyne_out", PointCloud2, queue_size=1)
pub_out.publish(out_ros_msg)
```

---

함수명	기능	파일포멧
np.save()	NumPy 배열 객체 1개를 파일에 저장	바이너리
np.savez()	NumPy 배열 객체 복수개를 파일에 저장	바이너리
np.load()	NumPy 배열 저장 파일로 부터 객체 로딩	바이너리
np.loadtxt()	텍스트 파일로 부터 배열 로딩	텍스트
np.savetxt()	텍스트 파일에 NumPy 배열 객체 저장	텍스트



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