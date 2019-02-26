# 배경 제거 

## 1. 베이스라인 생성 



## 2. 배경 제거 `(bg_helper.py)`

```python 

#!/usr/bin/env python3
# coding: utf-8

import sys
sys.path.append("/workspace/include")


import pcl_helper
import filter_helper


import pickle
import numpy as np
import pcl
import pcl_msg
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

baselinefile = "baseline.pkl"
leaf_size = 0.1

print("serchPointNP Loaded : {}".format(baselinefile))
pkl_file = open(baselinefile, 'rb')   
arr = pickle.load(pkl_file)

searchPoint = pcl.PointCloud()
searchPoint.from_array(arr)   

print("Normal Baseline point {}".format(searchPoint))
baseline = filter_helper.do_voxel_grid_downssampling(searchPoint,leaf_size)
print("Voxeled Basedline point {}".format(searchPoint))


def background_removal(daytime):

    resolution = 0.5#0.8  # 값이 커지면 missing 존재, noise도 존재 
    #배경 포인트 
    octree = baseline.make_octreeChangeDetector(resolution)
    octree.add_points_from_input_cloud ()
    octree.switchBuffers () #Switch buffers and reset current octree structure.

    # 입력 포인트 #cloudB cloudA
       
    daytime = pcl_helper.XYZRGB_to_XYZ(daytime)

    octree.set_input_cloud(daytime)
    octree.add_points_from_input_cloud ()
    newPointIdxVector = octree.get_PointIndicesFromNewVoxels ()
    daytime.extract(newPointIdxVector)


    #배경 제거 포인트 
    result = np.zeros((len(newPointIdxVector)+1, 3), dtype=np.float32)
    for i in range(0, len(newPointIdxVector)):
        result[i][0] = daytime[newPointIdxVector[i]][0]
        result[i][1] = daytime[newPointIdxVector[i]][1]
        result[i][2] = daytime[newPointIdxVector[i]][2]
    #print("Shape of Result : {}".format(result.shape))

    pc = pcl.PointCloud(result)
    

    #pc = filter_helper.do_statistical_outlier_filtering(pc,10,0.001) #(pc,10,0.001)


    cloud = pcl_helper.XYZ_to_XYZRGB(pc,[255,255,255])
    
    return cloud
"""
def callback(input_ros_msg):
    
    pcl_xyzrgb = pcl_helper.ros_to_pcl(input_ros_msg) #ROS 메시지를 PCL로 변경
    pcl_xyz = pcl_helper.XYZRGB_to_XYZ(pcl_xyzrgb)

    pcl_xyzrgb = background_removal(pcl_xyz, searchPoint)
    bg_ros_msg = pcl_helper.pcl_to_ros(pcl_xyzrgb) #PCL을 ROS 메시지로 변경 
    pub_bg = rospy.Publisher("/velodyne_bg", PointCloud2, queue_size=1)
    pub_bg.publish(bg_ros_msg)
"""
```
