# Spatial Partitioning and Search Operations with Octrees

    Cpp : http://pointclouds.org/documentation/tutorials/octree.php#octree-search    
    python : https://github.com/strawlab/python-pcl/blob/master/examples/official/octree/octree_search.py

An octree is a tree-based data structure for managing sparse 3-D data. 

Each internal node has exactly eight children. 

In this tutorial we will learn how to use the octree for spatial partitioning and neighbor search within pointcloud data. 

Particularly, we explain how to perform a “Neighbors within Voxel Search”, the “K Nearest Neighbor Search” and “Neighbors within Radius Search”.





```python
resolution = 0.5#128.0f #  length of one side of a voxel, 단위 = 해당 pointcloud의 단위 따름 
octree = cloud.make_octreeSearch(resolution)
octree.add_points_from_input_cloud()
```

Then we create an octree instance which is initialized with its resolution. 
This octree keeps a vector of point indices within its leaf nodes. 
The **resolution parameter** describes the length of the smallest voxels at lowest octree level. 
The depth of the octree is therefore a function of the resolution as well as the spatial dimension of the pointcloud. 
If a bounding box of the pointcloud is know, it should be assigned to the octree by using the defineBoundingBox method. 
Then we assign a pointer to the PointCloud and add all points to the octree.

> The resolution parameter describes the length of the smallest voxels at lowest octree level



```python 

def background_removal2(input_pcl_xyzrgb, searchPoint):
    resolution = 0.2 #128.f 
    radius = 0.5 #256.0f * rand () / (RAND_MAX + 1.0f);
    
    #배경 포인트 
    input_pcl_xyzrgb = pcl_helper.XYZRGB_to_XYZ(input_pcl_xyzrgb)   

    octree = input_pcl_xyzrgb.make_octreeSearch(resolution)
    octree.add_points_from_input_cloud()
    
    backroud = input_pcl_xyzrgb
    
    # backroud = DayTime(=cloud) - NightTime(=searchPoint)
    # vox위치값을 기반으로 radius거리의 cloud제거
    for i in range(0,searchPoint.size-1):
        searchPoints = (searchPoint[i][0], searchPoint[i][1], searchPoint[i][2])
        [ind, sqdist] = octree.radius_search (searchPoints, radius)
        lind = ind.tolist()
        backroud = backroud.extract(lind,negative=True)
    
    pc = pcl_helper.XYZ_to_XYZRGB(backroud,[255,255,255])

    return pc
    
    
# 호출 
def callback(input_ros_msg):    
    pcl_xyzrgb = pcl_helper.ros_to_pcl(input_ros_msg) #ROS 메시지를 PCL로 변경    
    background_pcl_xyzrgb = background_removal(pcl_xyzrgb, searchPoint) # 탐지 영역(RoI) 설정 
    roi_ros_msg = pcl_helper.pcl_to_ros(background_pcl_xyzrgb) #PCL을 ROS 메시지로 변경 
    pub = rospy.Publisher("/velodyne_bg", PointCloud2, queue_size=1)
    pub.publish(roi_ros_msg)

if __name__ == "__main__":
    searchPoint = pcl.PointCloud()
    searchPoint.from_file('./background_extraction/nighttime2.pcd')    
    rospy.init_node('myopen3d_node', anonymous=True)
    rospy.Subscriber('/lidar_201/velodyne_points', PointCloud2, callback)    
    rospy.spin()
```