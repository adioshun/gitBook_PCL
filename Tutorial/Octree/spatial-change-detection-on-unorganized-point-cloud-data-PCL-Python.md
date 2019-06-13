# [Spatial change detection on unorganized point cloud data](https://github.com/strawlab/python-pcl/blob/master/examples/official/octree/octree_change_detection.py)




```python
# 정의 
def background_removal(daytime, nighttime):

    resolution = 0.8
    #배경 포인트 
    octree = nighttime.make_octreeChangeDetector(resolution)
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
    print("Shape of Result : {}".format(result.shape))

    pc = pcl.PointCloud(result)
    

    cloud = filter.do_statistical_outlier_filtering(pc,10,0.01) #(pc,10,0.001)


    cloud = pcl_helper.XYZ_to_XYZRGB(cloud,[255,255,255])
    #print("pc type :{}".format(type(pc)))
    return cloud

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