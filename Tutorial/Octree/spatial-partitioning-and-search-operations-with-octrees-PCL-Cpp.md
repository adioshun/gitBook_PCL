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

---
# [PCL Series 3 - Use of Octree- Search ](https://blog.csdn.net/qq_22170875/article/details/84844385)

```cpp

#include <pcl/io/pcd_io.h>  //文件输入输出
#include <pcl/octree/octree_search.h>  //octree相关定义
#include <pcl/visualization/cloud_viewer.h>  //vtk可视化相关定义
#include <pcl/point_types.h>  //点类型相关定义

#include <iostream>
#include <vector>

int main()
{
	//1.读取点云
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("rabbit.pcd", *cloud) == -1)
	{
		PCL_ERROR("Cloudn't read file!");
		return -1;
	}

	//2.原始点云着色
	for (size_t i = 0; i < cloud->points.size(); ++i){
		cloud->points[i].r = 255;
		cloud->points[i].g = 255;
		cloud->points[i].b = 255;
	}

	//3.创建Octree实例对象
	float resolution = 0.03f;  //设置octree体素分辨率
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(resolution); //建立octree对象
	octree.setInputCloud(cloud); //传入需要建立kdtree的点云指针
	octree.addPointsFromInputCloud();  //构建Octree

	//3.1.体素内近邻搜索
	pcl::PointXYZRGB searchPoint1 = cloud->points[1250]; //设置查找点
	std::vector<int> pointIdxVec;  //保存体素近邻搜索的结果向量
	if (octree.voxelSearch(searchPoint1, pointIdxVec))
	{
		std::cout << "Neighbors within voxel search at (" << searchPoint1.x
			<< " " << searchPoint1.y
			<< " " << searchPoint1.z << ")"
			<< std::endl;

		//给查找到的近邻点设置颜色
		for (size_t i = 0; i < pointIdxVec.size(); ++i){
			cloud->points[pointIdxVec[i]].r = 255;
			cloud->points[pointIdxVec[i]].g = 0;
			cloud->points[pointIdxVec[i]].b = 0;
		}		
	}

	//3.2.K近邻搜索
	pcl::PointXYZRGB searchPoint2 = cloud->points[3000]; //设置查找点
	int K = 200;
	std::vector<int> pointIdxNKNSearch; //保存K近邻点的索引结果
	std::vector<float> pointNKNSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方

	std::cout << "K nearest neighbor search at (" << searchPoint2.x
		<< " " << searchPoint2.y
		<< " " << searchPoint2.z
		<< ") with K=" << K << std::endl;

	if (octree.nearestKSearch(searchPoint2, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
	{   //给查找到的近邻点设置颜色
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
			cloud->points[pointIdxNKNSearch[i]].r = 0;
			cloud->points[pointIdxNKNSearch[i]].g = 255;
			cloud->points[pointIdxNKNSearch[i]].b = 0;
		}	
	}
	std::cout << "K = 200近邻点个数：" << pointIdxNKNSearch.size() << endl;

	//3.3.半径内近邻搜索
	pcl::PointXYZRGB searchPoint3 = cloud->points[6500]; //设置查找点
	std::vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
	std::vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
	float radius = 0.02; //设置查找半径范围

	std::cout << "Neighbors within radius search at (" << searchPoint3.x
		<< " " << searchPoint3.y
		<< " " << searchPoint3.z
		<< ") with radius=" << radius << std::endl;

	if (octree.radiusSearch(searchPoint3, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{    //给查找到的近邻点设置颜色
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
			cloud->points[pointIdxRadiusSearch[i]].r = 0;
			cloud->points[pointIdxRadiusSearch[i]].g = 0;
			cloud->points[pointIdxRadiusSearch[i]].b = 255;
		}		
	}
	std::cout << "半径0.02近邻点个数： " << pointIdxRadiusSearch.size() << endl;

	//4.显示点云
	pcl::visualization::CloudViewer viewer("cloud viewer");
	viewer.showCloud(cloud);

	system("pause");
	return 0;
}

```

![](https://img-blog.csdnimg.cn/20181205215523236.JPG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIyMTcwODc1,size_16,color_FFFFFF,t_70)