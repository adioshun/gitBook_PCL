# [How to use a KdTree to search](http://pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search)

> python : https://github.com/strawlab/python-pcl/blob/master/examples/kdtree.py
> https://github.com/strawlab/python-pcl/blob/master/examples/official/kdtree/kdtree_search.py

- KdTree를 이용하여 특정 포인트나 위치에서 k-NN 을 찾는 법을 살펴 본다. `In this tutorial we will go over how to use a KdTree for finding the K nearest neighbors of a specific point or location, `

- and then we will also go over how to find all neighbors within some radius specified by the user (in this case random).

## Theoretical primer

A k-d tree, or k-dimensional tree, is a data structure used in computer science for organizing some number of points in a space with k dimensions. 

It is a binary search tree with other constraints imposed on it. 

K-d trees are very useful for range and nearest neighbor searches. 

For our purposes we will generally only be dealing with point clouds in three dimensions, so all of our k-d trees will be three-dimensional. 

Each level of a k-d tree splits all children along a specific dimension, using a hyperplane that is perpendicular to the corresponding axis. 
- At the root of the tree all children will be split based on the first dimension (i.e. if the first dimension coordinate is less than the root it will be in the left-sub tree and if it is greater than the root it will obviously be in the right sub-tree). 
- Each level down in the tree divides on the next dimension, returning to the first dimension once all others have been exhausted. 

They most efficient way to build a k-d tree is to use a partition method like the one Quick Sort uses to place the median point at the root and everything with a smaller one dimensional value to the left and larger to the right. 

You then repeat this procedure on both the left and right sub-trees until the last trees that you are to partition are only composed of one element.


|![](http://pointclouds.org/documentation/tutorials/_images/2d_kdtree.png)|![](http://pointclouds.org/documentation/tutorials/_images/nn_kdtree.gif)|
|-|-|
|This is an example of a 2-dimensional k-d tree|This is a demonstration of hour the Nearest-Neighbor search works|

> - N 차원의 공간에서 임의의 영역에 포함되는 점들을 효율적으로 찾기 위한 트리.
> - 노드 추가와 검색은 상당히 빠르나, 삭제가 느리다. 즉 동적으로 움직이는 점들을 관리하는 데에는 적당하지 않다.



---




---

# [PCL Series 2 - Use of Kd-Tree](https://blog.csdn.net/qq_22170875/article/details/84786533)

```cpp

#include <pcl/kdtree/kdtree_flann.h> //kdtree近邻搜索
#include <pcl/io/pcd_io.h> //文件输入输出
#include <pcl/point_types.h> //点类型相关定义
#include <pcl/visualization/cloud_viewer.h> //可视化相关定义
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
//3.建立kd-tree
pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree; //建立kdtree对象
kdtree.setInputCloud(cloud); //设置需要建立kdtree的点云指针
//4.K近邻搜索
pcl::PointXYZRGB searchPoint = cloud->points[1000]; //设置查找点
int K = 900; //设置需要查找的近邻点个数
std::vector<int> pointIdxNKNSearch(K); //保存每个近邻点的索引
std::vector<float> pointNKNSquaredDistance(K); //保存每个近邻点与查找点之间的欧式距离平方
std::cout << "K nearest neighbor search at (" << searchPoint.x
<< " " << searchPoint.y
<< " " << searchPoint.z
<< ") with K=" << K << std::endl;
if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
{
for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
cloud->points[pointIdxNKNSearch[i]].r = 0;
cloud->points[pointIdxNKNSearch[i]].g = 255;
cloud->points[pointIdxNKNSearch[i]].b = 0;
}
}
std::cout << "K = 900近邻点个数：" << pointIdxNKNSearch.size() << endl;
//5.radius半径搜索
pcl::PointXYZRGB searchPoint1 = cloud->points[3500]; //设置查找点
std::vector<int> pointIdxRadiusSearch; //保存每个近邻点的索引
std::vector<float> pointRadiusSquaredDistance; //保存每个近邻点与查找点之间的欧式距离平方
float radius = 0.03; //设置查找半径范围
std::cout << "Neighbors within radius search at (" << searchPoint.x
<< " " << searchPoint.y
<< " " << searchPoint.z
<< ") with radius=" << radius << std::endl;
if (kdtree.radiusSearch(searchPoint1, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
{
for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i){
cloud->points[pointIdxRadiusSearch[i]].r = 255;
cloud->points[pointIdxRadiusSearch[i]].g = 0;
cloud->points[pointIdxRadiusSearch[i]].b = 0;
}
}
std::cout << "半径0.03近邻点个数： " << pointIdxRadiusSearch.size() << endl;
//6.显示点云
pcl::visualization::CloudViewer viewer("cloud viewer");
viewer.showCloud(cloud);
system("pause");
return 0;
}
```
---------------------
作者：chd_ayj
来源：CSDN
原文：https://blog.csdn.net/qq_22170875/article/details/84786533
版权声明：本文为博主原创文章，转载请附上博文链接！

![](https://img-blog.csdnimg.cn/20181204122116440.JPG?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3FxXzIyMTcwODc1,size_16,color_FFFFFF,t_70)