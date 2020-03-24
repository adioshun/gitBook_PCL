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

```cpp
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
#include <ctime>

//How to use a KdTree to search
//http://pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search
//Commnets : Hunjung, Lim (hunjung.lim@hotmail.com)

int
main (int argc, char** argv)
{
  
  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Intermediate/sample/cloud_cluster_0.pcd)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);	
  pcl::io::loadPCDFile<pcl::PointXYZRGB>("cloud_cluster_0.pcd", *cloud);

  // 시각적 확인을 위해 색상 통일 (255,255,255)
  for (size_t i = 0; i < cloud->points.size(); ++i){
  cloud->points[i].r = 255;
  cloud->points[i].g = 255;
  cloud->points[i].b = 255;
  }

  //KdTree 오브젝트 생성 
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (cloud);    //입력 

     //기준점(searchPoint) 설정 방법 #1(x,y,z 좌표 지정)
     //pcl::PointXYZRGB searchPoint;
     //searchPoint.x = 0.026256f;
     //searchPoint.y = -1.464739f;
     //searchPoint.z = 0.929567f;
  //기준점(searchPoint) 설정 방법 #2(3000번째 포인트)
  pcl::PointXYZRGB searchPoint = cloud->points[3000]; 

  //기준점 좌표 출력 
  std::cout << "searchPoint :" << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z  << std::endl;
   

  //기준점에서 가까운 순서중 K번째까지의 포인트 탐색 (K nearest neighbor search)
  int K = 10;   // 탐색할 포인트 수 설정 
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    //시각적 확인을 위하여 색상 변경 (0,255,0)
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
    {
      cloud->points[pointIdxNKNSearch[i]].r = 0;
      cloud->points[pointIdxNKNSearch[i]].g = 255;
      cloud->points[pointIdxNKNSearch[i]].b = 0;
    }
  }

  // 탐색된 점의 수 출력 
  std::cout << "K = 10 ：" << pointIdxNKNSearch.size() << std::endl;


  // 기준점에서 지정된 반경내 포인트 탐색 (Neighbor search within radius)
  float radius = 0.02; //탐색할 반경 설정(Set the search radius)
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    //시각적 확인을 위하여 색상 변경 (0,0,255)
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
        cloud->points[pointIdxRadiusSearch[i]].r = 0;
        cloud->points[pointIdxRadiusSearch[i]].g = 0;
        cloud->points[pointIdxRadiusSearch[i]].b = 255;
        }
  }

  // 탐색된 점의 수 출력 
  std::cout << "Radius 0.02 nearest neighbors: " << pointIdxRadiusSearch.size() << std::endl;

  // 생성된 포인트클라우드 저장 
  pcl::io::savePCDFile<pcl::PointXYZRGB>("Kdtree_AllinOne.pcd", *cloud);

  return 0;
}
```

---

- [포인트 클라우드에서 누가 누가 빠른가? KDTree](https://blog.naver.com/laonple/221207919855)





