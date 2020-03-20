# [Euclidean Cluster Extraction](http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction)

In this tutorial we will learn how to extract Euclidean clusters with the `pcl::EuclideanClusterExtraction` class. 

> Plane model segmentation은 다른 곳에서 별도 확인 바람 `In order to not complicate the tutorial, certain elements of it such as the plane segmentation algorithm, will not be explained here. Please check the Plane model segmentation tutorial for more information.`


## Theoretical Primer

군집화 알고리즘은 점군을 작은 부분으로 나누어서 처리 시간을 줄일 필요가 있다. `A clustering method needs to divide an unorganized point cloud model P into smaller parts so that the overall processing time for P is significantly reduced. `

간단한 방법은 점군을 옥트리 구조로 만들어 유클리드 방식을 적용하는 것이다. `A simple data clustering approach in an Euclidean sense can be implemented by making use of a 3D grid subdivision of the space using fixed width boxes, or more generally, an octree data structure. `

옥트리 구조는 생성이 빠르고 occupied space에 대한 볼륨적 정보가 필요 할때 유용하다. `This particular representation is very fast to build and is useful for situations where either a volumetric representation of the occupied space is needed, `or the data in each resultant 3D box (or octree leaf) can be approximated with a different structure. 

In a more general sense however, we can make use of nearest neighbors and implement a clustering technique that is essentially similar to a flood fill algorithm.

책상위에 물체가 있는 점군 데이터가 있을때 각 물체를 군집화 해보자 `Let’s assume we have given a point cloud with a table and objects on top of it. We want to find and segment the individual object point clusters lying on the plane. `

Assuming that we use a Kd-tree structure for finding the nearest neighbors, the algorithmic steps for that would be (from [RusuDissertation]):

동작 과정은 아래와 같다. 
1. 입력 점군을 이용하여 kd-tree구조체를 생성한다. `create a Kd-tree representation for the input point cloud dataset P;`
2. 빈 리스트의 클러스터 c를 생성한다. 점검할 점군을 q로 추가 한다.`set up an empty list of clusters C, and a queue of the points that need to be checked Q;`
3. 다음 모든 점군에 대하여 아래 절차를 진행 한다. `then for every point \boldsymbol{p}_i \in P, perform the following steps:`
	- 점군을 큐 Q에 추가 한다. `add \boldsymbol{p}_i to the current queue Q;`
	- 큐에 있는 포인트 P에 대하여 `for every point \boldsymbol{p}_i \in Q do:`
		- 지정된 반지름 안에 위치한 이웃 점군을 구한다. `search for the set P^i_k of point neighbors of \boldsymbol{p}_i in a sphere with radius r < d_{th};`
		- for every neighbor \boldsymbol{p}^k_i \in P^k_i, check if the point has already been processed, and if not add it to Q;
		- when the list of all points in Q has been processed, add Q to the list of clusters C, and reset Q to an empty list
4. the algorithm terminates when all points \boldsymbol{p}_i \in P have been processed and are now part of the list of point clusters C


## code 

```cpp 
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; 

ec.setClusterTolerance (0.02); // 2cm
ec.setMinClusterSize (100);
ec.setMaxClusterSize (25000);
ec.setSearchMethod (tree);
ec.setInputCloud (cloud_filtered);
ec.extract (cluster_indices);
```

- `pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec` : Here we are creating a EuclideanClusterExtraction object with point type PointXYZ since our point cloud is of type PointXYZ. 
- `ec.setClusterTolerance (0.02)` : We are also setting the parameters and variables for the extraction. 
	- 값이 작으면, If you take a very small value, it can happen that an actual object can be seen as multiple clusters. 
	- 값이 크면, On the other hand, if you set the value too high, it could happen, that multiple objects are seen as one cluster. 
	- 테스트 해보며 찾아야 함. So our recommendation is to just test and try out which value suits your dataset.

- `ec.setMinClusterSize (100)` & `ec.setMaxClusterSize (25000)` : 최소 포인트수와 최대 포인트 수 지정 `We impose that the clusters found must have at least setMinClusterSize() points and maximum setMaxClusterSize() points`

