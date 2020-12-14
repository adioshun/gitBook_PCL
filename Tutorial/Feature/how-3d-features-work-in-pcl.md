# [How 3D Features work in PCL](https://pcl.readthedocs.io/projects/tutorials/en/latest/how_features_work.html#how-3d-features-work)

- 3D feature estimation methodologies in PCL
- `pcl::Feature` class


## Theoretical primer

점군은 센서부터의 위치를 x,y,z 좌표로 표현한다. `In their native representation, points as defined in the concept of 3D mapping systems are simply represented using their Cartesian coordinates x, y, z, with respect to a given origin. `

센서가 움직이지 않는다고 가정하면 시간 t1에는 p1이 획득되고, t2에는 p2가 획득 되었다고 가정하자. `Assuming that the origin of the coordinate system does not change over time, there could be two points p1 and p2 , acquired at t1 and t2 , having the same coordinates. `

이 둘은 같은 것이라고 보장 할수 없다. 거리 정보가 그대로 여도 다른 표면을 가르치고 있을수 있다. `Comparing these points however is an ill-posed problem, because even though they are equal with respect to some distance measure (e.g. Euclidean metric), they could be sampled on completely different surfaces, and thus represent totally different information when taken together with the other surrounding points in their vicinity. `

시간이 흐를동안 세상이 움직이지 않는다는 보장이 없기 때문이다. That is because there are no guarantees that the world has not changed between t1 and t2. 


일부 센서는 위치 정보 외에도 여러 추가 정보(`intensity`, `surface remission value`,`color`)를 제공하지만 이역시 모호성을 없애지는 못한다. `Some acquisition devices might provide extra information for a sampled point, such as an intensity or surface remission value, or even a color, however that does not solve the problem completely and the comparison remains ambiguous.`

특정 서비스에서는 이러한 geometric surfaces간의 구분이 필요 할때가 있다. `Applications which need to compare points for various reasons require better characteristics and metrics to be able to distinguish between geometric surfaces.`

따라서 좌표 정보가 아닌 ``local descriptor``개념이 도입 되었다. ` The concept of a 3D point as a singular entity with Cartesian coordinates therefore disappears, and a new concept, that of local descriptor takes its place.`

많은 용어(`shape descriptors` , `geometric features`)들이 있지만 여기서는 **point feature**로 통일 하겠다. `The literature is abundant of different naming schemes describing the same conceptualization, such as shape descriptors or geometric features but for the remaining of this document they will be referred to as point feature representations.`



주변 점군들을 활용 함으로써 sampled surface geometry를 추론 할수 있게 된다. 이를 통해 비교 작업시 모호성을 줄일수 있다.  `By including the surrounding neighbors, the underlying sampled surface geometry can be inferred and captured in the feature formulation, which contributes to solving the ambiguity comparison problem. `


![](https://pcl.readthedocs.io/projects/tutorials/en/latest/_images/s/good_features.jpg)

일번적으로 같거나 비슷한 표면 위에 있는 포인트들은 서로 비슷한 특징을 가진다. 반대로 다른 표면에 있으면 다르게 된다. `Ideally, the resultant features would be very similar (with respect to some metric) for points residing on the same or similar surfaces, and different for points found on different surfaces, as shown in the figure below. `


A good point feature representation distinguishes itself from a bad one, by being able to capture the same local surface characteristics in the presence of:
- rigid transformations : that is, 3D rotations and 3D translations in the data should not influence the resultant feature vector F estimation;
- varying sampling density : in principle, a local surface patch sampled more or less densely should have the same feature vector signature;
- noise : the point feature representation must retain the same or very similar values in its feature vector in the presence of mild noise in the data.

일반적으로 PCL features은 요청된 포인트의 이웃 점들을 계산 하기 위하여 근사방법(approximate methods)을 사용한다. In general, PCL features use approximate methods to compute the nearest neighbors of a query point, using fast kd-tree queries. 

There are two types of queries that we’re interested in:
- K개의 근접 서치 : determine the k (user given parameter) neighbors of a query point (also known as k-search);
- R반경내 근접 서치 : determine all the neighbors of a query point within a sphere of radius r (also known as radius-search).


## How to pass the input

As almost all classes in PCL that inherit from the base pcl::PCLBase class, the pcl::Feature class accepts input data in two different ways:

- an entire point cloud dataset, given via setInputCloud (PointCloudConstPtr &) - mandatory
    - Any feature estimation class with attempt to estimate a feature at every point in the given input cloud.

- a subset of a point cloud dataset, given via setInputCloud (PointCloudConstPtr &) and setIndices (IndicesConstPtr &) - optional

    - Any feature estimation class will attempt to estimate a feature at every point in the given input cloud that has an index in the given indices list. 
    - By default, if no set of indices is given, all points in the cloud will be considered.*



Because setInputCloud() is always required, there are up to four combinations that we can create using ` <setInputCloud(), setIndices(), setSearchSurface()>`. 


![](https://pcl.readthedocs.io/projects/tutorials/en/latest/_images/features_input_explained.png)


### 1. setIndices() = false, setSearchSurface() = false 

- 가장 일반적으로 사용됨. 입력된 점군에 대한 모든 feature를 구 `this is without a doubt the most used case in PCL, where the user is just feeding in a single PointCloud dataset and expects a certain feature estimated at all the points in the cloud.`

- Since we do not expect to maintain different implementation copies based on whether a set of indices and/or the search surface is given, whenever indices = false, PCL creates a set of internal indices (as a std::vector<int>) that basically point to the entire dataset (indices=1..N, where N is the number of points in the cloud).

- 절차 
 - First, we estimate the nearest neighbors of p_1, 
 - then the nearest neighbors of p_2, and so on, until we exhaust all the points in P.


### 2. setIndices() = true, setSearchSurface() = false 

- 인덱스가 있는 포인트들에 대하여서만 feature를 계산 `the feature estimation method will only compute features for those points which have an index in the given indices vector;`

- P2의 인덱스는 주어지지 않았으므로 P2의 feature나 neighbor를 계산 하지 않음 `Here, we assume that p_2’s index is not part of the indices vector given, so no neighbors or features will be estimated at p2.`


### 3. setIndices() = false, setSearchSurface() = true 

- 첫번째와 비슷하게 모든 입력에 대하여 Feature를 계산 한다. `as in the first case, features will be estimated for all points given as input,`

- but, the underlying neighboring surface given in setSearchSurface() will be used to obtain nearest neighbors for the input points, rather than the input cloud itself;

- If Q={q_1, q_2} is another cloud given as input, different than P, and P is the search surface for Q, then the neighbors of q_1 and q_2 will be computed from P.
 
### 4. setIndices() = true, setSearchSurface() = true 

- 많이 사용되지 않는 경우이다. indices 와  search surface가 모두 주어져있다.  `this is probably the rarest case, where both indices and a search surface is given.`

- In this case, features will be estimated for only a subset from the <input, indices> pair, using the search surface information given in setSearchSurface().

- Here, we assume that q_2’s index is not part of the indices vector given for Q, so no neighbors or features will be estimated at q2.


####### 활용예 

The most useful example when setSearchSurface() should be used, is when we have a very dense input dataset, but we do not want to estimate features at all the points in it, but rather at some keypoints discovered using the methods in pcl_keypoints, or at a downsampled version of the cloud (e.g., obtained using a pcl::VoxelGrid<T> filter). 

In this case, we pass the downsampled/keypoints input via setInputCloud(), and the original data as setSearchSurface().

---

## An example for normal estimation

방법이 정해 지면 요청되는 점의 이웃 점군들을 사용하여 Local Feature를 계산한다. `Once determined, the neighboring points of a query point can be used to estimate a local feature representation that captures the geometry of the underlying sampled surface around the query point. `

점의 방향(orientation)을 계산 하는것이 중요하다. 이를 Normal 이라 한다. `An important problem in describing the geometry of the surface is to first infer its orientation in a coordinate system, that is, estimate its normal. `

Surface normals은 중요한 특징 정보이며 여러 곳에서 많이 사용된다. `Surface normals are important properties of a surface and are heavily used in many areas such as computer graphics applications to apply the correct light sources that generate shadings and other visual effects (See [RusuDissertation] for more information).`



### 1. Estimate a set of surface normals for **all** the points in the input dataset.


```cpp
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

// How 3D Features work in PCL
// http://pointclouds.org/documentation/tutorials/how_features_work.php

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // *.PCD 파일 읽기 (https://raw.githubusercontent.com/adioshun/gitBook_Tutorial_PCL/master/Beginner/sample/tabletop.pcd)
  pcl::io::loadPCDFile<pcl::PointXYZ> ("tabletop.pcd", *cloud);
  
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);  
  ne.setRadiusSearch (0.03); // Use all neighbors in a sphere of radius 3cm  
  ne.compute (*cloud_normals); // Compute the features

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()
   // 포인트수 출력  
  std::cout << "Filtered " << cloud_normals->points.size ()  << std::endl;
}
```

---

### 2. Estimate a set of surface normals for a **subset** of the points in the input dataset.(인덱스 이용)

```cpp
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  ... read, pass in or create a point cloud ...

  // Create a set of indices to be used. For simplicity, we're going to be using the first 10% of the points in cloud
  std::vector<int> indices (floor (cloud->points.size () / 10));
  for (size_t i = 0; i < indices.size (); ++i) indices[i] = i;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

  // Pass the indices
  boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (indices));
  ne.setIndices (indicesptr);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input indicesptr->size ()
}
```

---

### 3. Estimate a set of surface normals for all the points in the input datase

 - but will estimate their nearest neighbors using another dataset. 
 - As previously mentioned, a good usecase for this is when the input is a downsampled version of the surface.
 
```cpp

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZ>);

  ... read, pass in or create a point cloud ...

  ... create a downsampled version of it ...

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud_downsampled);

  // Pass the original data (before downsampling) as the search surface
  ne.setSearchSurface (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given surface dataset.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  ne.setRadiusSearch (0.03);

  // Compute the features
  ne.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud_downsampled->points.size ()
}
```
