# [Estimating Surface Normals in a PointCloud](http://pointclouds.org/documentation/tutorials/normal_estimation.php#normal-estimation)

> Normal = 수직 방향, Surface Normal = 표면에 수직 방향
> 2차원 = Normal Line / 3차원 Surface normal(OR simply **normal**)

주요한 특징으로 조명에 의한 그림자 생성등 시각 효과증에서 많이 사용된다. `Surface normals are important properties of a geometric surface, and are heavily used in many areas such as computer graphics applications, to apply the correct light sources that generate shadings and other visual effects.`

기하학적 표면이 주어 졌을때, 표면의 특정 지점에서 수직인 벡터로 추론하는 것은 쉽다.`Given a geometric surface, it’s usually trivial to infer the direction of the normal at a certain point on the surface as the vector perpendicular to the surface in that point. `

However, since the point cloud datasets that we acquire represent a set of point samples on the real surface, there are two possibilities:
- obtain the underlying surface from the acquired point cloud dataset, using surface meshing techniques, and then compute the surface normals from the mesh;
- use approximations to infer the surface normals from the point cloud dataset directly.


## Theoretical primer

법선 추론 방법은 여러가지가 있다. 여기서는 가장 간단한 방법을 살펴 보겠다. `Though many different normal estimation methods exist, the one that we will concentrate on this tutorial is one of the simplest, and is formulated as follows. `


The problem of determining the normal to a point on the surface is approximated by the problem of estimating the normal of a plane tangent to the surface, which in turn becomes a least-square plane fitting estimation problem.

**[중요]** 표면 법선을 추론하는 것은 이웃 점들로 생성된 covariance matrix의 eigenvectors  eigenvalues를 분석 하는것으로 간주 할수 있다. `The solution for estimating the surface normal is therefore reduced to an analysis of the eigenvectors and eigenvalues(or PCA) of a covariance matrix created from the nearest neighbors of the query point. `


좀더 정확히는 각점 P_i에 **covariance matrix** C를 어셈블 하는 것이다. 

$$
\mathcal{C} = \frac{1}{k}\sum_{i=1}^{k}{\cdot (\boldsymbol{p}_i-\overline{\boldsymbol{p}})\cdot(\boldsymbol{p}_i-\overline{\boldsymbol{p}})^{T}}, ~\mathcal{C} \cdot \vec{{\mathsf v}_j} = \lambda_j \cdot \vec{{\mathsf v}_j},~ j \in \{0, 1, 2\}
$$

- k is the number of point neighbors considered in the neighborhood of p_i
- $$\overline{\boldsymbol{p}} $$ represents the 3D centroid of the nearest neighbors, 
- $$\lambda_j$$ is the j-th eigenvalue of the covariance matrix, 
- $$ \vec{{\mathsf v}_j}$$ the j-th eigenvector.


To estimate a **covariance matrix** from a set of points in PCL, you can use:

```cpp

  // Placeholder for the 3x3 covariance matrix at each surface patch
  Eigen::Matrix3f covariance_matrix;
  // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
  Eigen::Vector4f xyz_centroid;

  // Estimate the XYZ centroid
  compute3DCentroid (cloud, xyz_centroid);

  // Compute the 3x3 covariance matrix
  computeCovarianceMatrix (cloud, xyz_centroid, covariance_matrix);
```

###### 기존 문제점 

일반적으로 수학적 방식으로 위 문제를 해결 할수 없어 PCA를 이용하여 orientation을 계산 하는것은 좋은 성과를 보이지 않는다. `In general, because there is no mathematical way to solve for the sign of the normal, its orientation computed via Principal Component Analysis (PCA) as shown above is ambiguous, and not consistently oriented over an entire point cloud dataset. `

The figure below presents these effects on two sections of a larger dataset representing a part of a kitchen environment. 
- The right part of the figure presents the Extended Gaussian Image (EGI), also known as the normal sphere, which describes the orientation of all normals from the point cloud. 
- Since the datasets are 2.5D and have thus been acquired from a single viewpoint, normals should be present only on half of the sphere in the EGI. However, due to the orientation inconsistency, they are spread across the entire sphere.

|![](http://pointclouds.org/documentation/tutorials/_images/unflipped_scene2.jpg)|![](http://pointclouds.org/documentation/tutorials/_images/unflipped_sphere.jpg)|
|-|-|

###### 해결책 

The solution to this problem is trivial if the viewpoint $${\mathsf v}_p$$ is in fact known. 

To orient all normals $$\vec{\boldsymbol{n}}_i$$ consistently towards the viewpoint, they need to satisfy the equation: $$\vec{\boldsymbol{n}}_i \cdot ({\mathsf v}_p - \boldsymbol{p}_i) > 0$$


The figure below presents the results after all normals in the datasets from the above figure have been consistently oriented towards the viewpoint.

|![](http://pointclouds.org/documentation/tutorials/_images/flipped_scene2.jpg)|![](http://pointclouds.org/documentation/tutorials/_images/flipped_sphere.jpg)|
|-|-|

To re-orient a given point normal manually in PCL, you can use:
```cpp
flipNormalTowardsViewpoint (const PointT &point, float vp_x, float vp_y, float vp_z, Eigen::Vector4f &normal);
```

## Selecting the right scale

법선 추론은 주변의 점군을 이용하여 구한다. 이때 얼마만큼의 주변 점군을 고려 하지 정하는게 중요하다. (eg. K or Radious) `As previously explained, a surface normal at a point needs to be estimated from the surrounding point neighborhood support of the point (also called k-neighborhood). The specifics of the nearest-neighbor estimation problem raise the question of the right scale factor: given a sampled point cloud dataset , what are the correct k (given via pcl::Feature::setKSearch) or r (given via pcl::Feature::setRadiusSearch) values that should be used in determining the set of nearest neighbors of a point?`

This issue is of extreme importance and constitutes a limiting factor in the automatic estimation (i.e., without user given thresholds) of a point feature representation. 

아래 그림은 R,K, 값에 따른 결과 이다. `To better illustrate this issue, the figure below presents the effects of selecting a smaller scale (i.e., small r or k) versus a larger scale (i.e., large r or k). `


![](http://pointclouds.org/documentation/tutorials/_images/normals_different_radii.jpg)

The left part of the figures depicts a reasonable well chosen scale factor, with estimated surface normals approximately perpendicular for the two planar surfaces and small edges visible all across the table. 

If the scale factor however is too big (right part), and thus the set of neighbors is larger covering points from adjacent surfaces, the estimated point feature representations get distorted, with rotated surface normals at the edges of the two planar surfaces, and smeared edges and suppressed fine details.


## Estimating the normals


```cpp
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  ... read, pass in or create a point cloud ...

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);

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

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}
```

수행 절차 
```
for each point p in cloud P
  1. get the nearest neighbors of p
  2. compute the surface normal n of p
  3. check if n is consistently oriented towards the viewpoint and flip otherwise
```

The viewpoint is by default (0,0,0) and can be changed with: `setViewPoint (float vpx, float vpy, float vpz);`

To compute a single point normal, use:

```cpp
computePointNormal (const pcl::PointCloud<PointInT> &cloud, const std::vector<int> &indices, Eigen::Vector4f &plane_parameters, float &curvature);`
```

Where cloud is the input point cloud that contains the points, indices represents the set of k-nearest neighbors from cloud, and plane_parameters and curvature represent the output of the normal estimation, with plane_parameters holding the normal (nx, ny, nz) on the first 3 coordinates, and the fourth coordinate is D = nc . p_plane (centroid here) + p. The output surface curvature is estimated as a relationship between the eigenvalues of the covariance matrix (as presented above), as:

$$\sigma = \frac{\lambda_0}{\lambda_0 + \lambda_1 + \lambda_2}$$

## Speeding Normal Estimation with OpenMP

멀티코어를 이용하여 속도 향상을 원한다면 `For the speed-savvy users, PCL provides an additional implementation of surface normal estimation which uses multi-core/multi-threaded paradigms using OpenMP to speed the computation. `


The name of the class is `pcl::NormalEstimationOMP`, and its API is 100% compatible to the single-threaded `pcl::NormalEstimation`, which makes it suitable as a drop-in replacement. 

On a system with 8 cores, you should get anything between 6-8 times faster computation times.



RGB-D센서 데이터(Ordered cloud point)라면 **Normal Estimation Using Integral Images.** 추천 

---


# Surface curvature estimates


<a href="http://www.youtube.com/watch?feature=player_embedded&v=d3z35hpwPZQ" target="_blank">
<img src="http://img.youtube.com/vi/d3z35hpwPZQ/0.jpg" alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>
- Red = low curvature, 
- green-blue = curvature.

Normals are also important because they give us information about the curvature of the surface at some point

--- 

[How to visualize surface normals as Marker::Arrow for each point in rviz?](https://answers.ros.org/question/9095/how-to-visualize-surface-normals-as-markerarrow-for-each-point-in-rviz/): 부하때문에 어려움 

[pcl_normal_visualization.cpp](http://docs.ros.org/groovy/api/pcl_cloud_tools/html/pcl__normal__visualization_8cpp_source.html)