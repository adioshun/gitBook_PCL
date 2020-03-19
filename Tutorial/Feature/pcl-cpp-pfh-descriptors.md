# [Point Feature Histograms (PFH) descriptors](http://pointclouds.org/documentation/tutorials/pfh_estimation.php#pfh-estimation)

> [PCL/OpenNI tutorial](https://github.com/adioshun/gitBook_PCL/tree/master/Tutorial/Feature#11-pfh-point-feature-histogram)

포인트 특징 표현법 치고는 표면 법선과 곡선 추론은 기본적인 것이다. `As point feature representations go, surface normals and curvature estimates are somewhat basic in their representations of the geometry around a specific point. `

비록 계산이 간단하여 속도가 빠르지만 자세한 정보를 표현하기에는 어렵다. `Though extremely fast and easy to compute, they cannot capture too much detail, as they approximate the geometry of a point’s k-neighborhood with only a few values. `

As a direct consequence, most scenes will contain many points with the same or very similar feature values, thus reducing their informative characteristics.

This tutorial introduces a family of 3D feature descriptors coined PFH (Point Feature Histograms) for simplicity, presents their theoretical advantages and discusses implementation details from PCL’s perspective. 

PFH는 xyz 데이터와 노멀과 관계가 있으므로  노멀에 대한 이해가 필요 하다. `As a prerequisite, please go ahead and read the  [Estimating Surface Normals in a PointCloud]  tutorial first, as PFH signatures rely on both xyz 3D data as well as surface normals.`

# Theoretical primer

PFH의 목적은 포인트의 주변 포인트의 기하학적 특징을 곡선평균을 일반화하여 입력 하는 것이다. 이때 다차원 히스토그램을 사용한다.  `The goal of the PFH formulation is to encode a point’s k-neighborhood geometrical properties by generalizing the mean curvature around the point using a multi-dimensional histogram of values. `

This highly dimensional hyperspace provides an informative signature for the feature representation, is invariant to the 6D pose of the underlying surface, and copes very well with different sampling densities or noise levels present in the neighborhood.

PFH은 주변 포인트들과 그들의 표면 법선과의 관계를 기반으로 하고 있따. `A Point Feature Histogram representation is based on the relationships between the points in the k-neighborhood and their estimated surface normals. `

Simply put, it attempts to capture as best as possible the sampled surface variations by taking into account all the interactions between the directions of the estimated normals. 

 **The resultant hyperspace is thus dependent on the quality of the surface normal estimations at each point.**

The figure below presents an influence region diagram of the PFH computation for a query point (![p_q](http://pointclouds.org/documentation/tutorials/_images/math/2fa9878d2c9dd8bc006ba9d7986fab0030ed8452.png)), marked with red and placed in the middle of a circle (sphere in 3D) with radius  **r**, and all its  **k**  neighbors (points with distances smaller than the radius  **r**) are fully interconnected in a mesh. The final PFH descriptor is computed as a histogram of relationships between all pairs of points in the neighborhood, and thus has a computational complexity of  ![O(k^2)](http://pointclouds.org/documentation/tutorials/_images/math/0c54ec2390ec7c7cc19ef74f38272948b253af7d.png).

> 생략 

To create the final PFH representation for the query point, the set of all quadruplets is binned into a histogram. 

The binning process divides each features’s value range into **b** subdivisions, and counts the number of occurrences in each subinterval. 

Since three out of the four features presented above are measure of the angles between normals, their values can easily be normalized to the same interval on the trigonometric circle. 

A binning example is to divide each feature interval into the same number of equal parts, and therefore create a histogram with b^4 bins in a fully correlated space. 

In this space, a histogram bin increment corresponds to a point having certain values for all its 4 features. 

The figure below presents examples of Point Feature Histograms representations for different points in a cloud.

In some cases, the fourth feature, **d**, does not present an extreme significance for 2.5D datasets, usually acquired in robotics, as the distance between neighboring points increases from the viewpoint. Therefore, omitting **d** for scans where the local point density influences this feature dimension has proved to be beneficial.

## Estimating PFH features

Point Feature Histograms are implemented in PCL as part of the  [pcl_features](http://docs.pointclouds.org/trunk/a02944.html)  library.

The default PFH implementation uses 5 binning subdivisions (e.g., each of the four feature values will use this many bins from its value interval), and does not include the distances (as explained above – although the  **computePairFeatures**  method can be called by the user to obtain the distances too, if desired) which results in a 125-byte array (5^3) of float values. 

These are stored in a  **pcl::PFHSignature125**  point type.

The following code snippet will estimate a set of PFH features for all the points in the input dataset.

