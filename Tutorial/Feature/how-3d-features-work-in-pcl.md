# [How 3D Features work in PCL](http://pointclouds.org/documentation/tutorials/how_features_work.php)

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


![](http://pointclouds.org/documentation/tutorials/_images/good_features.jpg)

일번적으로 같거나 비슷한 표면 위에 있는 포인트들은 서로 비슷한 특징을 가진다. 반대로 다른 표면에 있으면 다르게 된다. `Ideally, the resultant features would be very similar (with respect to some metric) for points residing on the same or similar surfaces, and different for points found on different surfaces, as shown in the figure below. `


A good point feature representation distinguishes itself from a bad one, by being able to capture the same local surface characteristics in the presence of:
- rigid transformations : that is, 3D rotations and 3D translations in the data should not influence the resultant feature vector F estimation;
- varying sampling density : in principle, a local surface patch sampled more or less densely should have the same feature vector signature;
- noise : the point feature representation must retain the same or very similar values in its feature vector in the presence of mild noise in the data.

일반적으로 PCL features은 요청된 포인트에 대하여 이웃 점들을 계산 하기 위하여 In general, PCL features use approximate methods to compute the nearest neighbors of a query point, using fast kd-tree queries. 

There are two types of queries that we’re interested in:

determine the k (user given parameter) neighbors of a query point (also known as k-search);
determine all the neighbors of a query point within a sphere of radius r (also known as radius-search).


