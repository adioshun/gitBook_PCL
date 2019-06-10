# [Removing outliers using a Statistical OutlierRemoval filter](http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal)


In this tutorial we will learn how to remove noisy measurements, (e.g. outliers), from a point cloud dataset using statistical analysis techniques.



## Background

Laser scans typically generate point cloud datasets of varying point densities. Additionally, measurement errors lead to sparse outliers which corrupt the results even more. This complicates the estimation of local point cloud characteristics such as surface normals or curvature changes, leading to erroneous values, which in turn might cause point cloud registration failures. Some of these irregularities can be solved by performing a statistical analysis on each point’s neighborhood, and trimming those which do not meet a certain criteria. Our sparse outlier removal is based on the computation of the distribution of point to neighbors distances in the input dataset. For each point, we compute the mean distance from it to all its neighbors. By assuming that the resulted distribution is Gaussian with a mean and a standard deviation, all points whose mean distances are outside an interval defined by the global distances mean and standard deviation can be considered as outliers and trimmed from the dataset.

The following picture shows the effects of the sparse outlier analysis and removal: the original dataset is shown on the left, while the resultant one on the right. The graphic shows the mean k-nearest neighbor distances in a point neighborhood before and after filtering.




---
동작 과정 
- First, for every point, the mean distance to its K neighbors is computed. 
- Then, if we asume that the result is a normal (gaussian) distribution with a mean μ and a standard deviation σ, we can deem it safe to remove all points with mean distances that fall out of the global mean plus deviation. 

Basically, it runs a statistical analysis of the distances between neighboring points, and trims all which are not considered "normal" (you define what "normal" is with the parameters of the algorithm).