# [Removing outliers using a Statistical OutlierRemoval filter](http://pointclouds.org/documentation/tutorials/statistical_outlier.php#statistical-outlier-removal)


In this tutorial we will learn how to remove noisy measurements, (e.g. outliers), from a point cloud dataset using statistical analysis techniques.



## Background

Laser scans typically generate point cloud datasets of varying point densities. Additionally, measurement errors lead to sparse outliers which corrupt the results even more. This complicates the estimation of local point cloud characteristics such as surface normals or curvature changes, leading to erroneous values, which in turn might cause point cloud registration failures. Some of these irregularities can be solved by performing a statistical analysis on each point’s neighborhood, and trimming those which do not meet a certain criteria. Our sparse outlier removal is based on the computation of the distribution of point to neighbors distances in the input dataset. For each point, we compute the mean distance from it to all its neighbors. By assuming that the resulted distribution is Gaussian with a mean and a standard deviation, all points whose mean distances are outside an interval defined by the global distances mean and standard deviation can be considered as outliers and trimmed from the dataset.

The following picture shows the effects of the sparse outlier analysis and removal: the original dataset is shown on the left, while the resultant one on the right. The graphic shows the mean k-nearest neighbor distances in a point neighborhood before and after filtering.


```python 

def do_statistical_outlier_filtering(pcl_data,mean_k,tresh):
    '''
    :param pcl_data: point could data subscriber
    :param mean_k:  number of neighboring points to analyze for any given point
    :param tresh:   Any point with a mean distance larger than global will be considered outlier
    :return: Statistical outlier filtered point cloud data
    eg) cloud = do_statistical_outlier_filtering(cloud,10,0.001)
    : https://github.com/fouliex/RoboticPerception
    '''
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(mean_k)
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()

    
cloud = do_statistical_outlier_filtering(cloud,10,0.001)
    
```

    입력 cloud포맷 : pcl_xyz 
    pcl_xyz = pcl_helper.XYZRGB_to_XYZ(pcl_xyzrgb)
    pcl_xyzrgb시 : TypeError: __cinit__() takes exactly 1 positional argument (0 given) 에러 



---

The statistical outlier removal process is a bit more refined. First, for every point, the mean distance to its K neighbors is computed. Then, if we asume that the result is a normal (gaussian) distribution with a mean μ and a standard deviation σ, we can deem it safe to remove all points with mean distances that fall out of the global mean plus deviation. Basically, it runs a statistical analysis of the distances between neighboring points, and trims all which are not considered "normal" (you define what "normal" is with the parameters of the algorithm).