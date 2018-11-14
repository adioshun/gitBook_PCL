# [Removing outliers using a Conditional or Radius Outlier removal](http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers)

> https://github.com/strawlab/python-pcl/blob/master/examples/official/Filtering/remove_outliers.py

## Conditional removal 

- x,y,z값을 GT, GE, LT, LE, EQ 조건에 따라 필터링 



## Radius Outlier removal


The radius-based outlier removal is the simplest method of all. You must specify a search radius and the minimum number of neighbors than a point must have to avoid being labelled as outlier. The algorithm will then iterate through all points (which can be slow in if the cloud is big) and perform the check: if less than that number of points are found within the radius, it is removed.



## Statistical Outlier removal

> 



