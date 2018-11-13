# [Estimating VFH(Viewpoint Feature Histogram) signatures for a set of points](http://pointclouds.org/documentation/tutorials/vfh_estimation.php#vfh-estimation)




#### python-pcl 코드 완성 안된듯 

```python 

import pcl
cloud = pcl.load("/workspace/_pcd/1530509312.094227000.pcd")
print(cloud)

search_method = cloud.make_kdtree()
feature = cloud.make_VFHEstimation()
feature.set_SearchMethod(search_method)
feature.set_KSearch(50)
kfh = feature.compute() #??? 없음
```