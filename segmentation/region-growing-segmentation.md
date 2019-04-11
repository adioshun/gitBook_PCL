# [Region growing segmentation](http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php#region-growing-segmentation)


> 물체의 normals과 curvatures정보를 이용하여 비슷한것끼리 묶어서 그룹으로 처리 

- This is also a greedy-like, flood fill approach like the Euclidean one

-  The angle between their **normals** and the difference of **curvatures** are checked to see if they could belong to the same smooth surface.


유클리드 방식과의 차이점 : Think about a box laying on a table: 
- 유클리드는 테이블의 위 상자의 경우 붙어 있으므로 하나로 간주한다. with Euclidean segmentation, both would be considered to be in the same cluster because they are "touching". 
- 리전그로위은 테이블과 상자의 Normal정보가 다름으로 분류가 가능하다. With region growing segmentation, this would not happen, because there is a 90° (with ideal normal estimation, that is) difference between the normals of a point in the table and another one in the box's lateral.

---




[RegionGrowing.py](https://github.com/Panchamy/RegionGrowing/blob/master/RegionGrowing.py)

[Wrapping C with Python: 3D image segmentation with region growing](http://notmatthancock.github.io/2017/10/09/region-growing-wrapping-c.html)

[Region Growing Segmentation](http://simpleitk-prototype.readthedocs.io/en/latest/user_guide/segmentation/plot_region_growing.html)


---

## python-pcl활용 (아직 적용 안된듯)

```python
#https://github.com/strawlab/python-pcl/blob/master/tests/test_segmentation.py
import pcl
p = pcl.load("./table_scene_lms400.pcd")
vg = p.make_voxel_grid_filter()
vg.set_leaf_size(0.01, 0.01, 0.01)

cloud_filtered = vg.filter()
tree = cloud_filtered.make_kdtree()
segment = cloud_filtered.make_RegionGrowing(ksearch=50)
segment.set_MinClusterSize(100)
segment.set_MaxClusterSize(25000)
segment.set_NumberOfNeighbours(5)
segment.set_SmoothnessThreshold(0.2)
segment.set_CurvatureThreshold(0.05)
segment.set_SearchMethod(tree)
cluster_indices = segment.Extract()

cloud_cluster = pcl.PointCloud()
```

---
## pypcl활용 

```python 
# https://github.com/cmpute/pypcl/blob/master/test/segment_test.py
# https://raw.githubusercontent.com/cmpute/pypcl/depricated/test/segment_test.py


'''
Tests of pcl.segment
'''

import os
import sys
import logging
import numpy as np
from numpy.random import RandomState
import pytest
sys.path.append(os.path.dirname(__file__) + '/' + os.path.pardir)
import pcl
import pcl.segment as ps
import pcl.features as pf

def test_regiongrow():
    '''
    Test RegionGrowing
    '''

    cloud = pcl.PointCloud(np.random.rand(200, 3), ['x', 'y', 'z'])
    # cloud = pcl.io.loadpcd(os.path.dirname(__file__) + '/data/car6.pcd')
    regiongrow = ps.RegionGrowing(cloud)
    with pytest.raises(ValueError): # No normal is inputted
        regiongrow.extract()
    nestimate = pf.NormalEstimation(cloud)
    nestimate.search_k = 10
    normals = nestimate.compute()
    regiongrow.input_normals = normals
    regions = regiongrow.extract()
    # TODO: visual or other checking is needed

    cluster = regions[-1]
    assert regiongrow.get_segment_from_point(cluster[-1]) == cluster
    color = regiongrow.get_colored_cloud()
    colora = regiongrow.get_colored_cloud_rgba()
    assert (colora.rgba['a'] == 255).all()
    coloredcloud = cloud + color
    coloredclouda = cloud + colora
    assert 'rgb' in coloredcloud.names
    assert 'rgba' in coloredclouda.names

if __name__ == '__main__':
    pytest.main([__file__, '-s'])
```