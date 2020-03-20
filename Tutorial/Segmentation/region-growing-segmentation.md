# [Region growing segmentation](http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php#region-growing-segmentation)

> [region_growing_tutorial.pcd](https://raw.github.com/PointCloudLibrary/data/master/tutorials/region_growing_tutorial.pcd) 사용 

In this tutorial we will learn how to use the **region growing algorithm** implemented in the `pcl::RegionGrowing` class. 

목적 : **smoothness constraint**를 기준으로 비슷한것 묶음 `The purpose of the said algorithm is to merge the points that are close enough in terms of the smoothness constraint. `

따라서 결과물들은 비슷한 smooth surface끼리 그룹화 된다. `Thereby, the output of this algorithm is the set of clusters, were each cluster is a set of points that are considered to be a part of the same smooth surface. `

원리 : 포인트 Normal간의 각도(angle)비교 `The work of this algorithm is based on the comparison of the angles between the points normals.`

## Theoretical Primer

포인드들을 곡률에 따라 정렬한다. `First of all it sorts the points by their curvature value. `

위 작업을 필수 적인데 이유는 시작점인 최소 곡률값을 찾기 위해서 이다. `It needs to be done because the region begins its growth from the point that has the minimum curvature value. `

최소 곡률값을 찾는 이유는 이 곳이 평면에 위치 하기 때문이다. 평면에서 시작해야 전체 세그멘트의 양을 줄일수 있다. `The reason for this is that the point with the minimum curvature is located in the flat area (growth from the flattest area allows to reduce the total number of segments).`

So we have the sorted cloud. Until there are unlabeled points in the cloud, algorithm picks up the point with minimum curvature value and starts the growth of the region. 


This process occurs as follows:

> 생략 

---

> 물체의 normals과 curvatures정보를 이용하여 비슷한것끼리 묶어서 그룹으로 처리 

- This is also a greedy-like, flood fill approach like the Euclidean one

-  The angle between their **normals** and the difference of **curvatures** are checked to see if they could belong to the same smooth surface.


유클리드 방식과의 차이점 : `Think about a box laying on a table: `
- 유클리드는 테이블의 위 상자의 경우 붙어 있으므로 하나로 간주한다. `with Euclidean segmentation, both would be considered to be in the same cluster because they are "touching". `
- 리전그로위은 테이블과 상자의 Normal정보가 다름으로 분류가 가능하다. `With region growing segmentation, this would not happen, because there is a 90° (with ideal normal estimation, that is) difference between the normals of a point in the table and another one in the box's lateral.`




## Code 


```cpp

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  if ( pcl::io::loadPCDFile <pcl::PointXYZ> ("region_growing_tutorial.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::search::Search<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  int counter = 0;
  while (counter < clusters[0].indices.size ())
  {
    std::cout << clusters[0].indices[counter] << ", ";
    counter++;
    if (counter % 10 == 0)
      std::cout << std::endl;
  }
  std::cout << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::CloudViewer viewer ("Cluster viewer");
  viewer.showCloud(colored_cloud);
  while (!viewer.wasStopped ())
  {
  }

  return (0);
}





```



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
segment.set_NumberOfNeighbours(5)https://legacy.gitbook.com/book/adioshun/pcl/edit#
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
