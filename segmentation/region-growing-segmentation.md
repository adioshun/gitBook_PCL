# [Region growing segmentation](http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php#region-growing-segmentation)


> 물체의 normals과 curvatures정보를 이용하여 비슷한것끼리 묶어서 그룹으로 처리 

- This is also a greedy-like, flood fill approach like the Euclidean one

-  The angle between their **normals** and the difference of **curvatures** are checked to see if they could belong to the same smooth surface.


유클리드 방식과의 차이점 : Think about a box laying on a table: 
- 유클리드는 테이블의 위 상자의 경우 붙어 있으므로 하나로 간주한다. with Euclidean segmentation, both would be considered to be in the same cluster because they are "touching". 
- 리전그로위은 테이블과 상자의 Normal정보가 다름으로 분류가 가능하다. With region growing segmentation, this would not happen, because there is a 90° (with ideal normal estimation, that is) difference between the normals of a point in the table and another one in the box's lateral.

---

https://github.com/cmpute/pypcl/blob/master/test/segment_test.py


[RegionGrowing.py](https://github.com/Panchamy/RegionGrowing/blob/master/RegionGrowing.py)

[Wrapping C with Python: 3D image segmentation with region growing](http://notmatthancock.github.io/2017/10/09/region-growing-wrapping-c.html)

[Region Growing Segmentation](http://simpleitk-prototype.readthedocs.io/en/latest/user_guide/segmentation/plot_region_growing.html)