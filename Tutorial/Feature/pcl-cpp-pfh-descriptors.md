# [Point Feature Histograms (PFH) descriptors](http://pointclouds.org/documentation/tutorials/pfh_estimation.php#pfh-estimation)


Noraml/Curvature정보들은 기본적인 geometry라 빠르고 계산하기 쉽지만 자세한 특징을 기술 하긴 어렵다. 
- `As point feature representations go, surface normals and curvature estimates are somewhat basic in their representations of the geometry around a specific point. Though extremely fast and easy to compute, they cannot capture too much detail, as they approximate the geometry of a point’s k-neighborhood with only a few values. `
- As a direct consequence, most scenes will contain many points with the same or very similar feature values, thus reducing their informative characteristics.

본 챕터에서 살펴볼 PFH (Point Feature Histograms)를 이해하기 위해서는 Estimating Surface Normals에 대하여 알아야 한다. 

PFH는 rely on both **xyz 3D data** as well as **surface normals**.


