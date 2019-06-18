# [Fast triangulation of unordered point clouds](http://pointclouds.org/documentation/tutorials/greedy_projection.php#greedy-triangulation)

> 점들을 연결하여 벽면등 모양 구체화, 시각화에 좋을듯 


Triangulation is a a way of estimating the surface captured by a point cloud, by connecting points with each other, ending up with a continous polygon mesh (three-sided polygons, that is, triangles). 

완성된 surface mesh는 다른 시각화 툴(VTK)을 이용하여 볼수 있음