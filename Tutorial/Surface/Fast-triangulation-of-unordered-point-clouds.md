# [Fast triangulation of unordered point clouds](http://pointclouds.org/documentation/tutorials/greedy_projection.php#greedy-triangulation)

> 점들을 연결하여 벽면등 모양 구체화, 시각화에 좋을듯 


Triangulation is a a way of estimating the surface captured by a point cloud, by connecting points with each other, ending up with a continous polygon mesh (three-sided polygons, that is, triangles). 

완성된 surface mesh는 다른 시각화 툴(VTK)을 이용하여 볼수 있음













---

# [PCL/OpenNI tutorial 2: Cloud processing (Surface smoothing)](http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing\_\(Triangulation\))





Triangulation
Triangulation is a a way of estimating the surface captured by a point cloud, by connecting points with each other, ending up with a continous polygon mesh (three-sided polygons, that is, triangles). After you retrieve this surface mesh, you can for example export it to a format that most 3D modelling tools will understand, like VTK (Visualization Toolkit), which can be opened in Blender or 3ds Max. PCL can also convert from VTK to PLY or OBJ.

The following code uses PCL's implementation of a greedy triangulation algorithm that works with local 2D projections. For every point, it looks "down" along the normal, and connects neighboring points. For more information, specially regarding the parameters, check the API or the original PCL tutorial:
