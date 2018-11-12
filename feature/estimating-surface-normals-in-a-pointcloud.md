# [Estimating Surface Normals in a PointCloud](http://pointclouds.org/documentation/tutorials/normal_estimation.php#normal-estimation)

주요한 특징으로 조명에 의한 그림자 생성등 시각 효과증에서 많이 사용된다. `Surface normals are important properties of a geometric surface, and are heavily used in many areas such as computer graphics applications, to apply the correct light sources that generate shadings and other visual effects.`

Given a geometric surface, it’s usually trivial to infer the direction of the normal at a certain point on the surface as the vector perpendicular to the surface in that point. 

However, since the point cloud datasets that we acquire represent a set of point samples on the real surface, there are two possibilities:
- obtain the underlying surface from the acquired point cloud dataset, using surface meshing techniques, and then compute the surface normals from the mesh;
- use approximations to infer the surface normals from the point cloud dataset directly.


![](https://github.com/fouliex/RoboticPerception/raw/master/pr2_robot/misc/SurfaceNormal.JPG)

## [참고][Surface Normals](https://github.com/fouliex/RoboticPerception#surface-normals)


As one can see color can be used for object recognition, but another powerful way to find what we are looking for in our data is by searching for particular shapes vector images. 

We can search for a given template shape or simply take the gradient of the image and explore the distribution of lights and edges that emerge. 

Since we are working with 3D point clouds, we have an extra dimension of shape information to investigate. 

In our point cloud, we have partial information on the 3D shapes of the object which is to say we have the view of the object surface from just one perspective. What we would like to do is to compare the distribution of points with a ground truth or reference distribution in order to decide whether or not we have found what we are looking for. 

To do this, we need a metric that captures shape and one such metric is the distribution of surface normals.

The normal of a surface is just a unit vector that is perpendicular to that surface. 

The normals at different points, along with the changing surface, will point in different direction and the distribution of surface normals taken a whole can be used to describe the shape of the objects. 



```python 
import pcl
cloud = pcl.load("/workspace/_pcd/1530509312.094227000.pcd")

feature = cloud.make_NormalEstimation()
feature.set_RadiusSearch(0.03) #Use all neighbors in a sphere of radius 3cm
normals = feature.compute()
  """
  The actual *compute* call from the NormalEstimation class does nothing internally but:
  for each point p in cloud P
    1. get the nearest neighbors of p
    2. compute the surface normal n of p
    3. check if n is consistently oriented towards the viewpoint and flip otherwise
  """

print(cloud)
print(normals)

for i in range(0, normals.size):
    print ('normal_x: '  + str(normals[i][0]) + ', normal_y : ' + str(normals[i][1])  + ', normal_z : ' + str(normals[i][2]))
```