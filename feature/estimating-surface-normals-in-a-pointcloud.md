# [Estimating Surface Normals in a PointCloud](http://pointclouds.org/documentation/tutorials/normal_estimation.php#normal-estimation)

> Normal = 수직 방향, Surface Normal = 표면에 수직 방향
> 2차원 = Normal Line / 3차원 Surface normal(OR simply **normal**)

주요한 특징으로 조명에 의한 그림자 생성등 시각 효과증에서 많이 사용된다. `Surface normals are important properties of a geometric surface, and are heavily used in many areas such as computer graphics applications, to apply the correct light sources that generate shadings and other visual effects.`

Given a geometric surface, it’s usually trivial to infer the direction of the normal at a certain point on the surface as the vector perpendicular to the surface in that point. 

However, since the point cloud datasets that we acquire represent a set of point samples on the real surface, there are two possibilities:
- obtain the underlying surface from the acquired point cloud dataset, using surface meshing techniques, and then compute the surface normals from the mesh;
- use approximations to infer the surface normals from the point cloud dataset directly.

To gain an understanding of an object's shape, we will calculate the distribution of an object's surface normals. As a surface normal is perpendicular to the surface of a shape, the distribution of these normals will vary dramatically between a flat or round objects.



---
![](https://github.com/fouliex/RoboticPerception/raw/master/pr2_robot/misc/SurfaceNormal.JPG)

## [참고][Surface Normals](https://github.com/fouliex/RoboticPerception#surface-normals)


As one can see color can be used for object recognition, but another powerful way to find what we are looking for in our data is by searching for particular shapes vector images. 

We can search for a given template shape or simply take the gradient of the image and explore the distribution of lights and edges that emerge. 

Since we are working with 3D point clouds, we have an extra dimension of shape information to investigate. 

In our point cloud, we have partial information on the 3D shapes of the object which is to say we have the view of the object surface from just one perspective. What we would like to do is to compare the distribution of points with a ground truth or reference distribution in order to decide whether or not we have found what we are looking for. 

To do this, we need a metric that captures shape and one such metric is the distribution of surface normals.

The normal of a surface is just a unit vector that is perpendicular to that surface. 

The normals at different points, along with the changing surface, will point in different direction and the distribution of surface normals taken a whole can be used to describe the shape of the objects. 

---

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


```python 
import numpy as np
from open3d import *

pcd = read_point_cloud("/workspace/tmp/normal/fragment.pcd")
# Instead of "setRadiusSearch()", you can make use of "setKSearch()", which takes an integer, K.
KD = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30)
estimate_normals(pcd, search_param = KD)
# setViewPoint() # 옵

print(pcd.has_normals())
pcd.normals[0]
pcd.points[0]

normals = np.asarray(pcd.normals)
normals[0]
```


### Normal Hist

```python 
  
def compute_normal_histograms(normal_cloud, nbins=32, nrange=(-1,1)):
    '''
    Computes and bins the point-cloud data using the objects distribution of surface normals.
    :param: normal_cloud, point cloud containing the filtered clusters.
    :param: nbins,number of bins that data will be pooled into.
    :param: nrange, value range of the data to be pooled.
    :return: the normalised histogram of surface normals
    '''
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []
    
    for I in range(0,normal_cloud.size):
        norm_x_vals.append(normal_cloud[I][0])
        norm_y_vals.append(normal_cloud[I][1])
        norm_z_vals.append(normal_cloud[I][2])
    
    """
    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])
    """
    

    # Compute histograms of normal values (just like with color)
    norm_x_hist = np.histogram(norm_x_vals, bins=nbins, range=nrange)
    norm_y_hist = np.histogram(norm_y_vals, bins=nbins, range=nrange)
    norm_z_hist = np.histogram(norm_z_vals, bins=nbins, range=nrange) 

    # Concatenate and normalize the histograms
    hist_features = np.concatenate((norm_x_hist[0], norm_y_hist[0], norm_z_hist[0])).astype(np.float64)
    normed_features = hist_features / np.sum(hist_features)

    return normed_features

## 시각화 
?%matplotlib
import matplotlib.pyplot as plt
plt.hist(normed_features, nbins)
plt.xlabel('Weight (kg)', fontsize = 14)
plt.xticks(fontsize = 14)
plt.yticks(fontsize = 14)

```

---


# Surface curvature estimates

<iframe width="560" height="315" src="https://www.youtube.com/embed/d3z35hpwPZQ" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

<a href="http://www.youtube.com/watch?feature=player_embedded&v=d3z35hpwPZQ
" target="_blank"><img src="http://img.youtube.com/vi/d3z35hpwPZQ/0.jpg" 
alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>
- Red = low curvature, 
- green-blue = curvature.

Normals are also important because they give us information about the curvature of the surface at some point

--- 

[How to visualize surface normals as Marker::Arrow for each point in rviz?](https://answers.ros.org/question/9095/how-to-visualize-surface-normals-as-markerarrow-for-each-point-in-rviz/): 부하때문에 어려움 

[pcl_normal_visualization.cpp](http://docs.ros.org/groovy/api/pcl_cloud_tools/html/pcl__normal__visualization_8cpp_source.html)