# normal estimation(법선추정)

## 1. 개요 

### 1.1 정의  

법선(Normals)
- 평면에 있는 직선의 한 점을 지나면서 이 직선에 수직인 직선을 법선이라고 한다. 
- 평면곡선의 경우 그 곡선 위의 한 점에서 그은 접선에 수직인 직선을 원래 곡선의 법선이라고 한다.
- 삼차원 공간에서는 공간에 있는 평면 위의 한 점을 지나면서 그 평면에 수직인 직선을 법선이라고 한다.

### 1.2 분류 

- 꼭지점 법선(Vertex Normals)
- 면 법선(Face Normals) 

> Normal의 어원은 라틴어 Norma 로 '목수의 직각자' 라는 뜻이라고 합니다.


![image](https://user-images.githubusercontent.com/17797922/41693140-e87b4298-753e-11e8-8d66-0c1ca989e531.png)


법선 백터 추정(Normal Estimation) : 샘플링 된 값들로부터 방향 정보를 복원해 내는 작업 
- 한점의 깊이 점보만으로는 법선 벡터를 구할수 없다. 그러나 벡터를 구하려고 하는 대상 점의 이웃한 점들이 가지고 있는 값들을 이용하면 샘플링하기 전에 그 점을 포함하고 있던 면의 법선 벡터를 근사적으로 추정 할수 있다. 
- 대상점을 중심으로 한 국소적인 정보로 부터 구해 낸 법선 벡터를 추정 법선 벡터(estimated normal)라 한다. 


### 1.3 추정 방법 


법선 백터 추정 방법 
- 깊이 경사도 방법 (The depth gradient method) : 
- N-neighbor depth gradient method 
- Context sensitive method 

> 표면 특성 감응식 법선 벡터 추정 방법(Surface-Characteristic-Sensitive Normal Estimation Method), 신병석, 1995


### 1.4 활용 

normal vector를 제대로 찾아내지 못하면 registration의 실패 요인이 될 수 있다.

The exact computation of vertex normal vectors is essential for user to apply a variety of geometric operations to the mesh and get more realistic rendering results. `보다 다양한 형태로의 변형이나 현실감 있는 렌더링을 얻기 위해서는 정점에서의 올바른 법선벡터(vertex normal) 계산이 필수적이다. `



Normal manipulation(조정) is possible with "n", "+", "-" on the screen

### 1.5 동작 원리 

전처리 과정에서 가장 중요한 법선 추정은 주변의 점들을 이용한다. 

한 점 A에서 법선 추정은 그림 2와 같이 한점을 기준으로 일정거리 혹은 일정개수 만큼의 점들을 통해 Normal을 추정하게 된다. 

이때 KD-Tree 자료구조를 이용하여 주변 점들로부터, 공분산 행렬(Covarience Matrix)을 구성하여 고유 벡터(Eigen Vector)를 계산한다




`estimate_normals` computes normal for every point. 

- The function finds adjacent points and calculate the principal axis of the adjacent points using covariance analysis.

- The function takes an instance of `KDTreeSearchParamHybrid` class as an argument. 

- The two key arguments : search radius and maximum nearest neighbor
        - radius = 0.1 
        - max_nn = 30 
        - It has 10cm of search radius, and only considers up to 30 neighbors to save computation time.

Recompute the normal of the downsampled point cloud

`estimate_normals` Calculates the normals of all points. 
- This function finds neighboring points using covariance analysis and computes the principal axes of neighboring points.


- The covariance analysis algorithm generates two mutually opposite vectors as normal candidates. 

- If you do not know the global structure of the geometry, neither is correct. 

- This is a problem known as the normal direction problem. 

- Open3D attempts to orient the normals so that they are aligned with the original normals when normals are present. 

- Otherwise, Open3D makes random guesses. 

- If the direction is important, you need to call an orientation function such as  orient_normals_to_align_with_directionand orient_normals_towards_camera_location.

- draw_geometries To visualize the point cloud nand press to display the normal of the point. -You +can control the length of the normal by using keys and keys.


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

# [Normal estimation](http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing_\(basic\)#Normal_estimation)

노멀은 평면에 직각인 unit vector이다. `As you may remember from geometry class, the normal of a plane is an unit vector that is perpendicular to it.`

The normal of a surface at a point is defined as the vector that is perpendicular to the plane that is tangent to the surface at the point. 

Surface normals can be calculated for the points of a cloud, too. 

이를 Feature로 사용할수는 있으나 설명력이 크진 않다. `It is considered a feature, albeit not a very discriminative one.`

계산을 위해 주변 점군이 필요 하다. `I will not go into detail with the math of the estimation method, but you just have to know that is uses the nearest neighbors (the points that are closest to the one we are calculating the normal for) to find out the tangent plane and the normal vector. `

You can customize the method with the search radius  and the viewpoint.
- search radius : think about a sphere of that radius, centered in the point; all neighboring points that lie within will be used for the computation
- viewpoint by default, the output normals will be directionless; by supposing that all vectors must point towards the camera - because otherwise they would belong to surfaces that are not visible from the sensor - they can all be re-oriented accordingly

노멀이 중요한 다른 이유는 **평면의 곡선(curvature)**을 알수 있다는 것이다. `Normals are also important because they give us information about the curvature of the surface at some point, which can be used to our advantage. `



---
## Matlab code `normals = pcnormals(ptCloud,k)`

input Arguments
- ptCloud — Object for storing point cloud
        - Object for storing point cloud, returned as a pointCloud object.
- k — Number of points used for local plane fitting
        - integer greater than or equal to 3
        - Number of points used for local plane fitting, specified as an integer greater than or equal to 3. Increasing this value improves accuracy but slows down computation time.

Output Arguments
- normals — Normals used to fit a local plane (M-by-3 | M-by-N-by-3)
        - Normals used to fit a local plane, returned as an M-by-3 or an M-by-N-by-3 vector. The normal vectors are computed locally using six neighboring points. The direction of each normal vector can be set based on how you acquired the points. The Estimate Normals of Point Cloud example, shows how to set the direction when the normal vectors are pointing towards the sensor.
        
        
        ```python
print("Recompute the normal of the downsampled point cloud")
estimate_normals(downpcd, search_param = KDTreeSearchParamHybrid(
radius = 0.1, max_nn = 30))
draw_geometries([downpcd])
print("")
```

---


# Surface curvature estimates


<a href="http://www.youtube.com/watch?feature=player_embedded&v=d3z35hpwPZQ" target="_blank">
<img src="http://img.youtube.com/vi/d3z35hpwPZQ/0.jpg" alt="IMAGE ALT TEXT HERE" width="240" height="180" border="10" /></a>
- Red = low curvature, 
- green-blue = curvature.

Normals are also important because they give us information about the curvature of the surface at some point

--- 

[How to visualize surface normals as Marker::Arrow for each point in rviz?](https://answers.ros.org/question/9095/how-to-visualize-surface-normals-as-markerarrow-for-each-point-in-rviz/): 부하때문에 어려움 

[pcl_normal_visualization.cpp](http://docs.ros.org/groovy/api/pcl_cloud_tools/html/pcl__normal__visualization_8cpp_source.html)


