# Key Point / Feature

http://www.pointclouds.org/assets/uploads/cglibs13_features.pdf
https://github.com/PointCloudLibrary/pcl/wiki/Overview-and-Comparison-of-Features

---

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


