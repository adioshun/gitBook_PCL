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

## python code 

```python 
import os
import sys
import pcl
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import glob
import pcl
import matplotlib.pyplot as plt
import numpy as np

def get_normals(cloud_path):
    """
    The actual *compute* call from the NormalEstimation class does nothing internally 	but:
    for each point p in cloud P
    1. get the nearest neighbors of p
    2. compute the surface normal n of p
    3. check if n is consistently oriented towards the viewpoint and flip otherwise

    # normals:  pcl._pcl.PointCloud_Normal,size: 26475
    # cloud:    pcl._pcl.PointCloud   
    """
    cloud = pcl.load(cloud_path)
    feature = cloud.make_NormalEstimation()
    #feature.set_RadiusSearch(0.1) #Use all neighbors in a sphere of radius 3cm
    feature.set_KSearch(3)
    normals = feature.compute()
    #for i in range(0, normals.size):
        #print ('normal_x: '  + str(normals[i][0]) + ', normal_y : ' + str(normals[i][1])  + ', normal_z : ' + str(normals[i][2]))
    return normals

normals=get_normals('./../sample_lcas.pcd')


def nan_process(array):
     return np.nan_to_num(array)
    
def compute_normal_histograms(normal_cloud, nbins=16):
    """
    Computes and bins the point-cloud data using the objects distribution of surface normals.
    :param: normal_cloud, point cloud containing the filtered clusters.
    :param: nbins,number of bins that data will be pooled into.
    :param: nrange, value range of the data to be pooled.
    :return: the normalised histogram of surface normals
    """
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    #normal_cloud = normal_cloud[~np.isnan(normal_cloud)]
    for I in range(0,normal_cloud.size):
        norm_x_vals.append(normal_cloud[I][0])
        norm_y_vals.append(normal_cloud[I][1])
        norm_z_vals.append(normal_cloud[I][2])

    '''
    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])
    '''

    #print('norm_x_vals:{}'.format(norm_x_vals))

    norm_x_vals=nan_process(np.array(norm_x_vals))
    norm_y_vals=nan_process(np.array(norm_y_vals))
    norm_z_vals=nan_process(np.array(norm_z_vals))
    '''
    print('norm_x_vals:{}'.format(~np.isnan(norm_x_vals)))
    print('norm_y_vals:{}'.format(~np.isnan(norm_y_vals)))
    print('norm_z_vals:{}'.format(~np.isnan(norm_z_vals)))
    ''' 

    # Compute histograms of normal values (just like with color)
    #return hist(the number of sample in each bin or pro densi) and bin edge
    norm_x_hist = np.histogram(norm_x_vals, bins=nbins) # 16bins
    norm_y_hist = np.histogram(norm_y_vals, bins=nbins)
    norm_z_hist = np.histogram(norm_z_vals, bins=nbins) 

    # Concatenate and normalize the histograms
    hist_features = np.concatenate((norm_x_hist[0], norm_y_hist[0], norm_z_hist[0])). astype(np.float64)  
    #print('hist_features:{}'.format(hist_features))
    #print('hist_features sum :{}'.format(np.sum(hist_features)))
    #plt.hist(hist_features,16)
    #plt.show()
    normed_features = hist_features / np.sum(hist_features) 
    return normed_features
    
normed_features=compute_normal_histograms(normals)

## visualize 
def plot_normals(normed_features,nbins):	
	plt.hist(normed_features, nbins)
	plt.xlabel('Weight (kg)', fontsize = 14)
	plt.xticks(fontsize = 14)
	plt.yticks(fontsize = 14)
        #plt.show()

plot_normals(normed_features,32)


```




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


