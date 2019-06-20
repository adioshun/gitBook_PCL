# Matplot를 이용한 시각화 


- [필수] 코드 수정후 KITTI외 시각화 기능으로 추가 https://github.com/navoshta/KITTI-Dataset/blob/master/kitti-dataset.ipynb

```python

#%matplotlib inline

X = data

# Plot result
import matplotlib.pyplot as plt

# Black removed and is used for noise instead.
unique_labels = set(labels)
colors = [plt.cm.Spectral(each)
          for each in np.linspace(0, 1, len(unique_labels))]
for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = [0, 0, 0, 1]

    class_member_mask = (labels == k)

    xy = X[class_member_mask & core_samples_mask]
    plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
             markeredgecolor='k', markersize=1)# 14

    xy = X[class_member_mask & ~core_samples_mask]
    plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
             markeredgecolor='k', markersize=1)

plt.title('Estimated number of clusters: %d' % n_clusters_)
plt.show()


```





### 1.6 Matplotlib이용



- 설치가 쉽지만, 느리고 3D를 충분히 표현하지 못한다. [[KITTI Data Demo]](https://github.com/hunjung-lim/awesome-vehicle-datasets/blob/master/vehicle/kitti/KITTI%2BDataset%2BExploration.ipynb)

https://github.com/navoshta/KITTI-Dataset/blob/master/kitti-dataset.ipynb

```python
from mpl_toolkits.mplot3d import Axes3D

f2 = plt.figure()
ax2 = f2.add_subplot(111, projection='3d')
# Plot every 100th point so things don't get too bogged down
velo_range = range(0, third_velo.shape[0], 100)
ax2.scatter(third_velo[velo_range, 0],
            third_velo[velo_range, 1],
            third_velo[velo_range, 2],
            c=third_velo[velo_range, 3],
            cmap='gray')
ax2.set_title('Third Velodyne scan (subsampled)')

plt.show()
```




In order to prevent matplotlib from crashing your computer, it is recomended to only view a subset of the point cloud data. 

For instance, if you are visualizing LIDAR data, then you may only want to view one in every 25-100 points. Below is some sample code to get you started.

```python
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

skip = 100   # Skip every n points

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
point_range = range(0, points.shape[0], skip) # skip points to prevent crash
ax.scatter(points[point_range, 0],   # x
           points[point_range, 1],   # y
           points[point_range, 2],   # z
           c=points[point_range, 2], # height data for color
           cmap='spectral',
           marker="x")
ax.axis('scaled')  # {equal, scaled}
plt.show()
```







---

### 1.5 3D Viewer

> [How to visualize a range image](http://pointclouds.org/documentation/tutorials/range_image_visualization.php#range-image-visualization)


Code Download : [range_image_visualization.cpp](https://gist.githubusercontent.com/adioshun/1ae4197af17f79f01f1ec3ec7c8f4bcb/raw/6ec7e03db63f0477688a66ae580c14795dec0803/range_image_visualization.cpp), [CMakeLists.txt](https://gist.githubusercontent.com/adioshun/1ae4197af17f79f01f1ec3ec7c8f4bcb/raw/6ec7e03db63f0477688a66ae580c14795dec0803/CMakeLists.txt)



```
mkdir range_image_visualization; cd range_image_visualization
vi range_image_visualization.cpp
vi CMakeLists.txt
mkdir build;cd build
cmake ..
make

# Test
wget https://raw.github.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd
./range_image_visualization table_scene_lms400.pcd 

```

> Simple Version : [Cloud Viewer](http://pointclouds.org/documentation/tutorials/cloud_viewer.php#cloud-viewer)




















---


