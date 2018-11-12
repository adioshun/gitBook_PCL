# k3d를 이용한 시각화 

```python 

# 시각화 부분

pa = cloud.to_array()

plot = k3d.plot()
print(pa.shape[0])
points_number = pa.shape[0]
colors = np.random.randint(0, 0xFFFFFF, points_number)

points = k3d.points(pa, colors, point_size=0.01, shader='3d')
plot += points
plot.camera_auto_fit = False
plot.display()

```


# Matplot를 이용한 시각화 


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

---
# Visualization 

## 1. PCD

[샘플 PCD Download](https://github.com/PointCloudLibrary/data/blob/master/tutorials/table_scene_lms400.pcd), [PCD File Format](http://www.jeffdelmerico.com/wp-content/uploads/2014/03/pcl_tutorial.pdf): slide 12

### 1.1 PCL viewer

- 설치 : sudo apt install pcl-tools

`/usr/bin/pcl_viewer/{  }.pcd`

> [PCL Visualization overview](http://pointclouds.org/documentation/overview/visualization.php), [Youtube](https://www.youtube.com/watch?v=BZBQXcBvHW0)



### 1.2 Jupyter

[point cloud visualization with jupyter/pcl-python/and potree](https://www.youtube.com/watch?v=s2IvpYvB7Ew)


### 1.3 Mayavi 이용 

[Mayavi 홈페이지](http://docs.enthought.com/mayavi/mayavi/)

[Plot with Mayavi in Jupyter notebook on Docker for Mac](https://taku-y.github.io/mac-docker-jupyter-mayavi.html)


[공식 설치 가이드 /w Jupyter](http://docs.enthought.com/mayavi/mayavi/installation.html#installing-with-pip)


```
# ubuntu 16.04 tested (??)
$ pip install numpy mayavi

$ pip install PyQt5

$ jupyter nbextension install --py mayavi --user
$ jupyter nbextension enable --py mayavi --user
```

###### Conda 설치  
    - python2 : `conda install -c anaconda mayavi`
    - python3 : `conda install -c clinicalgraphics vtk=7.1.0; pip install mayavi`


> ImportError: Could not import backend for traits 
> - conda install -c conda-forge pyside=1.2.4 
> - {OR} conda install pyqt=4

###### Pip 설치 


```bash
sudo apt-get install vtk6 tcl-vtk python-vtk
python -c "import vtk"
# ImportError: libGLU.so.1: cannot open shared object file: No such file or director
## -> cp -r /usr/lib/python2.7/dist-packages/vtk /opt/anaconda3/envs/python2_gpu/lib/python2.7/site-packages/
pip install mayavi
import mayavi.mlab as mlab


# echo "INSTALLING GUI BACKEND FOR MAYAVI"
# pip install mayavi[TraitsBackendQt]
sudo apt-get install build-essential git cmake libqt4-dev libphonon-dev python2.7-dev libxml2-dev libxslt1-dev qtmobility-dev libqtwebkit-dev
pip install pyside

sudo apt-get install python-pyqt4 for pyqt4
```

Test code
```python
from mayavi import mlab
mlab.init_notebook()
s = mlab.test_plot3d()
s
```

실행 코드 

```python
# ==============================================================================
#                                                                     VIZ_MAYAVI
# Input : kitti Raw Dataset 
# ==============================================================================
def viz_mayavi(points, vals="distance"):
    x = points[:, 0]  # x position of point
    y = points[:, 1]  # y position of point
    z = points[:, 2]  # z position of point
    # r = lidar[:, 3]  # reflectance value of point
    d = np.sqrt(x ** 2 + y ** 2)  # Map Distance from sensor

    # Plot using mayavi -Much faster and smoother than matplotlib
    import mayavi.mlab

    if vals == "height":
        col = z
    else:
        col = d

    fig = mayavi.mlab.figure(bgcolor=(0, 0, 0), size=(640, 360))
    mayavi.mlab.points3d(x, y, z,
                         col,          # Values used for Color
                         mode="point",
                         colormap='spectral', # 'bone', 'copper', 'gnuplot'
                         # color=(0, 1, 0),   # Used a fixed (r,g,b) instead
                         figure=fig,
                         )
    mayavi.mlab.show()
```


### 1.4 Paraview 

[ParaView/PCL Plugin](https://www.paraview.org/Wiki/ParaView/PCL_Plugin)
- apt-get install paraview


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

## plot.ly





















---



https://www.slicer.org/ : medical images


http://www.sci.utah.edu/software/imagevis3d.html

http://www.danielgm.net/cc/