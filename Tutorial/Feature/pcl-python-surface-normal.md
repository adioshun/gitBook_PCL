# [Estimating Surface Normals in a PointCloud-PCL-Python](https://github.com/strawlab/python-pcl/blob/master/tests/test_features.py)

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
    The actual *compute* call from the NormalEstimation class does nothing internally but:
    for each point p in cloud P
    1. get the nearest neighbors of p
    2. compute the surface normal n of p
    3. check if n is consistently oriented towards the viewpoint and flip otherwise
    
    # normals: pcl._pcl.PointCloud_Normal,size: 26475
    # cloud: pcl._pcl.PointCloud
    """
    cloud = pcl.load(cloud_path)
    feature = cloud.make_NormalEstimation()
    #feature.set_RadiusSearch(0.1) #Use all neighbors in a sphere of radius 3cm
    feature.set_KSearch(3)
    normals = feature.compute()
    #for i in range(0, normals.size):
    #print ('normal_x: ' + str(normals[i][0]) + ', normal_y : ' + str(normals[i][1]) + ', normal_z : ' + str(normals[i][2]))
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

[Normal Estimation Using Integral Images-PCL-Python](https://github.com/strawlab/python-pcl/blob/master/examples/official/Features/NormalEstimationUsingIntegralImages.py) : Normal estimation on organized clouds(RGB-D)
