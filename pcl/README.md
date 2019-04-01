# PCL



## 2. Tools

###### Official Tools of PCL


- pcl_viewer: a quick way for visualizing PCD (Point Cloud Data) files

- pcd2ply: converts PCD (Point Cloud Data) files to the PLY format

- octree_viewer: allows the visualization of octrees


###### PCl Tools

`sudo apt-get install pcl-tools` [[github]](https://github.com/ryanfb/pcl-tools)
- pcd2ply - convert PCD files to PLY files
- ply2pcd - convert PLY files to PCD files
- statistical_removal - statistical outliers removal
- pcd_viewer - PCL PCD viewer with added 
- pcdnormal2ply - convert PCD files with normals to PLY files
- normal_estimation_omp - estimate normals in a PCD using PCL's OpenMP implementation


###### pcl_helper.py

[github](https://github.com/udacity/RoboND-Perception-Exercises#documentation-for-pcl_helperpy), [Code](https://github.com/udacity/RoboND-Perception-Exercises/tree/master/Exercise-2/sensor_stick/scripts)



`random_color_gen()` : Generates a random set of r,g,b values
- Return: a 3-tuple with r,g,b values (range 0-255)

`ros_to_pcl(sensor_msgs/PointCloud2)` : Converts sensor_msgs/PointCloud2 to XYZRGB Point Cloud
- Return: pcl.PointCloud_PointXYZRGB

`pcl_to_ros(pcl.PointCloud_PointXYZRGB)`: Converts XYZRGB Point Cloud to sensor_msgs/PointCloud2
- Return: sensor_msgs/PointCloud2

`XYZRGB_to_XYZ(XYZRGB_cloud)`: Converts XYZRGB Point Cloud to XYZ Point CLoud
- Return: pcl.PointCloud

`XYZ_to_XYZRGB(XYZ_cloud, color)`:Takes a 3-tuple as color and adds it to XYZ Point Cloud
- Return: pcl.PointCloud_PointXYZRGB

`rgb_to_float(color)`:Converts 3-tuple color to a single float32
- Return: rgb packed as a single float32

`get_color_list(cluster_count)` : Creates a list of 3-tuple (rgb) with length of the list = cluster_count
- Return: get_color_list.color_list








## 3. Projects

- [Cluster Recognition and 6DOF Pose Estimation using VFH descriptors](http://pointclouds.org/documentation/tutorials/vfh_recognition.php#vfh-recognition)

- [Detecting people and their poses using PointCloud Library](http://pointclouds.org/documentation/tutorials/gpu_people.php#gpu-people): RGB-D??

- [RoboND-Perception-Exercises](https://github.com/udacity/RoboND-Perception-Exercises): Udacity Nanodegree program, python

---
## PCD 읽기




```python
import pcl
import numpy as np
p = pcl.PointCloud(np.array([[1, 2, 3], [3, 4, 5]], dtype=np.float32))
seg = p.make_segmenter()
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
indices, model = seg.segment()
```


```python
import pcl
p = pcl.PointCloud()
p.from_file("table_scene_lms400.pcd")
fil = p.make_statistical_outlier_filter()
fil.set_mean_k (50)
fil.set_std_dev_mul_thresh (1.0)
fil.filter().to_file("inliers.pcd")
```


> [How to extract depth information from the 3D point cloud data?](https://stackoverflow.com/questions/42430479/how-to-extract-depth-information-from-the-3d-point-cloud-data) : 포인트클라우드에서 depth 계산법 

---



