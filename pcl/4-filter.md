# Filter

## Outlier Filter

Statistical Outlier Filtering is use to remove outlieres using
- number of neighboring points of 10
- standard deviation threshold of 0.001

```python
def do_statistical_outlier_filtering(pcl_data,mean_k,tresh):
    '''
    :param pcl_data: point could data subscriber
    :param mean_k: number of neighboring points to analyze for any given point
    :param tresh: Any point with a mean distance larger than global will be considered outlier
    :return: Statistical outlier filtered point cloud data
    '''
    outlier_filter = pcl_data.make_statistical_outlier_filter()
    
    ## Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(mean_k) 
    
    # Any point with a mean distance larger than global (mean distance+x*std_dev)
    # will be considered outlier
    outlier_filter.set_std_dev_mul_thresh(tresh)
    return outlier_filter.filter()

# Convert ROS msg to PCL data
cloud = ros_to_pcl(pcl_msg)

# Statistical Outlier Filtering
cloud = do_statistical_outlier_filtering(cloud,10,0.001)
```

## ROI Filter

```python
# PassThrough filter
# Create a PassThrough filter object.
passthrough = cloud_filtered.make_passthrough_filter()

# Assign axis and range to the passthrough filter object.
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.6
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)

# Save pcd for tabletop objects
cloud_filtered = passthrough.filter()
filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered, filename)

```



```python
def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    '''
    Create a PassThrough  object and assigns a filter axis and range.
    :param pcl_data: point could data subscriber
    :param filter_axis: filter axis, x is in the direction of the camera, y is the left & right of the camera, z up & down
    :param axis_min: Minimum  axis to the passthrough filter object
    :param axis_max: Maximum axis to the passthrough filter object
    :return: passthrough on point cloud
    '''
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()


# Convert ROS msg to PCL data
cloud = ros_to_pcl(pcl_msg)

# PassThrough Filter
filter_axis ='z'
axis_min = 0.44
axis_max =0.85
cloud = do_passthrough(cloud,filter_axis,axis_min,axis_max)

filter_axis = 'x'
axis_min = 0.33
axis_max = 1.0
cloud = do_passthrough(cloud, filter_axis, axis_min, axis_max)

```

단위는 무었인가? 크기는? The indices_xz array indexes all points of cloud_in that have x between 0.0 and 1000.0 and z larger than 10.0 or smaller than -10.0

## 4. RANSAC


```python

def do_ransac_plane_segmentation(pcl_data,pcl_sac_model_plane,pcl_sac_ransac,max_distance):
    '''
    Create the segmentation object
    :param pcl_data: point could data subscriber
    :param pcl_sac_model_plane: use to determine plane models
    :param pcl_sac_ransac: RANdom SAmple Consensus
    :param max_distance: Max distance for apoint to be considered fitting the model
    :return: segmentation object
    '''
    seg = pcl_data.make_segmenter()
    seg.set_model_type(pcl_sac_model_plane)
    seg.set_method_type(pcl_sac_ransac)
    seg.set_distance_threshold(max_distance)
    return seg


def  extract_cloud_objects_and_cloud_table(pcl_data,ransac_segmentation):
    '''
    :param pcl_data:
    :param ransac_segmentation:
    :return: cloud table and cloud object
    '''
    inliers, coefficients = ransac_segmentation.segment()
    cloud_table = pcl_data.extract(inliers, negative=False)
    cloud_objects = pcl_data.extract(inliers, negative=True)
    return cloud_table,cloud_objects


# Convert ROS msg to PCL data
cloud = ros_to_pcl(pcl_msg)
# RANSAC Plane Segmentation
ransac_segmentation = do_ransac_plane_segmentation(cloud,pcl.SACMODEL_PLANE,pcl.SAC_RANSAC,0.01)

# Extract inliers and outliers
cloud_table,cloud_objects= extract_cloud_objects_and_cloud_table(cloud,ransac_segmentation )
```

SACMODEL

```
   48     SACMODEL_PLANE,
   49     SACMODEL_LINE,
   50     SACMODEL_CIRCLE2D,
   51     SACMODEL_CIRCLE3D,
   52     SACMODEL_SPHERE,
   53     SACMODEL_CYLINDER,
   54     SACMODEL_CONE,
   55     SACMODEL_TORUS,
   56     SACMODEL_PARALLEL_LINE,
   57     SACMODEL_PERPENDICULAR_PLANE,
   58     SACMODEL_PARALLEL_LINES,
   59     SACMODEL_NORMAL_PLANE,
   60     SACMODEL_NORMAL_SPHERE,
   61     SACMODEL_REGISTRATION,
   62     SACMODEL_REGISTRATION_2D,
   63     SACMODEL_PARALLEL_PLANE,
   64     SACMODEL_NORMAL_PARALLEL_PLANE,
   65     SACMODEL_STICK
  ```

