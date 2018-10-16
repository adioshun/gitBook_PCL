# [Filtering a PointCloud using a PassThrough filter](http://pointclouds.org/documentation/tutorials/passthrough.php#passthrough)

- In this tutorial we will learn how to perform a simple filtering along a specified dimension – that is, cut off values that are either inside or outside a given user range.

- RoI필터와 같이 x,y,z좌료를 정해 주고 crop하는 방식, 정교한 부분을 제거하지는 못한다. 


```python 
def do_passthrough(pcl_data,filter_axis,axis_min,axis_max):
    '''
    Create a PassThrough  object and assigns a filter axis and range.
    :param pcl_data: point could data subscriber
    :param filter_axis: filter axis
    :param axis_min: Minimum  axis to the passthrough filter object
    :param axis_max: Maximum axis to the passthrough filter object
    :return: passthrough on point cloud
    '''
    passthrough = pcl_data.make_passthrough_filter()
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    return passthrough.filter()
```

사용 

```python 
cloud = input_pcl_xyzrgb

filter_axis = 'x'
axis_min = 1.0
axis_max = 20.0
cloud = filter.do_passthrough(cloud, filter_axis, axis_min, axis_max)

filter_axis = 'y'
axis_min = -7.0
axis_max = 5.5
cloud = filter.do_passthrough(cloud, filter_axis, axis_min, axis_max)

filter_axis = 'z'
axis_min = -1.2
axis_max = 10.0
cloud = filter.do_passthrough(cloud, filter_axis, axis_min, axis_max)
```
