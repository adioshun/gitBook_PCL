![](https://i.imgur.com/giosTpW.png)

```
Figure 3.2: Example of a voxel filter.
Black dots are the centroids of each voxel.
Left example shows a big voxel size, where all data points are in the same voxel.
Right example is a smaller voxel size, with four centroids.
```

A voxel grid filter downsamples the data by taking a spatial average of the points in the cloud confined by each voxel. The set of points which lie within the bounds of a voxel are assigned to that voxel and are statistically combined into one output point.

I used an X, Y, and Z voxel grid filter leaf size of 0.01. This was a good compromise of leaving enough detail while minimizing processing time.

The input point cloud is segmented in smaller voxels \(similar to smaller cube\) with a fixed size.

근접한 포인트 들에서만 좋은 성능을 보임 `only works efficiently on nearby data points`


```
Figure 3.3: Example for the downsampling quality of a voxel grid filter on a) a tree further afar and b) on a tree nearby.
The pictures on the left hand side show the input point cloud and the pictures on the right hand side illustrate the output of the voxel grid filter.
The zoomed in pictures are turned approximately 90 ◦compared to the full picture.
멀리 있는(a)가 가까이 있는(b)보다 다운 샘플링 성능이 좋지 않음
```

다운 샘플링 rate는 voxel크기에 depend하다.

* voxel크기를 0.1m로 잡으면 50%의 포인트 수가 감소 한다. \(이건 레이져 수에 따라 다를듯\)

![](https://i.imgur.com/BMFmPzv.png)


```
Figure 3.4: Downsampling rate of voxel filters.
- Left side shows the quantity of data points using different voxel sizes on the same scenario.
- Right side illustrates average amount of data points depending on the different size of voxels.
```

```python
# Import PCL module


```
A Voxel Grid filter allows us to down sample the data by taking a spatial average of the points in the could confined by
each Voxel. We can adjust the sampling size by settings the Voxel size along each dimension. The set of points which lie
within the bounds of a Voxel are assigned to that Voxel and statistically combined into one output point.
Generally downsampling to a smaller Voxel size retain more information about the original point cloud.
After experimenting a bit, 0.01 is considered as a reasonable voxel(leaf) size for the tabletop.pcd data set.
```

import pcl

# Returns Downsampled version of a point cloud
# The bigger the leaf size the less information retained
def do_voxel_grid_filter(point_cloud, LEAF_SIZE = 0.01):
  voxel_filter = point_cloud.make_voxel_grid_filter()
  voxel_filter.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE) 
  return voxel_filter.filter()
```



