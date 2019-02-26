# Bird Eye View

## 1. ROS BEV to png
https://github.com/mjshiggins/ros-examples
---

## 2. ROS Height Map 

[velodyne_height_map](http://wiki.ros.org/velodyne_height_map): ROS, 3D Lidar를 2D BEV로 변경
- ROS obstacle detection for 3D point clouds using a height map algorithm

```
libvtkproj4-6.2.so.6.2.0에러시
sudo apt-get install libvtk6-dev

```

rosrun velodyne_height_map heightmap_node _height_threshold:=0.05 #5cm 이상 크기

---





---
http://ronny.rest/blog/post_2017_03_26_lidar_birds_eye/


```python

from PIL import Image as Image2
import cv2


import numpy as np

# ==============================================================================
# SCALE_TO_255
# ==============================================================================
def scale_to_255(a, min, max, dtype=np.uint8):
""" Scales an array of values from specified min, max range to 0-255
Optionally specify the data type of the output (default is uint8)
"""
return (((a - min) / float(max - min)) * 255).astype(dtype)


# ==============================================================================
# BIRDS_EYE_POINT_CLOUD
# ==============================================================================
def birds_eye_point_cloud(points,
side_range=(-10, 10),
fwd_range=(-10,10),
res=0.1,
min_height = -2.73,
max_height = 1.27,
saveto=None):
""" Creates an 2D birds eye view representation of the point cloud data.
You can optionally save the image to specified filename.

Args:
points: (numpy array)
N rows of points data
Each point should be specified by at least 3 elements x,y,z
side_range: (tuple of two floats)
(-left, right) in metres
left and right limits of rectangle to look at.
fwd_range: (tuple of two floats)
(-behind, front) in metres
back and front limits of rectangle to look at.
res: (float) desired resolution in metres to use
Each output pixel will represent an square region res x res
in size.
min_height: (float)(default=-2.73)
Used to truncate height values to this minumum height
relative to the sensor (in metres).
The default is set to -2.73, which is 1 metre below a flat
road surface given the configuration in the kitti dataset.
max_height: (float)(default=1.27)
Used to truncate height values to this maximum height
relative to the sensor (in metres).
The default is set to 1.27, which is 3m above a flat road
surface given the configuration in the kitti dataset.
saveto: (str or None)(default=None)
Filename to save the image as.
If None, then it just displays the image.
"""
if points.ndim is 0:
print ("Points .ndim is 0")
else:
x_lidar = points[:, 0]
y_lidar = points[:, 1]
z_lidar = points[:, 2]
# r_lidar = points[:, 3] # Reflectance

# INDICES FILTER - of values within the desired rectangle
# Note left side is positive y axis in LIDAR coordinates
ff = np.logical_and((x_lidar > fwd_range[0]), (x_lidar < fwd_range[1]))
ss = np.logical_and((y_lidar > -side_range[1]), (y_lidar < -side_range[0]))
indices = np.argwhere(np.logical_and(ff,ss)).flatten()

# CONVERT TO PIXEL POSITION VALUES - Based on resolution
x_img = (-y_lidar[indices]/res).astype(np.int32) # x axis is -y in LIDAR
y_img = (x_lidar[indices]/res).astype(np.int32) # y axis is -x in LIDAR
# will be inverted later

# SHIFT PIXELS TO HAVE MINIMUM BE (0,0)
# floor used to prevent issues with -ve vals rounding upwards
x_img -= int(np.floor(side_range[0]/res))
y_img -= int(np.floor(fwd_range[0]/res))

# CLIP HEIGHT VALUES - to between min and max heights
pixel_values = np.clip(a = z_lidar[indices],
a_min=min_height,
a_max=max_height)

# RESCALE THE HEIGHT VALUES - to be between the range 0-255
pixel_values = scale_to_255(pixel_values, min=min_height, max=max_height)

# FILL PIXEL VALUES IN IMAGE ARRAY
x_max = int((side_range[1] - side_range[0])/res)
y_max = int((fwd_range[1] - fwd_range[0])/res)
im = np.zeros([y_max, x_max], dtype=np.uint8)
im[-y_img, x_img] = pixel_values # -y because images start from top left

imgRGB=cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)

"""
# Convert from numpy array to a PIL image
im2 = Image2.fromarray(im)

# SAVE THE IMAGE
if saveto is not None:
im2.save(saveto)
else:
im2.show()
"""
return imgRGB


```

```
# View a Square that is 10m on all sides of the car
birds_eye_point_cloud(lidar, side_range=(-10, 10), fwd_range=(-10, 10), res=0.1, saveto="lidar_pil_01.png")


# View a Square that is 10m on either side of the car and 20m in front
birds_eye_point_cloud(lidar, side_range=(-10, 10), fwd_range=(0, 20), res=0.1, saveto="lidar_pil_02.png")

# View a rectangle that is 5m on either side of the car and 20m in front
birds_eye_point_cloud(lidar, side_range=(-5, 5), fwd_range=(0, 20), res=0.1, saveto="lidar_pil_03.png")
```

```
data = np.asarray(input_pcl_xyz)


if data.ndim is 0:
print ("data.ndim is 0")
else :
frame = birds_eye_point_cloud(data, side_range=(-10, 10), fwd_range=(-10, 10), res=0.05, saveto=None)



#bridge = CvBridge()
image_pub = rospy.Publisher("image_topic_3",Image, queue_size=10)
image_pub.publish(bridge.cv2_to_imgmsg(frame, "passthrough"))

```



