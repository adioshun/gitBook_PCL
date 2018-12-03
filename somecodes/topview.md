# Top View 

## 1. ROS bird view `by mjshiggins`

> https://github.com/mjshiggins/ros-examples

```
rosrun lidar lidar_node
토픽 : /points_raw

```


## 2. [velodyne_height_map](http://wiki.ros.org/velodyne_height_map)

- ROS, 3D Lidar를 2D BEV로 변경
- ROS obstacle detection for 3D point clouds using a height map algorithm

```
libvtkproj4-6.2.so.6.2.0에러시
sudo apt-get install libvtk6-dev

```

rosrun velodyne_height_map heightmap_node _height_threshold:=0.05 #5cm 이상 크기




## 3. [Top View](https://github.com/hengck23/didi-udacity-2017/blob/master/baseline-04/didi_data/lidar_top.py) `by hengck23` (python)


```python

import os
os.environ['HOME'] = '/root'

SEED = 202


# std libs
import glob


# num libs
import math
import random
import numpy as np
random.seed(SEED)
np.random.seed(SEED)

import cv2
import mayavi.mlab as mlab

from didi_data.lidar import *


## top side view from
## http://ronny.rest/blog/post_2017_03_26_lidar_birds_eye/
## See Bo li's paper:
##    http://prclibo.github.io/
##    [1] "Multi-View 3D Object Detection Network for Autonomous Driving" - Xiaozhi Chen, Huimin Ma, Ji Wan, Bo Li and Tian Xia , arXiv 2016
##    [2] "3D Fully Convolutional Network for Vehicle Detection in Point Cloud" - Bo Li, arXiv 2016
##    [3] "Vehicle Detection from 3D Lidar Using Fully Convolutional Network" - Bo Li and Tianlei Zhang and Tian Xia , arXiv 2016
##

TOP_Y_MIN=-20     #40
TOP_Y_MAX=+20
TOP_X_MIN=-20
TOP_X_MAX=+20     #70.4
TOP_Z_MIN=-2.0    ###<todo> determine the correct values!
TOP_Z_MAX= 0.4

TOP_X_STEP=0.1  #0.1
TOP_Y_STEP=0.1
TOP_Z_STEP=0.4


def lidar_to_top_coords(x,y,z=None):
    X0, Xn = 0, int((TOP_X_MAX-TOP_X_MIN)//TOP_X_STEP)+1
    Y0, Yn = 0, int((TOP_Y_MAX-TOP_Y_MIN)//TOP_Y_STEP)+1
    xx = Yn-int((y-TOP_Y_MIN)//TOP_Y_STEP)
    yy = Xn-int((x-TOP_X_MIN)//TOP_X_STEP)

    return xx,yy


def top_to_lidar_coords(xx,yy):
    X0, Xn = 0, int((TOP_X_MAX-TOP_X_MIN)//TOP_X_STEP)+1
    Y0, Yn = 0, int((TOP_Y_MAX-TOP_Y_MIN)//TOP_Y_STEP)+1
    y = Xn*TOP_Y_STEP-(xx+0.5)*TOP_Y_STEP + TOP_Y_MIN
    x = Yn*TOP_X_STEP-(yy+0.5)*TOP_X_STEP + TOP_X_MIN

    return x,y



## lidar to top ##
def lidar_to_top(lidar):

    idx = np.where (lidar['x']>TOP_X_MIN)
    lidar = lidar[idx]
    idx = np.where (lidar['x']<TOP_X_MAX)
    lidar = lidar[idx]

    idx = np.where (lidar['y']>TOP_Y_MIN)
    lidar = lidar[idx]
    idx = np.where (lidar['y']<TOP_Y_MAX)
    lidar = lidar[idx]

    idx = np.where (lidar['z']>TOP_Z_MIN)
    lidar = lidar[idx]
    idx = np.where (lidar['z']<TOP_Z_MAX)
    lidar = lidar[idx]

    x = lidar['x']
    y = lidar['y']
    z = lidar['z']
    r = lidar['intensity']
    qxs=((x-TOP_X_MIN)//TOP_X_STEP).astype(np.int32)
    qys=((y-TOP_Y_MIN)//TOP_Y_STEP).astype(np.int32)
    qzs=((z-TOP_Z_MIN)//TOP_Z_STEP).astype(np.int32)
    quantized = np.dstack((qxs,qys,qzs,r)).squeeze()

    X0, Xn = 0, int((TOP_X_MAX-TOP_X_MIN)//TOP_X_STEP)+1
    Y0, Yn = 0, int((TOP_Y_MAX-TOP_Y_MIN)//TOP_Y_STEP)+1
    Z0, Zn = 0, int((TOP_Z_MAX-TOP_Z_MIN)//TOP_Z_STEP)+1
    height  = Yn - Y0
    width   = Xn - X0
    channel = Zn - Z0  + 2
    print('height,width,channel=%d,%d,%d'%(height,width,channel))
    top = np.zeros(shape=(width,height,channel), dtype=np.float32)


    # histogram = Bin(channel, 0, Zn, "z", Bin(height, 0, Yn, "y", Bin(width, 0, Xn, "x", Maximize("intensity"))))
    # histogram.fill.numpy({"x": qxs, "y": qys, "z": qzs, "intensity": prs})

    if 1:  #new method
        for z in range(Zn):
            iz = np.where (quantized[:,2]==z)
            quantized_z = quantized[iz]

            for y in range(Yn):
                iy  = np.where (quantized_z[:,1]==y)
                quantized_zy = quantized_z[iy]

                for x in range(Xn):
                    ix  = np.where (quantized_zy[:,0]==x)
                    quantized_zyx = quantized_zy[ix]
                    if len(quantized_zyx)>0:
                        yy,xx,zz = -x,-y, z

                        #height per slice
                        max_height = max(0,np.max(quantized_zyx[:,2])-TOP_Z_MIN)
                        top[yy,xx,zz]=max_height

                        #intensity
                        max_intensity = np.max(quantized_zyx[:,3])
                        top[yy,xx,Zn]=max_intensity

                        #density
                        count = len(idx)
                        top[yy,xx,Zn+1]+=count

                    pass
                pass
            pass

    top[:,:,Zn+1] = np.log(top[:,:,Zn+1]+1)/math.log(16)

    if 0:
        top_image = np.sum(top,axis=2)
        top_image = top_image-np.min(top_image)
        top_image = (top_image/np.max(top_image)*255)
        #top_image = np.clip(top_image,0,255)
        top_image = np.dstack((top_image, top_image, top_image)).astype(np.uint8)


    if 1: #unprocess
        top_image = np.zeros((height,width),dtype=np.float32)

        num = len(lidar)
        for n in range(num):
            x,y   = qxs[n],qys[n]
            yy,xx = -x,-y
            top_image[yy,xx] += 1

        max_value = np.max(np.log(top_image+0.001))
        top_image = top_image/max_value *255
        top_image = np.dstack((top_image, top_image, top_image)).astype(np.uint8)



    return top, top_image

## drawing ####
def box3d_to_top_box(boxes3d):

    is_reshape = boxes3d.shape==(8,3) #support for single box3d
    if is_reshape:
        boxes3d = boxes3d.reshape(1,8,3)

    num  = len(boxes3d)
    top_boxes = np.zeros((num,4),  dtype=np.float32)
    for n in range(num):
        b   = boxes3d[n]

        x0 = b[0,0]
        y0 = b[0,1]
        x1 = b[1,0]
        y1 = b[1,1]
        x2 = b[2,0]
        y2 = b[2,1]
        x3 = b[3,0]
        y3 = b[3,1]
        u0,v0=lidar_to_top_coords(x0,y0)
        u1,v1=lidar_to_top_coords(x1,y1)
        u2,v2=lidar_to_top_coords(x2,y2)
        u3,v3=lidar_to_top_coords(x3,y3)

        umin=min(u0,u1,u2,u3)
        umax=max(u0,u1,u2,u3)
        vmin=min(v0,v1,v2,v3)
        vmax=max(v0,v1,v2,v3)

        top_boxes[n]=np.array([umin,vmin,umax,vmax])

    if is_reshape:
        top_boxes = top_boxes.squeeze()

    return top_boxes


def top_box_to_box3d(boxes):

    is_reshape = boxes.shape==(4) #support for single box
    if is_reshape:
        boxes = boxes.reshape(1,4)

    num = len(boxes)
    boxes3d = np.zeros((num,8,3),dtype=np.float32)
    for n in range(num):
        x1,y1,x2,y2 = boxes[n]
        points = [ (x1,y1), (x1,y2), (x2,y2), (x2,y1) ]
        for k in range(4):
            xx,yy = points[k]
            x,y  = top_to_lidar_coords(xx,yy)
            boxes3d[n,k,  :] = x,y, -2  ## <todo>
            boxes3d[n,4+k,:] = x,y,0.4

    if is_reshape:
        boxes3d = boxes3d.squeeze()

    return boxes3d



def draw_box3d_on_top(image, boxes3d, color=(255,255,255)):

    top_boxes = box3d_to_top_box(boxes3d)
    is_reshape = top_boxes.shape==(4)
    if is_reshape:
        top_boxes = top_boxes.reshape(1,4)

    num = len(top_boxes)
    for n in range(num):
        b = top_boxes[n]
        x1,y1,x2,y2  = b
        cv2.rectangle(image,(x1,y1),(x2,y2),color,1,cv2.LINE_AA)



# main #################################################################
# for demo data:  /root/share/project/didi/data/didi/didi-2/Out/1/15

if __name__ == '__main__':

    lidar_dir         = '/root/share/project/didi/data/didi/didi-2/Out/1/15/lidar'
    gt_boxes3d_dir    = '/root/share/project/didi/data/didi/didi-2/Out/1/15/processed/gt_boxes3d'
    lidar_top_dir     = '/root/share/project/didi/data/didi/didi-2/Out/1/15/processed/lidar_top'
    lidar_top_img_dir = '/root/share/project/didi/data/didi/didi-2/Out/1/15/processed/lidar_top_img'
    mark_dir          = '/root/share/project/didi/data/didi/didi-2/Out/1/15/processed/mark-top-box'
    avi_file          = '/root/share/project/didi/data/didi/didi-2/Out/1/15/processed/mark-top-box.avi'
    os.makedirs(mark_dir, exist_ok=True)
    os.makedirs(lidar_top_dir, exist_ok=True)
    os.makedirs(lidar_top_img_dir, exist_ok=True)

    fig   = mlab.figure(figure=None, bgcolor=(0,0,0), fgcolor=None, engine=None, size=(500, 500))
    for file in sorted(glob.glob(lidar_dir + '/*.npy')):
        name = os.path.basename(file).replace('.npy','')

        lidar_file    = lidar_dir +'/'+name+'.npy'
        top_file      = lidar_top_dir +'/'+name+'.npy'
        top_img_file  = lidar_top_img_dir +'/'+name+'.png'
        mark_file     = mark_dir +'/'+name+'.png'
        boxes3d_file  = gt_boxes3d_dir+'/'+name+'.npy'

        lidar = np.load(lidar_file)
        top, top_img = lidar_to_top(lidar)
        boxes3d = np.load(boxes3d_file)

        #save
        cv2.imwrite(top_img_file,top_img)
        np.save(top_file,top)

        #show
        mlab.clf(fig)
        draw_didi_lidar(fig, lidar, is_grid=1, is_axis=1)

        if len(boxes3d)!=0:
            draw_didi_boxes3d(fig, boxes3d)
            draw_box3d_on_top(top_img, boxes3d, color=(255,255,255))


        azimuth,elevation,distance,focalpoint = MM_PER_VIEW1
        mlab.view(azimuth,elevation,distance,focalpoint)
        #
        mlab.show(1)
        imshow('top_img',top_img,1)
        cv2.waitKey(1)

        #save
        cv2.imwrite(mark_file,top_img)

    dir_to_avi(avi_file, mark_dir)

```

## 4. Top View `by ronny` (python)



> http://ronny.rest/blog/post_2017_03_26_lidar_birds_eye/


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



