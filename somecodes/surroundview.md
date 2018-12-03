# Surround VIew 

## 1. ronny (python)

> http://ronny.rest/blog/post_2017_04_03_point_cloud_panorama/



## 2. [Surround View](https://github.com/hengck23/didi-udacity-2017/blob/master/baseline-04/didi_data/lidar_surround.py)


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


## 360 side view from
## http://ronny.rest/blog/post_2017_04_03_point_cloud_panorama/
## See Bo li's paper:
##    http://prclibo.github.io/
##    [1] "Multi-View 3D Object Detection Network for Autonomous Driving" - Xiaozhi Chen, Huimin Ma, Ji Wan, Bo Li and Tian Xia , arXiv 2016
##    [2] "3D Fully Convolutional Network for Vehicle Detection in Point Cloud" - Bo Li, arXiv 2016
##    [3] "Vehicle Detection from 3D Lidar Using Fully Convolutional Network" - Bo Li and Tianlei Zhang and Tian Xia , arXiv 2016
##


##   cylindrial projection
SURROUND_U_STEP = 1.    #resolution
SURROUND_V_STEP = 1.33
SURROUND_U_MIN, SURROUND_U_MAX = np.array([0,    360])/SURROUND_U_STEP  # horizontal of cylindrial projection
SURROUND_V_MIN, SURROUND_V_MAX = np.array([-90,   90])/SURROUND_V_STEP  # vertical   of cylindrial projection


def lidar_to_surround_coords(x, y, z, d ):
    u =   np.arctan2(x, y)/np.pi*180 /SURROUND_U_STEP
    v = - np.arctan2(z, d)/np.pi*180 /SURROUND_V_STEP
    u = (u +90)%360  ##<todo> car will be spit into 2 at boundary  ...

    u = np.rint(u)
    v = np.rint(v)
    u = (u - SURROUND_U_MIN).astype(np.uint8)
    v = (v - SURROUND_V_MIN).astype(np.uint8)

    return u,v


def lidar_to_surround(lidar):
    def normalise_to_255(a):
        return (((a - min(a)) / float(max(a) - min(a))) * 255).astype(np.uint8)

    x = lidar['x']
    y = lidar['y']
    z = lidar['z']
    r = lidar['intensity']
    d = np.sqrt(x ** 2 + y ** 2)  # map distance relative to origin
    u,v = lidar_to_surround_coords(x,y,z,d)

    width  = int(SURROUND_U_MAX - SURROUND_U_MIN + 1)
    height = int(SURROUND_V_MAX - SURROUND_V_MIN + 1)
    surround     = np.zeros((height, width, 3), dtype=np.float32)
    surround_img = np.zeros((height, width, 3), dtype=np.uint8)

    surround[v, u, 0] = d
    surround[v, u, 1] = z
    surround[v, u, 2] = r
    surround_img[v, u, 0] = normalise_to_255(np.clip(d,     0, 30))
    surround_img[v, u, 1] = normalise_to_255(np.clip(z+1.8, 0, 100))
    surround_img[v, u, 2] = normalise_to_255(np.clip(r,     0, 30))

    return surround, surround_img


## drawing ####
def box3d_to_surround_box(boxes3d):
    is_reshape = boxes3d.shape==(8,3) #support for single box3d

    if is_reshape:
        boxes3d = boxes3d.reshape(1,8,3)

    num = len(boxes3d)
    surround_boxes = np.zeros((num,4),  dtype=np.float32)
    for n in range(num):
        b = boxes3d[n]

        x = b[:,0]
        y = b[:,1]
        z = b[:,2]
        d = np.sqrt(x ** 2 + y ** 2)
        u,v = lidar_to_surround_coords(x,y,z,d)
        umin,umax = np.min(u),np.max(u)
        vmin,vmax = np.min(v),np.max(v)
        surround_boxes[n] = np.array([umin,vmin,umax,vmax])

    if is_reshape:
        surround_boxes = surround_boxes.squeeze()

    return surround_boxes

def draw_box3d_on_surround(image, boxes3d, color=(255,255,255)):

    surround_boxes = box3d_to_surround_box(boxes3d)
    is_reshape = surround_boxes.shape==(4)
    if is_reshape:
        surround_boxes = surround_boxes.reshape(1,4)

    num = len(surround_boxes)
    for n in range(num):
        b = surround_boxes[n]
        x1,y1,x2,y2  = b
        cv2.rectangle(image,(x1,y1),(x2,y2),color,1,cv2.LINE_AA)


# main #################################################################
# for demo data:  /root/share/project/didi/data/didi/didi-2/Out/1/15

if __name__ == '__main__':

    lidar_dir              = '/root/share/project/didi/data/didi/didi-2/Out/1/15/lidar'
    gt_boxes3d_dir         = '/root/share/project/didi/data/didi/didi-2/Out/1/15/processed/gt_boxes3d'
    lidar_surround_dir     = '/root/share/project/didi/data/didi/didi-2/Out/1/15/processed/lidar_surround'
    lidar_surround_img_dir = '/root/share/project/didi/data/didi/didi-2/Out/1/15/processed/lidar_surround_img'

    mark_dir  = '/root/share/project/didi/data/didi/didi-2/Out/1/15/processed/mark-surround-box'
    avi_file  = '/root/share/project/didi/data/didi/didi-2/Out/1/15/processed/mark-surround-box.avi'
    os.makedirs(lidar_surround_dir, exist_ok=True)
    os.makedirs(lidar_surround_img_dir, exist_ok=True)
    os.makedirs(mark_dir, exist_ok=True)

    fig   = mlab.figure(figure=None, bgcolor=(0,0,0), fgcolor=None, engine=None, size=(500, 500))
    for file in sorted(glob.glob(lidar_dir + '/*.npy')):
        name = os.path.basename(file).replace('.npy','')

        lidar_file         = lidar_dir +'/'+name+'.npy'
        surround_file      = lidar_surround_dir +'/'+name+'.npy'
        surround_img_file  = lidar_surround_img_dir +'/'+name+'.png'
        mark_file    = mark_dir +'/'+name+'.png'
        boxes3d_file = gt_boxes3d_dir+'/'+name+'.npy'

        lidar = np.load(lidar_file)
        surround, surround_img = lidar_to_surround(lidar)
        boxes3d = np.load(boxes3d_file)

        #save
        cv2.imwrite(surround_img_file,surround_img)
        np.save(surround_file,surround)

        #show
        mlab.clf(fig)
        draw_didi_lidar(fig, lidar, is_grid=1, is_axis=1)

        if len(boxes3d)!=0:
            draw_didi_boxes3d(fig, boxes3d)
            draw_box3d_on_surround(surround_img, boxes3d, color=(255,255,255))

        azimuth,elevation,distance,focalpoint = MM_PER_VIEW1
        mlab.view(azimuth,elevation,distance,focalpoint)

        mlab.show(1)
        imshow('surround_img',surround_img,3)
        cv2.waitKey(10)

        #save
        cv2.imwrite(mark_file,surround_img)

    dir_to_avi(avi_file, mark_dir)


```