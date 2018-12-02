# [Top View](https://github.com/hengck23/didi-udacity-2017/blob/master/baseline-04/didi_data/lidar_top.py)


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