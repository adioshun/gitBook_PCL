# Surround VIew 

## 1. ronny (python)

> http://ronny.rest/blog/post_2017_04_03_point_cloud_panorama/



## 2. [Surround View](https://github.com/hengck23/didi-udacity-2017/blob/master/baseline-04/didi_data/lidar_surround.py)


```python 



SEED = 202

import math
import random
import numpy as np
random.seed(SEED)
np.random.seed(SEED)

import cv2
from lidar import *


##   cylindrial projection
SURROUND_U_STEP = 1.    #resolution
SURROUND_V_STEP = 1.33
SURROUND_U_MIN, SURROUND_U_MAX = np.array([0,    360])/SURROUND_U_STEP  # horizontal of cylindrial projection
SURROUND_V_MIN, SURROUND_V_MAX = np.array([-90,   90])/SURROUND_V_STEP  # vertical   of cylindrial projection


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

def lidar_to_surround_coords(x, y, z, d ):
    u =   np.arctan2(x, y)/np.pi*180 /SURROUND_U_STEP
    v = - np.arctan2(z, d)/np.pi*180 /SURROUND_V_STEP
    u = (u +90)%360  ##<todo> car will be spit into 2 at boundary  ...

    u = np.rint(u)
    v = np.rint(v)
    u = (u - SURROUND_U_MIN).astype(np.uint8)
    v = (v - SURROUND_V_MIN).astype(np.uint8)

    return u,v

lidar = np.load("/root/share/project/didi/data/didi/didi-2/Data/1/15/lidar/1530509304325762000.npy")
surround, surround_img = lidar_to_surround(lidar)
cv2.imwrite("./output/surround.png",surround_img)
from IPython.display import Image
Image(filename="./output/surround.png") 


```