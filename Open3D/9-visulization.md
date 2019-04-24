# Visualiztion 


## 1. `draw_geometries()`

Open3D provides a convenient visualization function `draw_geometries` which takes a list of geometry objects (PointCloud, TriangleMesh, or Image), and renders them together. 

### 1.1 하나씩 시각화 

```python
print("Load a ply point cloud, print it, and render it")
pcd = read_point_cloud("test.pcd")
draw_geometries([pcd])
```


### 1. 2 연속 재생[Non-blocking visualization](http://www.open3d.org/docs/tutorial/Advanced/non_blocking_visualization.html#non-blocking-visualization)

- 컨트롤 flag : `update_geometry()` and `update_renderer()`


## 2. `draw_geometries_with_custom_animation()`

> `examples/Python/Advanced/customized_visualization.py`

 allows the programmer to define a custom view trajectory and play an animation in the GUI. 



## 3. ustomized Visualization](http://www.open3d.org/docs/tutorial/Advanced/customized_visualization.html#customized-visualization)

### 3.1 Mimic draw_geometries() with Visualizer class

`draw_geometries()`와 동일한 기능 수행 


```python
def custom_draw_geometry(pcd):
    # The following code achieves the same effect as:
    # draw_geometries([pcd])
    vis = Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
```



### 3.2 Change field of view



### 3.3 `custom_draw_geometry_with_rotation()`

- 회전 기능으로, 데모용으로 좋을듯 

- Function `draw_geometries_with_animation_callback` registers a Python callback function rotate_view as the idle function of the main loop.

- It rotates the view along the x-axis whenever the visualizer is idle. This defines an animation behavior.

```python
def custom_draw_geometry_with_rotation(pcd):
    def rotate_view(vis):
        ctr = vis.get_view_control()
        ctr.rotate(10.0, 0.0)
        return False
    draw_geometries_with_animation_callback([pcd], rotate_view)
```

### 3.4 키값 연계 

- 배경색 변경 
- 깊이 정보 추출
- 이미지 캡쳐 




---

> 시각화 창에서 `h` 키 이용 도움말 보기

```
-- Mouse view control --
Left button + drag : Rotate.
Ctrl + left button + drag : Translate.
Wheel : Zoom in/out.

-- Keyboard view control --
[/] : Increase/decrease field of view.
R : Reset view point.
Ctrl/Cmd + C : Copy current view status into the clipboard.
Ctrl/Cmd + V : Paste view status from clipboard.

-- General control --
Q, Esc : Exit window.
H : Print help message.
P, PrtScn : Take a screen capture.
D : Take a depth capture.
O : Take a capture of current rendering settings.

-- Render mode control --
L : Turn on/off lighting.
+/- : Increase/decrease point size.
N : Turn on/off point cloud normal rendering.
S : Toggle between mesh flat shading and smooth shading.
W : Turn on/off mesh wireframe.
B : Turn on/off back face rendering.
I : Turn on/off image zoom in interpolation.
T : Toggle among image render:
no stretch / keep ratio / freely stretch.

-- Color control --
0..4,9 : Set point cloud color option.
0 - Default behavior, render point color.
1 - Render point color.
2 - x coordinate as color.
3 - y coordinate as color.
4 - z coordinate as color.
9 - normal as color.
Ctrl + 0..4,9: Set mesh color option.
0 - Default behavior, render uniform gray color.
1 - Render point color.
2 - x coordinate as color.
3 - y coordinate as color.
4 - z coordinate as color.
9 - normal as color.
Shift + 0..4 : Color map options.
0 - Gray scale color.
1 - JET color map.
2 - SUMMER color map.
3 - WINTER color map.
4 - HOT color map.
```










## 2. Jupyter 


---





## 1. 그리기 

> Detection 후 B.Box 그릴때 필요 할듯 
 
http://www.open3d.org/docs/tutorial/Basic/visualization.html#draw-multiple-geometries


## 2. 색변경 


np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]

![](https://cdn-ak.f.st-hatena.com/images/fotolife/r/robonchu/20180225/20180225112642.png)

```python
We pick the 1500-th point as the anchor point and paint it red.

# src/Python/Tutorial/Basic/kdtree.py

import sys
import numpy as np
sys.path.append("../..")
from py3d import *

if __name__ == "__main__":

    print("Testing kdtree in py3d ...")
    print("Load a point cloud and paint it gray.")
    pcd = read_point_cloud("../../TestData/Feature/cloud_bin_0.pcd")
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    pcd_tree = KDTreeFlann(pcd)

    print("Paint the 1500th point red.")
    pcd.colors[1500] = [1, 0, 0]

    print("Find its 200 nearest neighbors, paint blue.")
    [k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[1500], 200)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]

    print("Find its neighbors with distance less than 0.2, paint green.")
    [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[1500], 0.2)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]

    print("Visualize the point cloud.")
    draw_geometries([pcd])
    print("")
```



> [pcd.colors](http://www.open3d.org/docs/tutorial/Basic/kdtree.html#using-search-radius-vector-3d)