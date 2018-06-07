# Visulaiztion 

## 1. 보기 


Open3D provides a convenient visualization function `draw_geometries` which takes a list of geometry objects (PointCloud, TriangleMesh, or Image), and renders them together. 

```python
print("Load a ply point cloud, print it, and render it")
pcd = read_point_cloud("test.pcd")
draw_geometries([pcd])
```

> 시각화 창에서 `h` 키 이용 도움말 보기 

```
  -- Mouse view control --
    Left button + drag        : Rotate.
    Ctrl + left button + drag : Translate.
    Wheel                     : Zoom in/out.

  -- Keyboard view control --
    [/]          : Increase/decrease field of view.
    R            : Reset view point.
    Ctrl/Cmd + C : Copy current view status into the clipboard.
    Ctrl/Cmd + V : Paste view status from clipboard.

  -- General control --
    Q, Esc       : Exit window.
    H            : Print help message.
    P, PrtScn    : Take a screen capture.
    D            : Take a depth capture.
    O            : Take a capture of current rendering settings.

  -- Render mode control --
    L            : Turn on/off lighting.
    +/-          : Increase/decrease point size.
    N            : Turn on/off point cloud normal rendering.
    S            : Toggle between mesh flat shading and smooth shading.
    W            : Turn on/off mesh wireframe.
    B            : Turn on/off back face rendering.
    I            : Turn on/off image zoom in interpolation.
    T            : Toggle among image render:
                   no stretch / keep ratio / freely stretch.

  -- Color control --
    0..4,9       : Set point cloud color option.
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




## 2. 그리기 

> Detection 후 B.Box 그릴때 필요 할듯 
 
http://www.open3d.org/docs/tutorial/Basic/visualization.html#draw-multiple-geometries


## 3. 색변경 


np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]

> [pcd.colors](http://www.open3d.org/docs/tutorial/Basic/kdtree.html#using-search-radius-vector-3d)