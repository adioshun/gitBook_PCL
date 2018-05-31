# Visulaiztion 

## 1. 보기 


Open3D provides a convenient visualization function `draw_geometries` which takes a list of geometry objects (PointCloud, TriangleMesh, or Image), and renders them together. 

```python
print("Load a ply point cloud, print it, and render it")
pcd = read_point_cloud("test.pcd")
draw_geometries([pcd])
```

> 시각화 창에서 `h` 키 이용 도움말 보기 





## 그리기 

> Detection 후 B.Box 그릴때 필요 할듯 
 
http://www.open3d.org/docs/tutorial/Basic/visualization.html#draw-multiple-geometries