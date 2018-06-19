## Crop 

```python
print("We load a polygon volume and use it to crop the original point cloud")
vol = read_selection_polygon_volume("../../TestData/Crop/cropped.json")
chair = vol.crop_point_cloud(pcd)
draw_geometries([chair])
print("")
```