```

fromfrom pyntcloud  pyntcloud importimport PyntCloud

cloud  PyntCloud  cloud == PyntCloud.from_file( PyntCloud.from_file(""some_file.plysome_file.ply"")

cloud.add_scalar_field()  cloud.add_scalar_field(""hsvhsv"")

voxelgrid_id )  voxelgrid_id == cloud.add_structure( cloud.add_structure(""voxelgridvoxelgrid"", , x_y_zx_y_z==[[3232, , 3232, , 3232])

points ])  points == cloud.get_sample( cloud.get_sample(""voxelgrid_nearestvoxelgrid_nearest"", , voxelgridvoxelgrid==voxelgrid_id)

new_cloud voxelgrid_id)  new_cloud == PyntCloud(points)

new_cloud.to_file( PyntCloud(points)  new_cloud.to "out_file.ply")
```