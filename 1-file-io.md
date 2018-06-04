
```python
import pypcd
# also can read from file handles.
pc = pypcd.PointCloud.from_path('foo.pcd')
# pc.pc_data has the data as a structured array
# pc.fields, pc.count, etc have the metadata

meta = pc.get_metadata()
meta


# save as binary compressed
pc.save_pcd('bar.pcd', compression='binary_compressed')
```