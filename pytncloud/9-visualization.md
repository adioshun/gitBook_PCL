```python

cloud = PyntCloud.from_file("sample_table.pcd")
cloud.plot() #jupyter
```



```python 

#pcd 

import pcl
import numpy as np
import pandas as pd



cloud = pcl.load("sample_sec.pcd")
pa = cloud.to_array()
df = pd.DataFrame(pa, columns=['x', 'y', 'z'])
df.to_csv('test.csv')


from pyntcloud import PyntCloud
cloud = PyntCloud.from_file("test.csv")
cloud.plot()


```