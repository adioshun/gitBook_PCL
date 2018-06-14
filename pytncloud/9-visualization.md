```python

cloud = PyntCloud.from_file("sample_sec.pcd")
#Error : ValueError: field '__0000' occurs more than once

cloud.plot() #jupyter
```

dataframe으로 변경 후 실행  

```python 

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