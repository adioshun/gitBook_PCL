
## File input 

```python

from pyntcloud import PyntCloud

cloud = PyntCloud.from_file("test/data/filters/filters.ply")

cloud = PyntCloud.from_file("example.pts",
                            sep=" ",
                            header=0,
                            names=["x","y","z"])


#정보 출력 
print(cloud)
cloud.points.describe()
cloud.points.boxplot()
```

## 생성 

요구사항 #1 : points must be a pandas DataFrame
요구사항 #2 : points must have ‘x’, ‘y’ and ‘z’ columns




```python

import pandas as pd
from pyntcloud import PyntCloud

# 에러 발생 
points = pd.DataFrame(np.random.rand(1000, 3))
cloud = PyntCloud(points)

# 에러 발생 
points = np.random.rand(1000, 3)
cloud = PyntCloud(points)


cloud = pcl.load("sample_sec.pcd")
pa = cloud.to_array()

df = pd.DataFrame(pa, columns=['x', 'y', 'z'])
cloud = PyntCloud(df)  # 되는지 확인 필요




```



## File ouput
```python
my_point_cloud.to_file("out_file.obj", internal=["points", "mesh"])

```