
## File input 

```python

from pyntcloud import PyntCloud

cloud = PyntCloud.from_file("test/data/filters/filters.ply")

print(cloud)

```

## 생성 

```python

import pandas as pd
from pyntcloud import PyntCloud

cloud = PyntCloud.from_file("example.pts",
                            sep=" ",
                            header=0,
                            names=["x","y","z"])

```



## File ouput
```python
my_point_cloud.to_file("out_file.obj", internal=["points", "mesh"])

```