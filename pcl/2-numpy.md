```python
import numpy as np
import pcl
p = pcl.PointCloud(10)  # "empty" point cloud
a = np.asarray(p)       # NumPy view on the cloud
a[:] = 0                # fill with zeros
print(p[3])             # prints (0.0, 0.0, 0.0)
a[:, 0] = 1             # set x coordinates to 1
print(p[3])             # prints (1.0, 0.0, 0.0)

```



## 1. From pc to arr

pa = pc.
## 2. From arr to pc

pc = pcl.PointCloud(array)




---

함수명	기능	파일포멧
np.save()	NumPy 배열 객체 1개를 파일에 저장	바이너리
np.savez()	NumPy 배열 객체 복수개를 파일에 저장	바이너리
np.load()	NumPy 배열 저장 파일로 부터 객체 로딩	바이너리
np.loadtxt()	텍스트 파일로 부터 배열 로딩	텍스트
np.savetxt()	텍스트 파일에 NumPy 배열 객체 저장	텍스트