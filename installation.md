![](https://www.pdal.io/_images/pdal_logo.png)

# PDAL - Point Data Abstraction Library

PDAL is a C++ BSD library for translating and manipulating point cloud data. [[홈페이지](https://www.pdal.io/index.html)

[A Tour of 3D Point Cloud Processing](https://www.rockestate.be/blog/2017/10/26/point-cloud-processing.html): 2017, Combining the strengths of pdal, ipyvolume and jupyter

> 참고 : [libpointmatcher](https://libpointmatcher.readthedocs.io/en/latest/) : [Down-sampling](https://libpointmatcher.readthedocs.io/en/latest/Datafilters/)

![image](https://user-images.githubusercontent.com/17797922/41730145-e92bc65e-752f-11e8-97be-771c986d5a5f.png)


```
# apt (Ubuntu 16.10)
apt-get install python3-pdal

# pip
pip install PDAL # https://pypi.org/project/PDAL/

# Conda
## `pdal` will install the PDAL binaries and development files.
## `python-pdal` will install the PDAL Python extension.
conda install -c conda-forge pdal python-pdal

```
---

# pyntcloud 

- Python 3 based
- [홈페이지](http://pyntcloud.readthedocs.io/en/latest/index.html#)
- [Github](https://github.com/daavoo/pyntcloud)

 Installation

```python 
# ubuntu16
pip3 install  numpy numba scipy pandas matplotlib
pip3 install git+https://github.com/daavoo/pyntcloud
```


Lightweigth visualizer

```python
#python3 
from pyntcloud import PyntCloud

```

![](https://raw.githubusercontent.com/daavoo/pyntcloud/master/docs/images/plot1.gif)


테트용 ply : https://raw.githubusercontent.com/daavoo/pyntcloud/master/examples/data/ankylosaurus_mesh.ply



---

# pypcd

[github](https://github.com/dimatura/pypcd)

설치 

> It also can't run on Python 3(Python2 Only??)



```python
# pip이용설치
pip3 install pypcd
#pip install pypcd

# 소스코드 설치 
git clone https://github.com/dimatura/pypcd
python setup.py

```

- PCD to numpy / pandas

- PCD to Images

---

# Re-implementation in pure python of Point Cloud Library (PCL)


pip3 install numba 


https://github.com/cmpute/pypcl


https://pypi.org/project/PyPCL/



https://github.com/cmpute/pypcl/blob/master/test/io_test.py

import pcl

---
# pylas

pip install pylas
Another way of reading LAS/LAZ in Python.