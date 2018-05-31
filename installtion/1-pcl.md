# 1. PCL for C++ 설치 

## 1.1 Source 설치 


```
sudo apt-get install git
cd
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
mkdir build
cd build
cmake ..
make
make install 

#checkinstall #apt-get install checkinstall
```


|Error Code | Solution|
|-|-|
|package 'eigen3' not found | apt-get install libeigen3-dev|
|package 'flann>=1.7.0' not found|apt-get install libflann-dev|





## 1.1 apt 설치 
```
add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
apt-get update
apt-get install libpcl-all

# for Ubuntu 16.04
sudo apt install libpcl-dev  # depends: libvtk6-dev
```

|Error Code | Solution|
|-|-|
|add-apt-repository command not found | `apt-get install software-properties-common python-software-properties`|
|Unable to locate package libpcl-all|apt-get install libpcl1|
|pip10, ImportError: cannot import name main|Downgrade: `python2 -m pip install --user --upgrade pip==9.0.3`|




## 1.2 deb 설치 
```
wget https://www.dropbox.com/s/9llzm20pc4opdn9/PCL-1.8.0-Linux.deb?dl=0 #파일명 에서 `?dl=0`제거 
dpkg -i PCL-1.8.0-Linux.deb
```

> 출처 : [POINT CLOUD LIBRARY ON UBUNTU 16.04 LTS](https://larrylisky.com/2016/11/03/point-cloud-library-on-ubuntu-16-04-lts/)

> [PLC for Ubuntu 16.4 & 17.10](https://askubuntu.com/questions/916260/how-to-install-point-cloud-library-v1-8-pcl-1-8-0-on-ubuntu-16-04-2-lts-for)







# 2. PCL for Python 설치 

- [공식 홈페이지](http://strawlab.github.io/python-pcl/), [example](https://github.com/strawlab/python-pcl/tree/master/examples)

- conda를 이용한 설치 방법 추천 

## 2.1 pip 설치 

```
# 
apt-get install build-essential
apt-get install -y python-pip git python-dev
apt install pkg-config

pip install numpy cython 

# ubunutu 14??
pip install git+https://github.com/strawlab/python-pcl
#pip install git+https://github.com/strawlab/python-pcl.git#egg=pcl

# ubuntu 16??
#git clone https://github.com/strawlab/python-pcl.git
#cd python-pcl
#python3 setup.py build
#python3 setup.py install
```

> 참고 : [python-pcl, python 3, ubuntu 14.04](http://adamsteer.blogspot.kr/2016/01/python-pcl-python-3-ubuntu-1404.html)


|에러코드|해결책|원인|
|-|-|-|
|fatal error: pcl/features/cppf.h: No such file or directory|`sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl;sudo apt-get update;sudo apt-get upgrade libpcl-features-dev libpcl-io-1.7 libpcl-io-1.7-dev`|ubnutu 14 떄문인|


### 2.2 conda 설치

```
conda config --add channels conda-forge
conda install -c sirokujira python-pcl #v0.3
#conda install -c https://conda.anaconda.org/ccordoba12 python-pcl  #v0.2
```


### 2.3 Python-pcl docker

```
#Dockerfile

FROM ubuntu:14.04  

RUN apt-get install -y software-properties-common
RUN add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
RUN apt-get update && \
    apt-get install -y libpcl-all  #유분투 16에서 동작 암함 

RUN apt-get install -y python-pip git python-dev
RUN pip install cython
RUN pip install numpy
RUN pip install git+https://github.com/strawlab/python-pcl.git#egg=pcl
```


|에러|해결책|
|-|-|
|Python locale error: unsupported locale setting|$ export LC_ALL="en_US.UTF-8"<br>$ export LC_CTYPE="en_US.UTF-8"<br>$ sudo dpkg-reconfigure locales|



## 3. PCL for ROS

> How-to : Gitbook - ros_autoware - rospcl

### 3.1 Source 설치 (ubuntu 14)

```
## Download
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/perception_pcl
rosdep install --from-paths ./ --ignore-src --rosdistro indigo -y

## Register {launch} with ROS via catkin_make.
cd ~/catkin_ws
catkin_make 
source ./devel/setup.sh

## Check
rospack profile
rospack list | grep {velodyne}
rospack find {}
```

## 3. VALIDATION 

### 3.1 PCL for C++

```
cd ~
mkdir pcl-test && cd pcl-test
```

Create a CMakeLists.txt file:

```
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl-test)
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl-test main.cpp)
target_link_libraries(pcl-test ${PCL_LIBRARIES})

SET(COMPILE_FLAGS "-std=c++11")
add_definitions(${COMPILE_FLAGS})
```

Create a main.cpp file:

```
#include <iostream>

int main() {
    std::cout << "hello, world!" << std::endl;
    return (0);
}
```

Compile:

```
mkdir build && cd build
cmake ..
make
```

Test: 

```
./pcl-test
``` 

Output -> hello, world!

### 3.2 PCL for Python

```
python
import pcl
```

---

# pypcd

[github](https://github.com/dimatura/pypcd)

설치 (Python2 Only)

```
# pip이용설치
pip install pypcd

# 소스코드 설치 
git clone https://github.com/dimatura/pypcd
python setup.py

```

- PCD to numpy / pandas

- PCD to Images

- ROS지원 : [설치](https://github.com/dimatura/pypcd#using-with-ros) & [사용법]()


---

```python
from __future__ import print_function
import pcl
import numpy as np


#p = pcl.PointCloud()
#p.from_file("test.pcd") # Deprecated; use pcl.load instead.

pc = pcl.load("sample_sec.pcd")
pa = pc.to_array()

print(type(pc))
print(type(pa))

print(pa.shape)
print(pa.ndim)
print(pa.dtype)

print(pa)

pc.width


cloud = pcl.load('sample_sec.pcd')
print ('Loaded ' + str(cloud.width * cloud.height) + ' data points from test_pcd.pcd with the following fields: ')
for i in range(0, cloud.size):
    print ('x: ' + str(cloud[i][0]) + ', y : ' + str(cloud[i][1]) + ', z : ' + str(cloud[i][2]))

```



