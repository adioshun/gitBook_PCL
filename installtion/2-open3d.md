# Open 3D 

## 1. 소개 

관련 링크 : [홈페이지](http://www.open3d.org/docs/getting_started.html#compiling-from-source), [깃허브](), [논문](http://www.open3d.org/paper.pdf), [블로그]()


기존 PCL의 제약 
- Only C ++ can be used (C ++ weak ...)
- I have to build every bit for a bit of correction
- I need to write a lot of rows

Open3D 장점 
- Can use Python
- It is multiplied by a small number of lines (it is 1/5 of PCL)
- Debugging with Jupyter Notebook


## 2. 설치 

### 2.1 [pip](https://pypi.org/project/open3d-python/) 설치 

ubnutu16(0)

```
apt-get install libglu1-mesa-dev libgl1-mesa-glx libglew-dev libglfw3-dev libjsoncpp-dev libeigen3-dev libpng16-dev libjpeg-dev python-dev python3-dev python-tk python3-tk
pip install open3d-python
```

### 2.3 소스코드 이용 설치

Ubuntu 16(x), 17(o) & 18(o), python3, pip

```python
apt-get install cmake 
pip3 install numpy

cd ~
git clone https://github.com/IntelVCL/Open3D

# install the dependencies and use CMake to build the project
cd ~/Open3D
util/scripts/install-deps-ubuntu.sh
#apt-get install xorg-dev libglu1-mesa-dev libgl1-mesa-glx libglew-dev libglfw3-dev libjsoncpp-dev libeigen3-dev libpng16-dev libjpeg-dev python-dev python3-dev python-tk python3-tk apt-get install libpng-dev pybind11-dev
mkdir build
cd build
cmake ../src
make -j

# Install Python binding module 

cd util/scripts
./install.sh
```


### 2.3 Docker 이용 설치

```
docker pull adioshun/ubuntu16:Open3D
docker run -it -p 7777:8888 --name "Open3D" adioshun/ubuntu16:Open3D /bin/bash
```


## 3. ROS 연동 

[Open3D on ROS scripts](https://github.com/karaage0703/open3d_ros)

[How to set up Open3D and ROS linkage by "open3d_ros](https://karaage.hatenadiary.jp/entry/2018/03/12/073000): 중간 부분 부터 



---
# [References]

~~[Studying Open3D (first time)](http://robonchu.hatenablog.com/entry/2018/02/24/200635)~~
~~[Study of Open3D (second time)](http://robonchu.hatenablog.com/entry/2018/02/25/121024)~~
[Studying Oepn3D (third time)](http://robonchu.hatenablog.com/entry/2018/02/25/200510) : 추후 다시 살펴 보기 
