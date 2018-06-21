# Open 3D 

## 2. 설치 

### 2.1 [pip](https://pypi.org/project/open3d-python/) 설치 

ubnutu16(0)

```
apt-get install libglu1-mesa-dev libgl1-mesa-glx libglew-dev libglfw3-dev libjsoncpp-dev libeigen3-dev libpng16-dev libjpeg-dev 
apt-get install python-dev python3-dev python-tk python3-tk 



#apt-get install xorg-dev libglu1-mesa-dev libgl1-mesa-glx libglew-dev libglfw3-dev libjsoncpp-dev libeigen3-dev libpng16-dev libjpeg-dev 
apt-get install libpng12-dev 

apt-get pybind11-dev
## pybind11-dev
~/Open3D/src/External/pybind11/build
cmake ..
sudo make install 


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


## 4. VS 연동 

http://robotslam.blogspot.com/2018/03/open3d-modern-library-for-3d-data.html

---
# [References]

~~[Studying Open3D (first time)](http://robonchu.hatenablog.com/entry/2018/02/24/200635)~~
~~[Study of Open3D (second time)](http://robonchu.hatenablog.com/entry/2018/02/25/121024)~~
[Studying Oepn3D (third time)](http://robonchu.hatenablog.com/entry/2018/02/25/200510) : 추후 다시 살펴 보기 
