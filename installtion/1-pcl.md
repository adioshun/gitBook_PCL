# 1. PCL for C++ 설치 

## A. Source 설치 

```python 
apt-get update -qq && apt-get install -y --no-install-recommends \
      make cmake build-essential git \
      libeigen3-dev \
      libflann-dev \
      libusb-1.0-0-dev \
      libvtk6-qt-dev \
      libpcap-dev \
      libboost-all-dev \
      libproj-dev \
      && rm -rf /var/lib/apt/lists/*
 
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
tar zvfx pcl-1.8.1.tar.gz

cd pcl-1.8.1
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make -j2 install
 
# or (확인 안됨)
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
mkdir build && cd build
cmake ..
make
#checkinstall #apt-get install checkinstall
make install 

# Error 
ln -s /usr/lib/x86_64-linux-gnu/libproj.so.<버젼> /usr/lib/x86_64-linux-gnu/libproj.so # make[2]: *** No rule to make target '/usr/lib/x86_64-linux-gnu/libproj.so',

```







## B. apt 설치 

> PCL설치는 되지만 pcl-python과의 연동이 안되는듯 

```python
add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
apt-get update

# for Ubuntu 14.04
apt-get install libpcl-all

# for Ubuntu 16.04
sudo apt install libpcl-dev  # depends: libvtk6-dev
```

|Error Code | Solution|
|-|-|
|add-apt-repository command not found | `apt-get install software-properties-common python-software-properties`|
|Unable to locate package libpcl-all|apt-get install libpcl1|
|pip10, ImportError: cannot import name main|Downgrade: `python2 -m pip install --user --upgrade pip==9.0.3`|
|fatal error: 'pcl/point_cloud.h' file not found |seems due to that I don’t have an ROS environment.|



## 설치 테스트 

```
cd ~ && mkdir pcl-test && cd pcl-test

wget https://gist.githubusercontent.com/adioshun/319d6a1326d33fa42cdd56833c3ef560/raw/e10d3502ddcd871f9d6b7b57d176b17d52de5571/CMakeLists.txt
wget https://gist.githubusercontent.com/adioshun/319d6a1326d33fa42cdd56833c3ef560/raw/e10d3502ddcd871f9d6b7b57d176b17d52de5571/main.cpp

mkdir build && cd build
cmake .. && make && ./pcl-test
```






---





# 2. PCL for Python 설치 

- [공식 홈페이지](http://strawlab.github.io/python-pcl/), [example](https://github.com/strawlab/python-pcl/tree/master/examples)

- conda를 이용한 설치 방법 추천 

## 2.1 pip 설치 (ubuntu 14용)

```python 
apt-get install build-essential
apt-get install -y python3-pip git python-dev
apt install pkg-config


pip3 install numpy cython 

# ubunutu 14??
pip3 install git+https://github.com/strawlab/python-pcl
#pip install git+https://github.com/strawlab/python-pcl.git#egg=pcl

# ubuntu 16??
git clone https://github.com/strawlab/python-pcl.git
cd python-pcl
python3 setup.py build
python3 setup.py install
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





