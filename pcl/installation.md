# PCL & python-pcl

## 1. python-pcl 설치 

```python
"""
python3.5에서 안정되게 동작 하는것 같음 (PCL 1.7.2, Cython 0.25.2)
"""
sudo apt-get install -y python3-pip python3-dev

pip3 install cython==0.25.2 && pip3 install numpy
git clone https://github.com/strawlab/python-pcl.git
cd python-pcl
python --version
sudo python setup.py build_ext -i
sudo python setup.py install

#pip3 install git+https://github.com/strawlab/python-pcl
```

불필요한 파일 / 정보 삭제 

```
add-apt-repository --remove ppa:v-launchpad-jochen-sprickerhof-de/pcl -y 
sudo rm -rf /var/lib/apt/lists/*
```

[공식 홈페이지](http://strawlab.github.io/python-pcl/), [example](https://github.com/strawlab/python-pcl/tree/master/examples) [Qna](https://www.bountysource.com/teams/strawlab/issues?tracker_ids=658709)




### conda 설치

```
conda config --add channels conda-forge
conda install -c sirokujira python-pcl #v0.3
#conda install -c https://conda.anaconda.org/ccordoba12 python-pcl  #v0.2
```


---




## 2. pcl LIB. 설치 

```python



sudo apt-get update && sudo apt-get install -y software-properties-common git
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y && sudo apt-get update

sudo apt-get install -y libpcl-dev #ubuntu 16 (libpcl-dev 1.7.2)
sudo apt-get install -y libpcl-all #ubnutu 14

"""
### 설치 테스트
cd ~ && mkdir pcl-test && cd pcl-test

wget https://gist.githubusercontent.com/adioshun/319d6a1326d33fa42cdd56833c3ef560/raw/e10d3502ddcd871f9d6b7b57d176b17d52de5571/CMakeLists.txt 
wget https://gist.githubusercontent.com/adioshun/319d6a1326d33fa42cdd56833c3ef560/raw/e10d3502ddcd871f9d6b7b57d176b17d52de5571/main.cpp
mkdir build && cd build
cmake .. && make && ./pcl-test
# Error
sudo ln -s /usr/lib/x86_64-linux-gnu/libproj.so.9.1.0 /usr/lib/x86_64-linux-gnu/libproj.so
sudo ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/libvtkproj4.so
"""
```







## [소스설치](https://askubuntu.com/questions/916260/how-to-install-point-cloud-library-v1-8-pcl-1-8-0-on-ubuntu-16-04-2-lts-for)


### 사전 설치

```python
apt-get install software-properties-common -y

#Install oracle-java8-jdk:
sudo add-apt-repository -y ppa:webupd8team/java && sudo apt update && sudo apt -y install oracle-java8-installer
#sudo add-apt-repository -y ppa:webupd8team/java && apt update && apt -y install oracle-java8-installer

#Install universal pre-requisites:

sudo apt -y install g++ cmake cmake-gui doxygen mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev
sudo apt -y install git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer
sudo apt -y install phonon-backend-vlc graphviz mono-complete qt-sdk libflann-dev  
#sudo apt -y install g++ cmake cmake-gui doxygen mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer phonon-backend-vlc graphviz mono-complete qt-sdk libflann-dev  
```   

### For PCL v1.8, Ubuntu 16.04.2 input the following:

```python
sudo apt -y install libflann1.8 libboost1.58-all-dev cmake #libeigen3-dev (아래에서 dpkg로 설치)

cd ~/Downloads
wget http://launchpadlibrarian.net/209530212/libeigen3-dev_3.2.5-4_all.deb
sudo dpkg -i libeigen3-dev_3.2.5-4_all.deb
sudo apt-mark hold libeigen3-dev

wget http://www.vtk.org/files/release/7.1/VTK-7.1.0.tar.gz
tar -xf VTK-7.1.0.tar.gz
cd VTK-7.1.0 && mkdir build && cd build
cmake ..
make                                                                   
sudo make install

cd ~/Downloads
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.0.tar.gz
tar -xf pcl-1.8.0.tar.gz
cd pcl-pcl-1.8.0 && mkdir build && cd build
cmake ..
make
sudo make install

cd ~/Downloads
rm libeigen3-dev_3.2.5-4_all.deb VTK-7.1.0.tar.gz pcl-1.8.0.tar.gz
sudo rm -r VTK-7.1.0 pcl-pcl-1.8.0

# docker pull adioshun/pcls:pcl_only

```

### For PCL v1.8.1, Ubuntu 17.10 input the following:

```python
sudo apt -y install libflann1.9 libboost1.63-all-dev libeigen3-dev

cd ~/Downloads
wget http://www.vtk.org/files/release/8.0/VTK-8.0.1.tar.gz
tar -xf VTK-8.0.1.tar.gz
cd VTK-8.0.1 && mkdir build && cd build
cmake ..
make                                                                   
sudo make install

cd ~/Downloads
wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.8.1.tar.gz
tar -xf pcl-1.8.1.tar.gz
cd pcl-pcl-1.8.1 && mkdir build && cd build
cmake ..
make
sudo make install

cd ~/Downloads
rm VTK-8.0.1.tar.gz pcl-1.8.1.tar.gz
sudo rm -r VTK-8.0.1 pcl-pcl-1.8.1
```


## Github 소스 설치 

```python 
sudo apt-get update -qq && sudo apt-get install -y --no-install-recommends \
      make cmake build-essential git \
      libeigen3-dev \
      libflann-dev \
      libusb-1.0-0-dev \
      libvtk6-qt-dev \
      libpcap-dev \
      libboost-all-dev \
      libproj-dev \
      && sudo rm -rf /var/lib/apt/lists/*

# ubuntu 16 (checked) 
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
```





|Error Code | Solution|
|-|-|
|add-apt-repository command not found | `apt-get install software-properties-common python-software-properties`|
|Unable to locate package libpcl-all|apt-get install libpcl1|
|pip10, ImportError: cannot import name main|Downgrade: `python2 -m pip install --user --upgrade pip==9.0.3`|
|fatal error: 'pcl/point_cloud.h' file not found |seems due to that I don’t have an ROS environment.|
|Python locale error: unsupported locale setting|$ export LC_ALL="en_US.UTF-8"<br>$ export LC_CTYPE="en_US.UTF-8"<br>$ sudo dpkg-reconfigure locales|
|fatal error: pcl/features/cppf.h: No such file or directory|`sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl;sudo apt-get update;sudo apt-get upgrade libpcl-features-dev libpcl-io-1.7 libpcl-io-1.7-dev`|ubnutu 14 떄문인|
|'pcl_2d-1.8', required by 'pcl_features-1.8'| line 10 in /usr/local/lib/pkgconfig/pcl_features-1.8.pc <br>`Requires: pcl_common-1.8 pcl_search-1.8 pcl_kdtree-1.8 pcl_octree-1.8 pcl_filters-1.8 #pcl_2d-1.8`|[출처](https://github.com/strawlab/python-pcl/issues/97)|















---

## [Docker](https://hub.docker.com/r/atatb/ubuntu-16-pcl-1.8.0/~/dockerfile/)

```

FROMFROM ubuntu: ubuntu:16.04

MAINTAINERMAINTAINER Kenji Nomura <atatb23@gmail.com>

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -qq && apt-get install -y --no-install-recommends \
make cmake build-essential git \
libeigen3-dev \
libflann-dev \
libusb-1.0-0-dev \
libvtk6-qt-dev \
libpcap-dev \
libboost-all-dev \
libproj-dev \
&& rm -rf /var/lib/apt/lists/*

RUN \
git config --global http.sslVerify false && \
git clone --branch pcl-1.8.1 --depth 1 https://github.com/PointCloudLibrary/pcl.git pcl-trunk && \
cd pcl-trunk && \
mkdir build && cd build && \
cmake -DCMAKE_BUILD_TYPE=Release .. && \
make -j 4 && make install && \
make clean

RUN ldconfig

```

[PCL docker with GPU feature enabled](https://hub.docker.com/r/youyue/pcl-docker/)

```
# pcl dockerfile
# Author: Airsquire You Yue
# Pull base image.
FROM nvidia/cuda:8.0-devel

# Install neccessary tools
RUN apt-get update
RUN apt-get install -y \
software-properties-common \
ca-certificates \
wget
RUN wget -O - http://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add -
RUN apt-add-repository "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-4.0 main"
RUN apt-get update
RUN apt-get install -y \
build-essential \
g++ \
python-dev \
autotools-dev \
libicu-dev \
libbz2-dev \
libboost-all-dev

RUN apt-get install -y \
mc \
lynx \
libqhull* \
pkg-config \
libxmu-dev \
libxi-dev \
--no-install-recommends --fix-missing

RUN apt-get install -y \
mesa-common-dev \
cmake \
git \
mercurial \
freeglut3-dev \
libflann-dev \
--no-install-recommends --fix-missing

RUN apt-get autoremove

# Install Eigen
RUN cd /opt && hg clone -r 3.2 https://bitbucket.org/eigen/eigen eigen
RUN mkdir -p /opt/eigen/build
RUN cd /opt/eigen/build && cmake ..
RUN cd /opt/eigen/build && make install

# Install VTK
RUN cd /opt && git clone git://vtk.org/VTK.git VTK
RUN cd /opt/VTK && git checkout tags/v8.0.0
RUN cd /opt/VTK && mkdir build
RUN cd /opt/VTK/build && cmake -DCMAKE_BUILD_TYPE:STRING=Release -D VTK_RENDERING_BACKEND=OpenGL ..
RUN cd /opt/VTK/build && make -j 32 && make install


# Install PCL
RUN cd /opt && git clone https://github.com/Airsquire/pcl pcl
RUN cd /opt/pcl && git checkout master
RUN mkdir -p /opt/pcl/build
RUN cd /opt/pcl/build && cmake -D WITH_CUDA=true -D BUILD_GPU=true -D BUILD_visualization=true -D BUILD_CUDA=true -D VTK_DIR=/opt/VTK/build -D BUILD_2d=true ..
RUN cd /opt/pcl/build && make -j 32 && make install
RUN cd /opt/pcl/build && make clean
```

> [Reason for building failure in docker hub
There is a size limitation for docker hub](https://stackoverflow.com/questions/31158913/is-there-any-limit-on-pull-number-in-docker-hub)

> Reason for not using CUDA 9.0+
The 'compute_20', 'sm_20', and 'sm_21' architectures are deprecated in CUDA 9.0. But PCL 1.8.1 is still using these architectures








