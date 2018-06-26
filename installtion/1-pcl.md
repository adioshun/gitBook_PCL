# PCL

[공식 홈페이지](http://strawlab.github.io/python-pcl/), [example](https://github.com/strawlab/python-pcl/tree/master/examples) [Qna](https://www.bountysource.com/teams/strawlab/issues?tracker_ids=658709)

[원하는 QT 버전에 맞게 PCL 설치하기](http://jinyongjeong.github.io/2017/01/09/pcl_install_with_qt5/): 2017


## 1. 패키지 설치 

Must Install Java : sudo add-apt-repository -y ppa:webupd8team/java && sudo apt update && sudo apt -y install oracle-java8-installer

For Ubuntu 16 :  PCL-1.8.1 supports Ubuntu > 16.04. Try to upgrade your O.S



#### Ubuntu 16 (python2 실패, python3에서 성공)

> ubuntu 17 (X)


```python
sudo apt-get update && sudo apt-get install -y software-properties-common git
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y && sudo apt-get update 


apt-get install -y libpcl-dev #ubuntu 16
#apt-get -s install libpcl-dev # PCL-1.8.1 supports Ubuntu > 16.04. Try to upgrade your O.S

apt-get install -y python3-pip python3-dev python-pip python-dev git

pip3 install cython==0.25.2 && pip3 install numpy
pip install cython==0.25.2 && pip install numpy



pip3 install git+https://github.com/strawlab/python-pcl
pip install git+https://github.com/strawlab/python-pcl

#git clone https://github.com/strawlab/python-pcl.git
#cd python-pcl
#python3 setup.py build
#python3 setup.py install


add-apt-repository --remove ppa:v-launchpad-jochen-sprickerhof-de/pcl -y 

# docker pull adioshun/pcls:u16python3_pcl

```

#### Ubuntu 14 (python2,3에서 성공)

```python
apt-get update && apt-get install -y software-properties-common git && add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y 

apt-get update && apt-get install -y libpcl-all #ubnutu 14

apt-get install -y python-pip python-dev git && pip install cython==0.25.2 && pip install numpy

pip install git+https://github.com/strawlab/python-pcl
#git clone https://github.com/strawlab/python-pcl.git
#cd python-pcl
#python3 setup.py build
#python3 setup.py install


add-apt-repository -remove ppa:v-launchpad-jochen-sprickerhof-de/pcl -y 

# docker pull adioshun/pcls:u14python23_pcl

```



## 2. [소스설치](https://askubuntu.com/questions/916260/how-to-install-point-cloud-library-v1-8-pcl-1-8-0-on-ubuntu-16-04-2-lts-for)


### 사전 설치

```python
apt-get install software-properties-common -y

#Install oracle-java8-jdk:
sudo add-apt-repository -y ppa:webupd8team/java && sudo apt update && sudo apt -y install oracle-java8-installer
#add-apt-repository -y ppa:webupd8team/java && apt update && apt -y install oracle-java8-installer

#Install universal pre-requisites:

sudo apt -y install g++ cmake cmake-gui doxygen mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev
sudo apt -y install git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer
sudo apt -y install phonon-backend-vlc graphviz mono-complete qt-sdk libflann-dev  
#apt -y install g++ cmake cmake-gui doxygen mpi-default-dev openmpi-bin openmpi-common libusb-1.0-0-dev libqhull* libusb-dev libgtest-dev git-core freeglut3-dev pkg-config build-essential libxmu-dev libxi-dev libphonon-dev libphonon-dev phonon-backend-gstreamer phonon-backend-vlc graphviz mono-complete qt-sdk libflann-dev  
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


### 설치 테스트 

```python
cd ~ && mkdir pcl-test && cd pcl-test

wget https://gist.githubusercontent.com/adioshun/319d6a1326d33fa42cdd56833c3ef560/raw/e10d3502ddcd871f9d6b7b57d176b17d52de5571/CMakeLists.txt 
wget https://gist.githubusercontent.com/adioshun/319d6a1326d33fa42cdd56833c3ef560/raw/e10d3502ddcd871f9d6b7b57d176b17d52de5571/main.cpp

mkdir build && cd build
cmake .. && make && ./pcl-test

# Error 
ln -s /usr/lib/x86_64-linux-gnu/libproj.so.<버젼> /usr/lib/x86_64-linux-gnu/libproj.so # make[2]: *** No rule to make target '/usr/lib/x86_64-linux-gnu/libproj.so',
```




|에러코드|해결책|원인|
|-|-|-|
|fatal error: pcl/features/cppf.h: No such file or directory|`sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl;sudo apt-get update;sudo apt-get upgrade libpcl-features-dev libpcl-io-1.7 libpcl-io-1.7-dev`|ubnutu 14 떄문인|
|'pcl_2d-1.8', required by 'pcl_features-1.8'| line 10 in /usr/local/lib/pkgconfig/pcl_features-1.8.pc <br>`Requires: pcl_common-1.8 pcl_search-1.8 pcl_kdtree-1.8 pcl_octree-1.8 pcl_filters-1.8 #pcl_2d-1.8`|[출처](https://github.com/strawlab/python-pcl/issues/97)|


### 2.2 conda 설치

```
conda config --add channels conda-forge
conda install -c sirokujira python-pcl #v0.3
#conda install -c https://conda.anaconda.org/ccordoba12 python-pcl  #v0.2
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
rosdep install --from-paths ./ --ignore-src --rosdistro $ROS_DISTRO

## Register {launch} with ROS via catkin_make.
cd ~/catkin_ws
catkin_make 
source ./devel/setup.sh

## Check
rospack profile
rospack list | grep {velodyne}
rospack find {}
```







---
## 삽질의 흔적들 


#### A. Source 설치 

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


## B. apt 설치 

> PCL설치는 되지만 pcl-python과 충돌 [ref](https://recordnotfound.com/python-pcl-strawlab-23316)

sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl -y

sudo add-apt-repository -remove ppa:v-launchpad-jochen-sprickerhof-de/pcl -y #설치 후 제거 

가능 버젼 확인 : apt-cache search libpcl-dev
설치 버젼 확인 : apt-get -s install libpcl-dev


PCL for Ubuntu 14.04 : apt-get install libpcl-all

PCL 1.7.2 for Ubuntu 16.04 : ~~sudo apt install libpcl-dev~~   #설치는 되나 hello 테스트 실패 

PCL 1.8.0 and Ubuntu 16.04 : ~~wget https://www.dropbox.com/s/9llzm20pc4opdn9/PCL-1.8.0-Linux.deb && dpkg -i PCL-1.8.0-Linux.deb~~  #설치는되나 hello 테스트 에러 


```
# PCL 1.7.2 and Ubuntu16.04 (use Debian package)
## PCL설치 
sudo apt-get update -y
sudo apt-get install build-essential devscripts
dget -u https://launchpad.net/ubuntu/+archive/primary/+files/pcl_1.7.2-14ubuntu1.16.04.1.dsc
cd pcl-1.7.2
sudo dpkg-buildpackage -r -uc -b
sudo dpkg -i pcl_*.deb
```




|Error Code | Solution|
|-|-|
|add-apt-repository command not found | `apt-get install software-properties-common python-software-properties`|
|Unable to locate package libpcl-all|apt-get install libpcl1|
|pip10, ImportError: cannot import name main|Downgrade: `python2 -m pip install --user --upgrade pip==9.0.3`|
|fatal error: 'pcl/point_cloud.h' file not found |seems due to that I don’t have an ROS environment.|










---










