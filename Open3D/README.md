# Open3D 

## 1. 개요 

관련 링크 : [홈페이지](http://www.open3d.org/docs/getting_started.html#compiling-from-source), [깃허브](https://github.com/IntelVCL/Open3D), [논문](http://www.open3d.org/paper.pdf)

튜토리얼: [Jupyter](https://nbviewer.jupyter.org/url/lang.sist.chukyo-u.ac.jp/Classes/Open3D/Open3D.ipynb), [SSII2018_Tutorial_Open3D](https://github.com/sakizuki/SSII2018_Tutorial_Open3D)


기존 PCL의 제약 
- Only C ++ can be used (C ++ weak ...)
- I have to build every bit for a bit of correction
- I need to write a lot of rows

Open3D 장점 
- Can use Python
- It is multiplied by a small number of lines (it is 1/5 of PCL)
- Debugging with Jupyter Notebook

2018.01. release

- Website: open3d.org
- Code: github.com/IntelVCL/Open3D
- Document: open3d.org/docs
- Getting started: open3d.org/docs/getting_started.html
- License: The MIT license

개발
- [Qianyi Zhou](http://qianyi.info/) 
- [Jaesik Park](http://jaesik.info/)

동작 환경 

- Windows, Visual Studio 2015 update 3+, CMake 3.0+
- OS X, Clang included in the latest Xcode, CMake 3.0+
- Ubuntu 16.04, native gcc (4.8+ or 5.x), CMake 3.0+

언어 
- For C++ code, it is recommended to use C++11 features. However, do not use C++14 or C++17
- For Python code, make sure it runs on both Python 2.7 and Python 3.x.

---

## 2. 설치 

### 2.1 [pip](https://pypi.org/project/open3d-python/) 설치 

ubnutu16(0)

```python

sudo apt-get install -y libpng16-tools # libpng16-dev #libpng-dev 
sudo apt-get install -y libglu1-mesa-dev libgl1-mesa-glx libglew-dev libglfw3-dev libjsoncpp-dev libeigen3-dev libjpeg-dev python-dev python3-dev python-tk python3-tk

sudo apt-get install -y python3-pip 
pip3 install open3d-python

#sudo apt-get install pybind11-dev # ~/Open3D/src/External/pybind11/build 
#sudo apt-get install xorg-dev 
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





## 4. VS 연동 

http://robotslam.blogspot.com/2018/03/open3d-modern-library-for-3d-data.html

---
# [References]

~~[Studying Open3D (first time)](http://robonchu.hatenablog.com/entry/2018/02/24/200635)~~
~~[Study of Open3D (second time)](http://robonchu.hatenablog.com/entry/2018/02/25/121024)~~
[Studying Oepn3D (third time)](http://robonchu.hatenablog.com/entry/2018/02/25/200510) : 추후 다시 살펴 보기 

