# Open 3D Installation [\[참고\]](http://www.open3d.org/docs/getting_started.html#compiling-from-source)

## 1. 소스코드 이용 설치

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

설치 테스트 \(python3\) : `import open3d`

## 2. pip 설치 

```
pip install open3d-python
```

## 3. Docker 이용 설치

```
docker pull adioshun/ubuntu16:Open3D
docker run -it -p 7777:8888 --name "Open3D" adioshun/ubuntu16:Open3D /bin/bash
```



