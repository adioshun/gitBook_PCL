# Open 3D Installation [[참고]](http://www.open3d.org/docs/getting_started.html#compiling-from-source)

Ubuntu 16, python3, pip

```python
apt-get install cmake 
pip3 install numpy

cd ~
git clone https://github.com/IntelVCL/Open3D

# install the dependencies and use CMake to build the project
util/scripts/install-deps-ubuntu.sh
mkdir build
cd build
cmake ../src
make -j

# Install Python binding module 

cd util/scripts
./install.sh

```

설치 테스트 (python3) : `import open3d`