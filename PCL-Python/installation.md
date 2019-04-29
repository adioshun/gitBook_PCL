# PCL-Python 


설치 요구 사항 
- PointCloudLibrary 1.6.x 1.7.x 1.8.x 1.9.x
    - ubuntu18에는 기본 PCL 포함 -> `apt-get install libpcl-dev`만 하면 됨 -> python-pcl설치 
    - ROS 설치시 PCL - V1.7 포함  
- NumPy 1.9, 1.10, 1.11, 1.12, 1.13, . . .
- Cython >=0.25.2


## 1. 소스 설치 

```python

pip3 install cython==0.25.2 && pip3 install numpy
git clone https://github.com/strawlab/python-pcl.git
cd python-pcl
python --version
sudo python3 setup.py build_ext -i
sudo python3 setup.py install

#pip3 install git+https://github.com/strawlab/python-pcl
```

불필요한 파일 / 정보 삭제 

```
add-apt-repository --remove ppa:v-launchpad-jochen-sprickerhof-de/pcl -y 
sudo rm -rf /var/lib/apt/lists/*
```

[공식 홈페이지](http://strawlab.github.io/python-pcl/), [example](https://github.com/strawlab/python-pcl/tree/master/examples) [Qna](https://www.bountysource.com/teams/strawlab/issues?tracker_ids=658709)


## 2. pip 설치 

```
pip install python-pcl
#pip install python-pcl -vvvv

#삭제 
pip uninstall python-pcl

#업그레이드 
pip install -U python-pcl

#재설치 
$ pip uninstall python-pcl
$ pip install python-pcl --no-cache-dir
# You need to reinstall python-pcl when you want to upgrade PointCloudLibrary

```

## 3. conda 설치

```
conda config --add channels conda-forge
conda install -c sirokujira python-pcl #v0.3
#conda install -c https://conda.anaconda.org/ccordoba12 python-pcl  #v0.2
```



















