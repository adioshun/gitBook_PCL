# PCL



## 2. Tools

###### Official Tools of PCL


- pcl_viewer: a quick way for visualizing PCD (Point Cloud Data) files

- pcd2ply: converts PCD (Point Cloud Data) files to the PLY format

- octree_viewer: allows the visualization of octrees


###### PCl Tools

`sudo apt-get install pcl-tools` [[github]](https://github.com/ryanfb/pcl-tools)
- pcd2ply - convert PCD files to PLY files
- ply2pcd - convert PLY files to PCD files
- statistical_removal - statistical outliers removal
- pcd_viewer - PCL PCD viewer with added 
- pcdnormal2ply - convert PCD files with normals to PLY files
- normal_estimation_omp - estimate normals in a PCD using PCL's OpenMP implementation





> [How to extract depth information from the 3D point cloud data?](https://stackoverflow.com/questions/42430479/how-to-extract-depth-information-from-the-3d-point-cloud-data) : 포인트클라우드에서 depth 계산법 


---

# PCL-Python

설치 요구 사항

* PointCloudLibrary 1.6.x 1.7.x 1.8.x 1.9.x
  * ubuntu18에는 기본 PCL 포함 -&gt; `apt-get install libpcl-dev`만 하면 됨 -&gt; python-pcl설치 
  * ROS 설치시 PCL - V1.7 포함  
* NumPy 1.9, 1.10, 1.11, 1.12, 1.13, . . .
* Cython &gt;=0.25.2

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





