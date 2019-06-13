

## 1. 개요 

> [Ronny Restrepo](http://ronny.rest/tutorials/module/pointclouds_01/point_cloud_data/)


## 2. Creating Birdseye View of Point Cloud Data

> 참고 : Height의 Level별 값 추출 (Height as Channels), [Creating Height Slices of Lidar Data](http://ronny.rest/blog/post_2017_03_27_lidar_height_slices/)

In order to create a birds eye view image, the relevant axes from the point cloud data will be the x and y axes.

![](http://i.imgur.com/cHsb48Y.png)

조심해야 할점 
- the x, and y axes mean the opposite thing.
- The x, and y axes point in the opposite direction.
- You have to shift the values across so that (0,0) is the smallest possible value in the image.


|- [Creating Birdseye View of Point Cloud Data 코드 및 설명(python)](http://ronny.rest/blog/post_2017_03_26_lidar_birds_eye/), [gist](https://gist.github.com/adioshun/12873804f472080c612e506310674797)|
|-|

> [참고] cpp로 작성한 코드 : [mjshiggins's github](https://github.com/mjshiggins/ros-examples)



## 3. Creating 360 degree Panoramic Views

- Project the `points in 3D` space into `cylindrical surface`

- LiDAR센서의 특징에 따라 설정값이 달라 진다. 
    - `h_res`: Horizontal resolution
    - `v_res`: vertical resolution

```python
# KTTI dataset = Velodyne HDL 64E 
## A vertical field of view of 26.9 degrees, at a resolution of 0.4 degree intervals. The vertical field of view is broken up into +2 degrees above the sensor, and -24.9 degrees below the sensor.
## A horizontal field of view of 360 degrees, at a resolution of 0.08 - 0.35 (depending on the rotation rate)
## Rotation rate can be selected to be betwen 5-20Hz.
## http://velodynelidar.com/docs/datasheet/63-9194%20Rev-E_HDL-64E_S3_Spec%20Sheet_Web.pdf

# Resolution and Field of View of LIDAR sensor
h_res = 0.35         # horizontal resolution, assuming rate of 20Hz is used 
v_res = 0.4          # vertical res
v_fov = (-24.9, 2.0) # Field of view (-ve, +ve) along vertical axis
v_fov_total = -v_fov[0] + v_fov[1] 
```

|- [Creating 360 degree Panoramic Views코드 및 설명(matplotlib)](http://ronny.rest/blog/post_2017_03_25_lidar_to_2d/)<br>- [Creating 360 degree Panoramic Views코드 및 설명(numpy)](http://ronny.rest/blog/post_2017_04_03_point_cloud_panorama/)|
|-|




---
# LAS 포맷

- [정의](https://www.asprs.org/committee-general/laser-las-file-format-exchange-activities.html)
- the most commonly used binary point data exchange format
- The LAS file format is a public file format for the interchange of 3-dimensional point cloud data data between data users
- supports the exchange of any 3-dimensional x,y,z tuplet
- [What Lidar processing tools are available in Python?](https://gis.stackexchange.com/questions/88322/what-lidar-processing-tools-are-available-in-python)
- 항공용 Lidar 데이터에서 주로 사용하는지 확인 필요

# pcap to LAS

> LAS format - the most commonly used binary point data exchange format

- (1) I suggest you create many many txt files containing the points in xyz or xyzi a layout (i = intensity). Maybe one file per driveline. (벨로뷰의 csv저장 기능 활용)
- (2) Then you convert each of them to the LAZ format with txt2las using the '-parse xyz' or the '-parse xyzi' option.
- (3) Then you tile the resulting LAZ files into buffered tiles that have less than, say 10 million, points per tile with lastile. `lastile -v -i drivelines/*.laz -merged -tile_size 100 -buffer 5 -odir raw_tiles -o gmu.laz`
- (4) Then you refine tiles to 10 million points per tile if needed
`lastile -v -i raw_tiles/*.laz -refine_tiles 10000000 -cores 4`
- (5) And then - starting from the tiles in the tiles_raw folder - you run one of those many tile-based batch processing pipelines for ground classification and DTM generation described previously in numerous tutorials, videos, and forum entries.









