# [Using a matrix to transform a point cloud](http://pointclouds.org/documentation/tutorials/matrix_transform.php#matrix-transform)

> 참고 코드 : [Jupyter](./%20Using%20a%20matrix%20to%20transform%20a%20point%20cloud.ipynb), [python-ros](./icp.py)


In this tutorial we will learn how to transform a point cloud using a 4x4 matrix. We will apply a rotation and a translation to a loaded point cloud and display then result.

This program is able to load one PCD or PLY file; apply a matrix transformation on it and display the original and transformed point cloud.


```cpp

/* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)


  */

```

### 첫번쨰 방법 

```cpp
  /*
  METHOD #1: Using a Matrix4f
  This is the "manual" method, perfect to understand but error prone !
  */
  Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

  // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
  float theta = M_PI/4; // The angle of rotation in radians
  transform_1 (0,0) = cos (theta);
  transform_1 (0,1) = -sin(theta);
  transform_1 (1,0) = sin (theta);
  transform_1 (1,1) = cos (theta);
  //    (row, column)

  // Define a translation of 2.5 meters on the x axis.
  transform_1 (0,3) = 2.5;

  // Print the transformation
  printf ("Method #1: using a Matrix4f\n");
  std::cout << transform_1 << std::endl;
```

### 두번째 방법 

```cpp
  /*  METHOD #2: Using a Affine3f
    This method is easier and less error prone
  */
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << 2.5, 0.0, 0.0;

  // The same rotation matrix as before; theta radians around Z axis
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  // Print the transformation
  printf ("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;
```



### 세번쨰 방법 

`rosrun tf static_transform_publisher 0 0 0 0 0 0 velodyne velodyne_201 10`

static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
- Publish a static coordinate transform to tf using an x/y/z offset in meters and yaw/pitch/roll in radians. 
- (yaw is rotation about Z, pitch is rotation about Y, and roll is rotation about X). 
- The period, in milliseconds, specifies how often to send a transform. 100ms (10hz) is a good value.

static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms
- Publish a static coordinate transform to tf using an x/y/z offset in meters and quaternion. 
- The period, in milliseconds, specifies how often to send a transform. 100ms (10hz) is a good value.


- Rotation matrix Vs. Euler angle 변환 [코드](https://www.learnopencv.com/rotation-matrix-to-euler-angles/), [웹사이트](https://www.andre-gaschler.com/rotationconverter/) ,[시각화검증](http://danceswithcode.net/engineeringnotes/rotations_in_3d/demo3D/rotations_in_3d_tool.html)

