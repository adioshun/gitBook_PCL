# [Using a matrix to transform a point cloud](http://pointclouds.org/documentation/tutorials/matrix_transform.php#matrix-transform)

> [Jupyter](http://nbviewer.jupyter.org/github/adioshun/gitBook_PCL/blob/master/basicusage/%20Using%20a%20matrix%20to%20transform%20a%20point%20cloud.ipynb), [python-ros](https://github.com/adioshun/gitBook_PCL/blob/master/basicusage/icp.py)

In this tutorial we will learn how to transform a point cloud using a 4x4 matrix. We will apply a rotation and a translation to a loaded point cloud and display then result.

This program is able to load one PCD or PLY file; apply a matrix transformation on it and display the original and transformed point cloud.


```cpp

/* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
  */

```

### 첫번쨰 방법 

```cpp

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


---

![image](https://user-images.githubusercontent.com/17797922/47072738-a2b80700-d231-11e8-8ff8-67aa8f23cad3.png)
- w == 1 이면, 벡터 (x,y,z,1) 은 공간에서의 위치 입니다.
- w == 0 이면, 벡터 (x,y,z,0) 은 방향입니다.

- The first 3 rows and columns (top left) components are the rotation matrix. 
- The first 3 rows of the last column is the translation.

[예제-평행 이동]
![image](https://user-images.githubusercontent.com/17797922/47073043-53260b00-d232-11e8-854b-26a675033d54.png)



---

## 참고 자료 

### Rotation Matrix를 목적 

물체의 회전을 표현 하기 위한 방법 
- Rotation matrix 
- Euler Angles 


#### 로봇 공학과 Rotation matrix 

내부 모델 : 물체를 움직이기 위한 모델, 
- Kinematics 내부모델 : 관절의 회전(Joint Space)과 주먹 끝(Task Space)의 움직임의 관계를 이해하는 것
- Dynamic 내부모델 : 
* 둘의 차이는 질량을 고려 하는냐 안 하느냐 

> Kinematics를 배우면 로봇을 원하는 궤적을 따라 움직이도록 할 수 있다

![image](https://user-images.githubusercontent.com/17797922/47067342-076c6500-d224-11e8-8242-343a24de2600.png)


관절의 종류 
- Prismatic : 1DoF, 직선, 미끄럼 관절 (eg, 인형 뽑기) 
- Revolute : 1DoF, 회전 관절 
- Universal = 2 x Revolution
- Spherical = 3 x Revolution 

관절별 내부 모델 
- Prismatic : 3V,2V,-5V = X,Y,Z (eg. 인형뽑기, 가로로 3초, 세로로 2초, 아래로 5초가면 집게의 위치)
- Revolute : 각도를 다룸 , $$(\theta_1,\theta_2,\theta_3) = (x,y,z) $$
    - x = L_1 cos\theta + L_2 cos\..............
    - y = ...................
    - z = z

> Revolute의 관계식 cos/sin을 통해 구하는것은 복잡하고 어렵기 때문에 이를 쉽게 하는것이 **Rotation Matrix**이다. 
> 각 관절에서의 Transformation Matrix를 구해서 이를 모두 곱하면 (x,y,z)가 나옴 
> Transformation Matrix = Rotation Matrix + 길이(관절과 관절 사이)정보 


### Rotation Matrix 개요 

- 3x3 행렬 
- det(R) = +1 (*determinant)
    - Identity Matrix도 det(R) = +1


   
#### Rotation Matrix 유도 방법 

- 삼각형의 합동 이용 
- 벡터의 내적을 이용
- 선형 변환을 이용하는 방법 
- etc. 

### Transformation Matrix 개요 

간단한 2차식에서의 예 : https://youtu.be/Y0Vjc9sOq-M?t=620



$$
- V_a = R_{ab}V_b 
    - R_{ab} = a프레임에서 바로본 b프레임 = [x_{a에서 바로본 b의}, y_{a에서 바로본 b의}, z_{a에서 바로본 b의}]
- $$R_{ab} * R_{bc} = R_{ac}$$ --> a에서b를 바라보고, b에서c를 바라본것은, 결과적으로 a에서 c를 바라본것과 같다. 
$$

> 공식 유도 : https://youtu.be/MgB0oiIhoTQ?t=238



### Euler angle (오일러 각) 회전


모든 Rotation Matrix는 3번의 회전(a,b,c)으로 표현 가능 
- roll, pitch, yaw

오일러 글을 알면 각각을 곱해서 Rotation matrix를 구할수 있음


3차원 공간에서 
- Particle은 (x,y,z) 3 DOF를 가짐 
- RIgid Body는 (x,y,z,a,b,c) 6 DoF를 가짐 
    - a(alpha) : x측 회전 roll
    - b(beta) : y측 회전 pitch
    - c(gamma) : z측 회전 yaw


### Space & Body fixed movement

Space fixed movement : 회전하는 축이 움직이지 않는것 

Body fixed movement : 이동한 다음에 움직이는 것, 로봇 세상에서 대부분의 움직임 


### Rotation matrix와 SO(3)그룹 

Roation matrix의 조건에 대한 설명 https://youtu.be/USbu0vIc8VQ

> 그룹 : element, operation (element가 operation후에도 element에 속하면 그룹이라 함) 

직교?





---
[회전변환행렬](http://t-robotics.blogspot.com/2013/07/rotation-matrix.html#.W8ZOAWgzYuV)
[저랑 야바위 한판 하실래요? Rotation Matrix](http://t-robotics.blogspot.com/2013/07/rotation-matrix.html#.W8ZOAWgzYuV): T-Robotics블로그 
선형 변환을 이용한 [회전 행렬(Rotation matrix)의 유도](https://o-tantk.github.io/posts/derive-rotation-matrix/)
[Rotation Matrix](http://dolphin.ivyro.net/file/mathematics/tutorial08.html)
[Youtube] Rotation matrix 회전행렬 : [[1강]](https://youtu.be/2oKGg_cYE70), [[2강]](), [[3강]]()
[오일러각과 회전행렬(Euler Angles and Rotation Matrix) ](http://blog.daum.net/pg365/170)
[Tutorial 3 : 행렬(매트릭스)](http://www.opengl-tutorial.org/kr/beginners-tutorials/tutorial-3-matrices/)