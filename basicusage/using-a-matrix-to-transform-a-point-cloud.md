# [Using a matrix to transform a point cloud](http://pointclouds.org/documentation/tutorials/matrix_transform.php#matrix-transform)

In this tutorial we will learn how to transform a point cloud using a 4x4 matrix. We will apply a rotation and a translation to a loaded point cloud and display then result.

This program is able to load one PCD or PLY file; apply a matrix transformation on it and display the original and transformed point cloud.













---

## 참고 자료 



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
- Revolute : 각도를 다룸 , $$(\theta_1, $$







---
[회전변환행렬](http://t-robotics.blogspot.com/2013/07/rotation-matrix.html#.W8ZOAWgzYuV): 위키피디
[저랑 야바위 한판 하실래요? Rotation Matrix](http://t-robotics.blogspot.com/2013/07/rotation-matrix.html#.W8ZOAWgzYuV): T-Robotics블로그 
[회전 행렬(Rotation matrix)의 유도](https://o-tantk.github.io/posts/derive-rotation-matrix/)
[Rotation Matrix](http://livingeasy.tistory.com/10)
[Youtube] Rotation matrix 회전행렬 : [[1강]](https://youtu.be/2oKGg_cYE70), [[2강]](), [[3강]]()