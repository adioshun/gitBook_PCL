# Random sample consensus algorithm (RANSAC)

![](https://i.imgur.com/E8Ynlcr.png)

UDACITY : https://classroom.udacity.com/courses/ud810/lessons/3189558841/concepts/31679389240923


```
"Random Sample Consensus: A Paradigm for Model Fitting with Application to Image Analysis and Automated Cartography", 1981 by Martin A. Fischler and Robert C. Bolles 
```
> 출처 : [RANSAC의 이해와 영상처리 활용(다크 프로그래머)](http://darkpgmr.tistory.com/61)


#### A. 정의 

- 무작위로 샘플 데이터들을 뽑은 다음에 최대로 컨센서스가 형성된 녀석을 선택
- Robust Estimation의 대표적인 알고리즘​


#### B. 최소 자승법 Vs. RACSAC
- 최소자승법(least square method)은 데이터들과의 ∑residual2을 최소화하도록 모델을 찾지만, 
- RANSAC은 컨센서스가 최대인, 즉 가장 많은 수의 데이터들로부터 지지를 받는 모델을 선택

|![image](https://user-images.githubusercontent.com/17797922/40406317-353f30fe-5e9b-11e8-827b-9aca87c0ab2c.png)|![image](https://user-images.githubusercontent.com/17797922/40406328-3cb3edac-5e9b-11e8-99f5-742d3df6b718.png)|![image](https://user-images.githubusercontent.com/17797922/40406328-3cb3edac-5e9b-11e8-99f5-742d3df6b718.png)|
|-|-|-|
|노이즈 데이터|최소 자승법|RANSAC|
|![image](https://user-images.githubusercontent.com/17797922/40406340-42ca8232-5e9b-11e8-95c0-1f2e7a85d0f2.png)|![image](https://user-images.githubusercontent.com/17797922/40406348-497adca8-5e9b-11e8-8550-366a32676f33.png)|![image](https://user-images.githubusercontent.com/17797922/40406351-502b341c-5e9b-11e8-96b3-f251ed4a9345.png)|
|아웃라이어 데이터|최소 자습법(해결 못함)|RANCAS(해결)|

#### C. RANSAC 개선버젼 들 

- MLESAC algorithm
- Local optimized RANSAC (LO-RANSAC), 
- randomized RANSAC algorithm (RRANSAC)

####  D. RANSAC의 활용예
- local feature matching을 이용하여 영상에서 특정 물체를 찾을 때
- Visual Odometry (인접한 영상프레임에서 카메라 모션을 추정할 때)
- 위치인식을 위해 scene matching을 수행할 때
- 물체 추적을 위해 인접한 영상프레임에서 이동체의 모션을 추정할 때



#### E. 기본 동작과정 

1. 최대값이 없도록 c_max = 0으로 초기화작업을 한다.
2. 무작위로 2점을 뽑는다. (p1, p2) (만약 2차함수면 포물선이기 때문에 3개를 뽑아야 한다.)
3. 두 점을 지나는 직선 f(x)를 구한다.
4. 임계값 T를 설정한다.
5. 구해진 f(x)와 데이터들간의 거리를 구한다. 거리는 $$ r_i = \mid y_i - f(x_i) \mid $$ 같다. 이 거리가 T를 넘지 않는다면 개수를 Counting 하라.
6. 개수인 C가 c_max와 비교해 더 크다면 현재 f(x)를 저장하고 그렇지 않으면 버린다.
7. 2~6을 N번 반복한 후 최종 저장된 f(x)를 반환한다.
8. (선택사항) 최종 f(x)를 지지하는 데이터들에 대해 최소자승법을 적용하여 결과를 refine한다.



#### F. 파라미터 

- 임계값 T : inlier(참인정보)와 outlier(거짓정보)의 경계

- 반복수 N : 샘플링 과정을 몇 번 (N) 반복

#### G. 문제점 

- 매번 결과가 달가 질수 있음 






---
## 2. Ground Segmentation

지면 포인트들을 그룹화  
배경을 제거 하기 위한 작업

### 2.1 RANSAC plane filtering

RANSAC은 주어진 데이터에서 반복적으로 샘플을 취하여 전체 데이터를 분석하는 방법론이다\[7,8\]. 이는 모든 데이터를 이용하지 않기 때문에 비교적 빠르고, 잡음에 강한 모습을 보인다

* 센서에서 측정한 데이터로부터 벽면과 같은 직선을 특징으로 뽑아내기 위해 RANSAC을 용용

* 지면을 제거하게 되면 각각의 오브젝트 들이 서로 연결되지 않고 떨어지기때문에 segmentation이 쉬워집

* 제거를 위해서 바닥은 평평\(`even plane`\)하거나, 약간의 경사가 있다고 가정 한다. \(`small elevations like curbside`\)

The algorithm assumes that all of the data in a dataset is composed of both **inliers** and **outliers**.

* Inliers can be defined by a particular model with a specific set of parameters.
* Outliers if that model does not fit then it gets discarded.

By modeling the table as a plane, we can remove it from the point cloud.

* Segmentation of the table from everything else 
* to identify the table.

RANSAC은 포인트들이 특정 Model에 속하는지 아닌지 식별 한다. `Random Sample Consensus \(RANSAC\) is used to identify points in the dataset that belong to a particular model.`

RANSAC은 포인트들은 두 종류로 구분 된다고 가정 한다: inlier, outlier ` It assumes that all of the data in a dataset is composed of both inliers and outliers,`
- where inliers can be defined by a particular model with a specific set of parameters, 
- and outliers don't.

```
RANSAC 알고리즘을 이용한 지상 라이다 포인트 클라우드의 세그먼테이션, 2009, 정성수 (파라미터 T 구하는법 기술)
```

> [cpp 코드 ](http://blog.daum.net/pg365/242)
>
> 상세한 내용은 [Object detection in 3D point clouds](https://www.mi.fu-berlin.de/inf/groups/ag-ki/Theses/Completed-theses/Master_Diploma-theses/2016/Damm/Master-Damm.pdf)의 20page참고

### 2.2 지면 모델

모든 영역을 센서 중심으로 2차원 극좌표 격자로 나눈 뒤, 각 격자에 속하는점들 중 최저점을 뽑아낸다. 반지름 방향으로 각 격자에서 뽑힌 최저점들과 여러 조건을 바탕으로 지면 모델을 결정하게 되고, 이 지면 모델로부터 일정 거리 이내의 점들을 지면점, 그렇지 않은 점들을 비 지면점으로구분

```
[8] H. Himmelsbach, Felix v. Hundelshausen and H.-J. Wuensche, “Fast Segmentation of 3D Point Clouds for Ground Vehicles,” Intelligent Vehicles Symposium, San Diego, CA, USA, June 2010.
```

지면 제거 및 클러스터링은 Yani의 DoN\(Difference of Normals\) 알고리 즘\[2\]을 적용했다. DoN 알고리즘에 모든 포인트 클라우드에 적용한다면 양이 워낙 많기 때  
문에 처리 속도가 빠르지 못한다. 따라서 실시간에는 적합하지 않지만 본논문은 전방에 대해서만 객체 검출을 진행 했고, 또한 근거리는 제외 대상 이고 일반적으로 객체는 높은 곳에 있지 않기 때문에 많은 양의 포인트 클라우드를 제외시킬 수 있어 실시간으로 사용 가능하다.

> 다중 저채널 라이다와 카메라의 센서 융합에 의한 차량 객체 검출 알고리즘

### 2.3 IPP


---

[[추천] UDACITY강좌](https://classroom.udacity.com/courses/ud810/lessons/3189558841/concepts/31679389240923) :컴퓨터 비젼 - RANSAC
