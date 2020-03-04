# RANSAC 

## 최소 자승법과 RANSAC

어떤 모델의 파라미터를 구하는 한 방법 

- 최소 자승법 정의 : 데이터와의 residual^2의 합을 최소화하도록 모델의 파라미터를 구하는 방법
- 최소 자승법 단점 : 데이터에 outlier가 있으면 적용이 어려움 
- 치소 자승법 대안 
	- LMedS
	- M-estimator 
	- RANSAC 



목적 : 측정 노이즈(Noise)가 심한 원본 데이터로부터 모델 **파라메타(Parameta)**를 예측하는 방법
- 직선(모델 : y=ax+b)을 찾는 예시에서는 RANSAC은 이 모델의 매개변수 a와 b를 추정해준다. 



> 출처: https://carstart.tistory.com/190 [나이 서른에 햇병아리 프로그래머]





> [최소자승법 이해와 다양한 활용예 (Least Square Method)](https://darkpgmr.tistory.com/56)

# 다시 살펴 보기 

https://www.google.co.kr/search?newwindow=1&client=ubuntu&hs=m2g&q=%ED%8C%8C%EC%9D%B4%EC%8D%AC+ransac&sa=X&ved=2ahUKEwjHn4Du8t7iAhXlGaYKHZC3CGkQ1QIoCHoECAoQCQ&biw=2560&bih=1194


https://carstart.tistory.com/190

https://kkhipp.tistory.com/50

https://m.blog.naver.com/PostView.nhn?blogId=tlaja&logNo=220766940920&proxyReferer=https%3A%2F%2Fwww.google.co.kr%2F

[cpp 코드 ](http://blog.daum.net/pg365/242)

[Object detection in 3D point clouds](https://www.mi.fu-berlin.de/inf/groups/ag-ki/Theses/Completed-theses/Master_Diploma-theses/2016/Damm/Master-Damm.pdf)의 20page참고

---



---

# Random sample consensus algorithm (RANSAC)

![](https://i.imgur.com/E8Ynlcr.png)

UDACITY : https://classroom.udacity.com/courses/ud810/lessons/3189558841/concepts/31679389240923


```
"Random Sample Consensus: A Paradigm for Model Fitting with Application to Image Analysis and Automated Cartography", 1981 by Martin A. Fischler and Robert C. Bolles 
```



---

# RANSAC의 이해와 영상처리 활용

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


```cpp

#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>


int
main(int argc, char** argv)
{
// initialize PointClouds
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);

// cloud에 랜덤 포인트 생성

std::vector<int> inliers;

// created RandomSampleConsensus object and compute the appropriated model
pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));

pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
ransac.setDistanceThreshold (.01);
ransac.computeModel();
ransac.getInliers(inliers);

// copies all inliers of the model computed to another PointCloud
pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

/// ...

return 0;
}
```

[[추천] UDACITY강좌](https://classroom.udacity.com/courses/ud810/lessons/3189558841/concepts/31679389240923) :컴퓨터 비젼 - RANSAC
