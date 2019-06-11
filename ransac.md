# RANSAC 

## 최소 자승법과 RANSAC

어떤 모델의 파라미터를 구하는 한 방법 

- 최소 자승법 정의 : 데이터와의 residual^2의 합을 최소화하도록 모델의 파라미터를 구하는 방법
- 최소 자승법 단점 : 데이터에 outlier가 있으면 적용이 어려움 
- 치소 자승법 대안 
	- LMedS
	- M-estimator 
	- RANSAC 




# 다시 살펴 보기 

https://www.google.co.kr/search?newwindow=1&client=ubuntu&hs=m2g&q=%ED%8C%8C%EC%9D%B4%EC%8D%AC+ransac&sa=X&ved=2ahUKEwjHn4Du8t7iAhXlGaYKHZC3CGkQ1QIoCHoECAoQCQ&biw=2560&bih=1194


https://carstart.tistory.com/190

https://kkhipp.tistory.com/50

https://m.blog.naver.com/PostView.nhn?blogId=tlaja&logNo=220766940920&proxyReferer=https%3A%2F%2Fwww.google.co.kr%2F

[cpp 코드 ](http://blog.daum.net/pg365/242)

[Object detection in 3D point clouds](https://www.mi.fu-berlin.de/inf/groups/ag-ki/Theses/Completed-theses/Master_Diploma-theses/2016/Damm/Master-Damm.pdf)의 20page참고

---

측정 노이즈(Noise)가 심한 원본 데이터로부터 모델 파라메타(Parameta)를 예측하는 방법




> 출처: https://carstart.tistory.com/190 [나이 서른에 햇병아리 프로그래머]



---

# Random sample consensus algorithm (RANSAC)

![](https://i.imgur.com/E8Ynlcr.png)

UDACITY : https://classroom.udacity.com/courses/ud810/lessons/3189558841/concepts/31679389240923


```
"Random Sample Consensus: A Paradigm for Model Fitting with Application to Image Analysis and Automated Cartography", 1981 by Martin A. Fischler and Robert C. Bolles 
```

---

# [How to use Random Sample Consensus model](http://pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus)


> PCL Tutirial 


In this tutorial we learn how to use a RandomSampleConsensus with a plane model to obtain the cloud fitting to this model.

정의 : 아웃라이어를 포함하는 데이터에서 수학적 모델의 **파라미터**를 예측 하기 위한 반복적 수행 기법 `It is an iterative method that is used to estimate parameters of a mathematical model from a set of data containing outliers. `

모든 데이터가 아웃라이어 + 인라이어로만 구성되어 있다고 가정 ` The RANSAC algorithm assumes that all of the data we are looking at is comprised of both inliers and outliers. `
-  인라이어는 모델을 통해 설명이 가능한것, `Inliers can be explained by a model with a particular set of parameter values, while `
-  아웃라이어는 모델에 맞지 않는것 `outliers do not fit that model in any circumstance. `

### wikipedia 정의 


입력 `The input to the RANSAC algorithm is `
- a set of observed data values, 
- a parameterized model which can explain or be fitted to the observations, 
- and some confidence parameters.

수행 방법은 무작위로 샘플을 선택(hypothetical inliers라고함) 하여 반복적으로 아래 절차를 진행 한다. `RANSAC achieves its goal by iteratively selecting a random subset of the original data. These data are hypothetical inliers and this hypothesis is then tested as follows:`
1. A model is fitted to the hypothetical inliers, i.e. all free parameters of the model are reconstructed from the inliers.
2. All other data are then tested against the fitted model and, if a point fits well to the estimated model, also considered as a hypothetical inlier.
3. The estimated model is reasonably good if sufficiently many points have been classified as hypothetical inliers.
4. The model is reestimated from all hypothetical inliers, because it has only been estimated from the initial set of hypothetical inliers.
5.  Finally, the model is evaluated by estimating the error of the inliers relative to the model.

이 반복은 정해진 횟수만큼 진행 된다. This procedure is repeated a fixed number of times, each time producing either a model which is rejected because too few points are classified as inliers or a refined model together with a corresponding error measure. In the latter case, we keep the refined model if its error is lower than the last saved model.

### 장단점 

장점 : 모델 파라미터에 대한 강건한 예측수행. 즉, outliers이 많이 포함되어 있어도 높은 정확도를 보인다. ` An advantage of RANSAC is its ability to do robust estimation of the model parameters, i.e., it can estimate the parameters with a high degree of accuracy even when a significant number of outliers are present in the data set. `

단점 #1 : 계산 부하 `A disadvantage of RANSAC is that there is no upper bound on the time it takes to compute these parameters. `
	- 반복에 제한이 있다면 좋은 결과를 보장 하기 어려움 `When the number of iterations computed is limited the solution obtained may not be optimal, and it may not even be one that fits the data in a good way. `
	- In this way RANSAC offers a trade-off; by computing a greater number of iterations the probability of a reasonable model being produced is increased. 

단점 #2 : **problem-specific thresholds**를 사전에 정의 하여야 함 `Another disadvantage of RANSAC is that it requires the setting of problem-specific thresholds.`

단점 #3 : 하나의 데이터셋에 하나의 모델만 적용이 가능하다. `RANSAC can only estimate one model for a particular data set. As for any one-model approach when two (or more) models exist, RANSAC may fail to find either one.`



## 예시 설명 
The pictures to the left and right show a simple application of the RANSAC algorithm on a 2-dimensional set of data. 



|![](http://pointclouds.org/documentation/tutorials/_images/random_sample_example1.png)|![](http://pointclouds.org/documentation/tutorials/_images/random_sample_example2.png)|
|-|-|

왼쪽 이미지는 인라이어와 아웃라이어를 포함한 데이터세을 표현 한다. `The image on our left is a visual representation of a data set containing both inliers and outliers. `

오른쪽 이미지 아웃라이어는 빨간색으로 인라이어는 파란색으로 표현 하고 있다. `The image on our right shows all of the outliers in red, and shows inliers in blue. `
- 파란선이 RANSAC의 결과 물이다. `The blue line is the result of the work done by RANSAC.`
- 이경우 모델은 Line이다. ` In this case the model that we are trying to fit to the data is a line, and it looks like it’s a fairly good fit to our data.`


---
RANSAC은 포인트들이 특정 Model에 속하는지 아닌지 식별 한다. `Random Sample Consensus \(RANSAC\) is used to identify points in the dataset that belong to a particular model.`

RANSAC은 포인트들은 두 종류로 구분 된다고 가정 한다: inlier, outlier ` It assumes that all of the data in a dataset is composed of both inliers and outliers,`
- where inliers can be defined by a particular model with a specific set of parameters, 
- Outliers if that model does not fit then it gets discarded.





|![](http://pointclouds.org/documentation/tutorials/_images/ransac_outliers_plane.png)|![](http://pointclouds.org/documentation/tutorials/_images/ransac_inliers_plane.png)|![](http://pointclouds.org/documentation/tutorials/_images/ransac_inliers_sphere.png)|
|-|-|-|
|원본|평면 모델 적용|구형 모델 적용|


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
