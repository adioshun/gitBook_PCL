# [How to use Random Sample Consensus model](http://pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus)


> PCL Tutirial 


In this tutorial we learn how to use a RandomSampleConsensus with a plane model to obtain the cloud fitting to this model.

### Theoretical Primer

정의 : 아웃라이어를 포함하는 데이터에서 수학적 모델의 **파라미터**를 예측 하기 위한 반복적 수행 기법 `It is an iterative method that is used to estimate parameters of a mathematical model from a set of data containing outliers. `

모든 데이터가 아웃라이어 + 인라이어로만 구성되어 있다고 가정 ` The RANSAC algorithm assumes that all of the data we are looking at is comprised of both inliers and outliers. `
-  인라이어는 모델을 통해 설명이 가능한것, `Inliers can be explained by a model with a particular set of parameter values, while `
-  아웃라이어는 모델에 맞지 않는것 `outliers do not fit that model in any circumstance. `

### wikipedia 정의 


입력 `The input to the RANSAC algorithm is `
- 입력 데이터 `a set of observed data values, `
- 파라미터로된 모델 `a parameterized model which can explain or be fitted to the observations, `
- 신뢰도 파라미터 `and some confidence parameters.`

수행 방법은 무작위로 샘플을 선택(hypothetical inliers라고함) 하여 반복적으로 아래 절차를 진행 한다. `RANSAC achieves its goal by iteratively selecting a random subset of the original data. These data are hypothetical inliers and this hypothesis is then tested as follows:`

1. A model is fitted to the hypothetical inliers, 
	- i.e. all free parameters of the model are reconstructed from the inliers.

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

## Code 

코드 분석을 위한 간략화 버젼 

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// How to use Random Sample Consensus model
// http://pointclouds.org/documentation/tutorials/random_sample_consensus.php#random-sample-consensus


int
main(int argc, char** argv)
{
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile<pcl::PointXYZ>("sample_consenus_input.pcd", *cloud);
  //https://github.com/adioshun/gitBook_Tutorial_PCL/blob/master/Beginner/sample/sample_consenus_input.pcd
  
  std::vector<int> inliers;
  
  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));  
  
  // model_p
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
  ransac.setDistanceThreshold (.01);
  ransac.computeModel();
  ransac.getInliers(inliers);
	
  pcl::copyPointCloud (*cloud, inliers, *final);
  pcl::io::savePCDFile<pcl::PointXYZ>("sample_consenus_final_model_p.pcd", *final);

  return 0;
  
  // 다른 방법들 
  //https://github.com/PointCloudLibrary/pcl/blob/master/test/sample_consensus/test_sample_consensus_plane_models.cpp
 }

```

## 결과 

RANSAC은 포인트들이 특정 Model에 속하는지 아닌지 식별 한다. `Random Sample Consensus \(RANSAC\) is used to identify points in the dataset that belong to a particular model.`

RANSAC은 포인트들은 두 종류로 구분 된다고 가정 한다: inlier, outlier ` It assumes that all of the data in a dataset is composed of both inliers and outliers,`
- where inliers can be defined by a particular model with a specific set of parameters, 
- Outliers if that model does not fit then it gets discarded.





|![](http://pointclouds.org/documentation/tutorials/_images/ransac_outliers_plane.png)|![](http://pointclouds.org/documentation/tutorials/_images/ransac_inliers_plane.png)|![](http://pointclouds.org/documentation/tutorials/_images/ransac_inliers_sphere.png)|
|-|-|-|
|원본|평면 모델 적용|구형 모델 적용|



