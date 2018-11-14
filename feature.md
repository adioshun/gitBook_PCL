# Lidar Features 

>  [PCL/OpenNI tutorial 4: 3D object recognition \(descriptors\)](http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_4:_3D_object_recognition_\(descriptors\))

![image](https://user-images.githubusercontent.com/17797922/47074467-68e8ff80-d235-11e8-9c5c-541cf31ac671.png)

Feature요구 사항 

1. It must be robust to transformations: 
    - rigid transformations (the ones that do not change the distance between points) like translations and rotations must not affect the feature. 
    - Even if we play with the cloud a bit beforehand, there should be no difference.
2. It must be robust to noise: 
    - measurement errors that cause noise should not change the feature estimation much.
3. It must be resolution invariant: 
    - if sampled with different density (like after performing downsampling), the result must be identical or similar.

계산후에는 식별자의 크기를 히스토그램들을 이용하여 줄여야 한다. `After calculating the necessary values, an additional step is performed to reduce the descriptor size: the result is binned into an histogram.`
- To do this, the value range of each variable that makes up the descriptor is divided into n subdivisions, 
- and the number of occurrences in each one is counted

분류 
- Global Descriptors
- Local Descriptors

## 1. Local descriptors

지역 기술자는 각 포인트들을 계산하다. Local descriptors are computed for individual points that we give as input. 

단지, 주변 포인트들을 고려하여 local geometry가 어떤지를 기술한다. They have no notion of what an object is, they just describe how the local geometry is around that point. 

대부분 사용자가 포인트의 어느 부분을 계산할지 선택 한다. Usually, it is your task to choose which points you want a descriptor to be computed for: the keypoints. 

Most of the time, you can get away by just performing a downsampling and choosing all remaining points, but keypoint detectors are available, like the one used for NARF, or ISS.

지역 기술자는 물체 인식이나 Registration에 활용된다. `Local descriptors are used for object recognition and registration.`

### 1.1 PFH (Point Feature Histogram)

- PCL에서 제공하는 가장 중요한 기술자 이다. `It is one of the most important descriptors offered by PCL and the basis of others such as FPFH. `

- PFH은 주변 Normal 방향들의 차이점을 분석하여 기하학적 정보를 수집한다. `The PFH tries to capture information of the geometry surrounding the point by analyzing the difference between the directions of the normals in the vicinity`
    - 이때문에 부정확한 Normal을 기반으로 하면 기술자 질도 좋지 않다. `(and because of this, an imprecise normal estimation may produce low-quality descriptors).`
    
1. 모든 포인들에 대하여 주변점들과 쌍을 생성한다. First, the algorithm pairs all points in the vicinity (not just the chosen keypoint with its neighbors, but also the neighbors with themselves). 
2.  Then, for each pair, a fixed coordinate frame is computed from their normals. 
    - With this frame, the difference between the normals can be encoded with 3 angular variables. 
These variables, together with the euclidean distance between the points, are saved, and then binned to an histogram when all pairs have been computed. 
The final descriptor is the concatenation of the histograms of each variable (4 in total).

|![](http://robotica.unileon.es/images/d/df/PFH_neighbors.png)|![](http://robotica.unileon.es/images/e/e1/PFH_frame.png)|
|-|-|
|Point pairs established|Fixed coordinate frame and angular features computed for one of the pairs|



### 1.2 

특정 점으로부터 일정 반경 내의 점들이 갖는 법선 백터들 간의 각도 관계를 표현하는 히스토 그램이다. 

1. 임이의 두점에서의 법선 벡터를 기반으로 Darboux좌표계를 식 (1)과 같이 정의하고 
2. 이를 기준으로 법선 벡터와 좌표계가 이루는 각도를 식 (2)와 같이 계산한다. 


점군 데이터의 표면 법선 벡터를 기반으로 
1. 질의점의 표면 법선 벡터와 목표점의 표면 법선 벡터를 기반으로 좌표계를 설정하고 
2. 이 좌표계와 법선 벡터 사이의 관계를 수치화하여 특징 벡터를 구성하는 방법이다.




$$ uvw $$프레임 = $$ u $$축을 질의점의 표면법선 벡터 $$n_s$$로 설정 한다면, $$v$$축과 $$w$$축은 아래와 같다. 

![](https://i.imgur.com/n8kn6Zg.png)

- 질의점 $$p_s$$의 표면 법선 벡터 = $$n_s$$
- 목표점 $$p_t$$의 표면 법선 벡터 = $$n_t$$

위와 같이 설정된 $uvw$프레임을 이용하여 표면법선벡터와의 관계를 수치화 할수 있다. 


특징 벡터 $$F= <\alpha, \phi, \theta, d >$$ 계산식 (4차원)
- 점 사이 의 거리
- 법선 벡터와 좌표계의 축이 이루는 각도가 각 차원의 값

![](https://i.imgur.com/ZOyMcAS.png)



차이점 
- 특히 FPFH 방식은 PFH와 달리 속도를 크게 개선한 방법으로 질의점 과 주변 점들 사이의 특징 벡터를 계산하고, 그것을 다시 활용하는 전략 을 이용한다. 
- 이렇게 미리 저장한 특징 벡터를 활용하여 성능에는 큰 차이가 없게 하고, 속도를 크게 개선하였다.

## Signature of Histogram of OrienTation (SHOT)

FPFH와 같이 3차원 기하학적 특성인 표면 법선 벡터를 이용한 특징
점 추출 방법인 SHOT는 3차원 점군 기반의 인식 기술에 널리 이용되고
있다. SHOT는 FPFH와 달리 따로 좌표계를 설정하여 특징 벡터를 구성
하지 않고, 특정한 그리드 영역을 만들고 그 안에 존재하는 점들의 표면
법선 벡터와 질의점의 표면 법선 벡터 사이의 각도를 이용하여 특징 벡
터를 구성한다.
그림 7과 같이 질의점을 기준으로 구형 그리드 구조를 설정한다. 구형
구조에서 반경에 따른 구간을 나누고 이것을 다시 방위각과 높이에 따른
섹터로 분할한다. 본 연구의 실험에서 사용된 SHOT의 그리드 구조는
방위각을 8개의 구간, 반경을 2개의 구간, 높이에 따른 구간을 2개로 나
누어서 총 32개의 구형 그리드 섹터로 나누어지는 구조를 사용하였다.


구간을 나눈 뒤, 질의점의 표면 법선 벡터를 $n_s$라 하고 특정 구간에
속하는 $i$번째 목표점의 표면 법선 벡터를 $n_i$라 한다면 이 두 벡터 사이
의 각을 나타내는 수치는 다음과 같이 표현될 수 있다.

![](https://i.imgur.com/41iPa6h.png)

이 수치는 다시 11개의 구간으로 구별되어지고, 위에서 그리드 구조로
나뉜 32개의 구형 그리드 섹터와 조합되어서 총 352개의 차원을 가지는
SHOT 특징 벡터를 구성한다.


> 하영민, 손 및 팔의 자세 추정을 위한 다시점 뎁스 데이터의 3차원 정합, 2014


## 2. Global descriptors

전역 기술자는 물체의 기하학 정보를 가지고 있다. Global descriptors encode object geometry. 


전역 기술자는 개별 포인트들을 계산하는 대신 물체를 나타내는 모든 클러스터를 계산한다. `They are not computed for individual points, but for a whole cluster that represents an object. `
- 이때문에 후보군 추출을 위한 전처리(=세그멘테이션)이 필요 하다. `Because of this, a preprocessing step (segmentation) is required, in order to retrieve possible candidates.`


전역 기술자는 물체 인식이나 분류, 기하학적 분석(물체 타입, 모양), 자세 추정 등에 활용된다. `Global descriptors are used for object recognition and classification, geometric analysis (object type, shape...), and pose estimation.`


많은 지역 기술자를은 전역 기술자처럼 사용 할수도 있다. You should also know that many local descriptors can also be used as global ones. 
- This can be done with descriptors that use a radius to search for neighbors (as PFH does). 
- The trick is to compute it for one single point in the object cluster, and set the radius to the maximum possible distance between any two points (so all points in the cluster are considered as neighbors).


---

## 1. Height Features


# Descriptors 

> Toword3d2011alex

- Holistic descriptors


- Segment Descriptors 

---

5. 객체 인식
> http://daddynkidsmakers.blogspot.com/2015/07/3.html

객체를 인식한다는 것은, 객체의 형상, 위치, 크기, 치수 및 속성값 등을 추출한다는 것이다. 세그먼테이션을 하였다면, 각 세그먼트마다, 이 속성값들을 추출하기 위해, 다양한 전략을 적용한다.

1) Hough Transformation
허프변환을 통해, 기본 형상의 단면에 대해 가장 잘 부합하는 치수를 찾는다. 허프변환은 2차원 비전에도 활용된 방식으로, 정규분포를 가정하고, 모델 치수의 평균값을 계산한다.

2) PFH Matching
인식하고자 하는 모델의 특징점 히스토그램을 미리 저장해 놓고, 입력되는 PCD에 대해, 이 값과 비교한다. 이는 2차원 비전에서 히스토그램 매칭과 같은 개념이다.

3) RANSAC
모델을 수학적으로 미리 정의해 놓는다. 그리고, 주어진 PCD의 샘플점을 획득해, 수학적 모델을 만들고, 이 모델과 PCD가 얼마정도 부합하지지 inlier 포인트 갯수를 계산한다. 이를 통해, 모델과 유사도를 계산해, 유사도가 미리 정의한 Tolerance 값을 넘으면, 그 모델의 수치가 PCD와 일치한 것으로 가정한다.

4) Curve fitting
커브 피팅은 주어진 PCD에 가장 일치하는 수학 모델의 계수값을 계산하는 방식이다. 예를 들어, 다음과 같은 모델 수식이 있다고 하면,

P(t) = ax + by + c

여기서, 주어진 포인트 P={x, y, z} 의 군 PCD에 대해, 이 모델 수식과 가장 잘 부합하는 a, b, c값을 찾는다. 이는 복잡한 수치해석이 필요하다.

5) ICP (Interactive Iterative Closest Point)
두개의 포인트 클라우드 PCD1, PCD2가 있을 경우, 각 PCD의 특징점 집합을 구해, 서로 위치가 같도록 (서로간의 거리가 가깝도록), 좌표 변환 행렬을 계산해 각 PCD에 적용한다. 그리고, 이 과정을 계산된 거리 편차가 Tolerance이하 일때까지 반복적으로 실행한다.

---

### 3.3 Features (7개)

Our feature vector per cluster consists of 8 different features introduced in the literature, 
- where f1 and f2 are presented by Premebida et al. [17] and describe the number of points included in a cluster and the minimum distance of the cluster to the sensor. 
- Navarro-Serment et al. [15] apply a Principal Component Analysis (PCA) to the clusters, which represents f3 to f7. 

Those features are the 3D covariance matrix of a cluster, the normalized moment of inertia tensor, the 2D covariance matrix in different zones (cf. [15]), the normalized 2D histogram for the main plane and the normalized 2D histogram for the secondary plane. 

In another approach, Kidono et al. [11] introduce two additional features. 
- The first one, the slice feature of a cluster, forms our last feature f8 and aims to differentiate pedestrians from false positives in the shape of trees or poles. 
    - A cluster is partitioned into slices along the z-axis and for each slice the first and second largest eigenvalue is calculated. 
    - As the descriptive power of the slice feature decreases over longer distances, only a rough estimate remains in long distances.
- The other feature introduced by Kidono et al. considers the distribution of the reflection intensities in the cluster. 
    -Since our LRF is not calibrated wrt. the intensities, this feature could not be integrated.

> Confidence-Based Pedestrian Tracking in Unstructured Environments Using 3D Laser Distance Measurements

---




