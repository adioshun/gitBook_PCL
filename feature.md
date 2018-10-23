# Lidar Features 

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
### [PCL/OpenNI tutorial 4: 3D object recognition (descriptors)](http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_4:_3D_object_recognition_(descriptors))

![image](https://user-images.githubusercontent.com/17797922/47074467-68e8ff80-d235-11e8-9c5c-541cf31ac671.png)





