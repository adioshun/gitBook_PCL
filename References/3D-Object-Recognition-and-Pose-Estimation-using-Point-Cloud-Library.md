> https://drive.google.com/file/d/1QtQTlm3_FiOdBslbtMAubVMyd2Bjofl1/view?fbclid=IwAR0NZfTAvfSwg_X_Flx5Uhg5GMLRaNFdgKU6PZRsHuskc95Sd2ErAKLg4LM


## 2.4. Local feature method

이전 방법 대비 강건성과 일관된 결과를 얻을수 있는 방법이 필요 하다. `It is necessary to find a more complex solution that will be robust and consistent, and can also eliminate the limitations or shortcomings of the previous method. `

해결 방법은 물체의 **local geometry**에 대한 유용한 정보 수집하고 -> 모델과 비교 하여 물체 탐지 및 위치, 회전을 알아 내는 것이다. `The problem will be accessed by collecting useful information about the local geometry of objects on the scene and then, by comparing the models and scenes, detect the desired object and determine its spatial position and orientation. `

지역특징 방법은 다음을 필요로 한다. `The local feature method requires filtering, key point calculation, surface normals, descriptors, extracting case scenes that will represent the model, and algorithms for comparing and evaluating descriptors. `
- 모델을 표현하는 정보 추출 절차  : filtering, key point calculation, surface normals, descriptors
- 기술자를 평가 하기 위한 비교 알고리즘 

The problem is the correct choice between the specified **requirements** and the correct setting of a number of **parameters**. 

지역특징 기반 방식에서 가장 중요한 역할을 차지 하는것은 **descriptors**이다. 이는 **키포인트** 주변 물체의 기하학적 정보를 저장 한다.  ` One of the most important roles in the local feature method has descriptors that count and store information about the geometry of the object around the key point. `

Based on computed information, a comparison of models and scenes is done, and ultimately the correspondence is determined. 

Correspondence means the very, very similar, points between the model and the scene.

### 2.4.1 Point cloud processing

필터링, 표면 재구성, 배경제거등은 지역 특징 방법에 사용된다. `Filtration, surface reconstruction and background removal, described in chapters 2.1 to 2.3, are still used and play an important role in the local feature method.`

descriptors가 좋은 정보 추출을 위해서는 노이즈나 불규칙성은 제거되어야 하기 때문이다. `Since descriptors, in the local feature method, collect information about geometry, the idea is that the scene has as little noise and irregularity as the collected information is credible, ie the same / similar in each frame for the same scene. `

여러 필터와 잡음 제거 방법들이 이러한 역할을 수행 한다. `In addition to the Passthrough and Voxel filters, the Statistical Outlier Removal Filter is particularly evident as it removes the remaining points caused by the sensor error, or points that do not have the role of describing the object on the scene, and can negatively affect stability, repeatability, decrease in the number of correspondence and wrong final position estimate.`

지역 영역의 기하학적 정보에 대한 중요 특징중 하나인 **법선**은 표면정보가 왜곡되어 있으면 제대로 구하기 힘들다. 이를 해결 하기 위해서 **표면 재구성** 절차가 사용된다. `Normal, as one of the main features of local area geometry information, can often be misdiagnosed for surface distortion due to sensor error. To correct the matter, algorithms from the surface reconstruction are used, whose actions are shown in Figure 2.3.`

배경제거는 필수는 아니지만 처리 속도 향상을 위해 필요 하다. `Removing the background is not a necessity of processing as in the cluster extraction method, however, descriptors are a processor-demanding process and the presence of background can significantly slow down the work of the program for a number of calculated features. `

배경이 없다는 것은 더 적은 수의 key points, normals, and descriptors계산만 해도 되기 때문이다. `The absence of the background means that a smaller calculation of the number of key points, normals, and descriptors is required.`

### 2.4.2. Normal

노멀 벡터는 접평면에 수직이다. 표면의 방향을 결정할때 사용된다.  `In 3D space, normal vectors are vertical to a tangent plane(접평면) made up of a set of points in the local area, and serve to mathematically determine the orientation of the surface that confines the observed whole, in this case, they are the required objects on the scene.`

![](https://i.imgur.com/bTHY63d.png)
```
Figure 2.5. Normal on the surface, 
- red areas - object surface, 
- blue area - tangential plane
```

문제는 동일한 접평면에 대하여 두개의 노멀(회전)이 탐지 될때 발생 한다. `The normatic calculation problem occurs because there is a possibility of double orientation of the normal to the same tangential plane as shown in the following figure.`

![](https://i.imgur.com/WcMef3e.png)

PCL에서는 노멀 계산을 위해서 PCA를 사용합니다.(차원 축소와 변형에 강건성을 유지 가능)` For calculating normal within a PCL, Principal Component Analysis (PCA) is used, a method that reduces the dimensionality and variance of the variables in order to make it clear and simplify a large number of data.`

무작위 수학적 계산이 불가능 하기 때문에 계산된 노멀 정보는 아래 그림처럼 표현 됩니다. `Due to the impossibility of a random mathematical calculation of the constant orientation of normal to the surface, normal different orientations appear in Figure 2.7.`

|원본 | 보정 |
|-|-|
|![](https://i.imgur.com/9DTnP8k.png)|![](https://i.imgur.com/pCB6SH6.png)|



문제 해결을 위해 관점(point of view)정보가 정의 되어야 합니다. `In order to correct the problem, a point of view (point of view) should be defined according to which normal orientation will be directed to achieve a uniform orientation.`

This equates to the equation:
$$
\overrightarrow{n_{i}} \cdot\left(v_{p}-p_{i}\right)>0
$$

- n  - vectors are normal, 
- v_p defined point of view
- p_i center point within the defined radius or adjacent points. [13]


먼저 센서를 기준으로 관점을 정하는 것으로 시작 됩니다. `Initially, the point of view is defined by the camera sensor, which may also sometimes cause inconsistent orientation of the normal.`

노멀 계산 과정은 아래와 같습니다. `Pseudo code for normal calculation:`
1. For all points within the clouds, find the nearest neighbor using a defined radius
2. Calculate normal n for point p
3. Orient normal to a point of view

### 2.4.3. Key points

![](https://i.imgur.com/CQ5NSnv.png)

필터링을 수행 했어도 input parameter for descriptors로는 점군의 수가  많아 계산 부하를 가져 올수 있다. 이경우 키포인트/관심점(points of interest)으로의 통합이 필요 하다. ` The captured scene with a 3D sensor after filtering can still contain tens of thousands of points depending on the number of objects on the scene, which is quite a lot like the input parameter for descriptors that are processor-computational, so it is necessary to integrate points of interest. `

키포인트는 적은 포인트로 물체를 표현하기 위해 사용된다. 따라서 포인트가 있는 특정한 지역의 특성을 계산하여 표현한다. (예. 기하학정보, 색상 변경) `Key points or points of interest have the purpose of presenting a scene /object in a small number of points so that they are calculated at locations where specific local characteristics are present, for example geometry or color changes. `

As such, they should fulfill the repeatability property, that is, regardless of the translation or rotation of the object, lie on the same geometry in multiple iterations of the program. [14] 

PCL에서 제공하는 키포인트 탐지 알고리즘은 아래와 같다. `Within the PCL there are more key point detectors such as`
- Noble, Curvature, ISS3D, Harris 3D, SUSAN, Lowe, KLT, Voxel, SIFT, NARF and many more. 

어떤 알고리즘을 선택 하느냐는 경험적 지식에 기반한다. `Selection was determined on the basis of experimental researches [15] and [16], where the percentage of repeatability of key points was observed with respect to multiple rotations or translation of objects relative to the defined radius for the classification of nearest neighbors.`

###### [16]의 연구 결과 

> 생략 

###### [17]의 연구 결과 

> 생략 

### 2.4.4. Descriptors

노멀과 키포인트등의 특징 계산후 기술자(descriptors)를 이용하여 물체를 인식 하는 작업을 수행 합니다. `After performing the required filtration and the above-described calculated features, such as normal and key points, descriptors play the descriptive role of object recognition. `

기술자는 기술하고자 하는 주변 포인트의 양에 따라서 로컬과 글로벌로 구분 된니다. `Descriptors are shared globally and locally, depending on the size of support around the point they describe. `

###### Local descriptors
- 지역 기술자는 물체를 이해 할때 계산된 키포인트 주변의 점들을 참고 하므로 잡음과 가려짐에 강건 합니다. ` Local descriptors describe the subject by looking at the neighborhood around each calculated key point making them descriptive and robust to noise and occlusion. `
- 가려짐은 비젼 시스템에서 빈번하게 발생 하는 물체의 일부분만 보이는 것이다. `Occlusion in vision signifies an occurrence when an object partially exits the frame or is covered with another object on the scene.`

###### global descriptors
- 전역 기술자는 전체 데이터를 사용한다. 따라서 (부분을 반복하는 로컬대비) 시간적으로 효율적이지만 물체의 세밀한 부분을 탐지 하는것은 약하다. `Unlike locals which describe parts of the object, global descriptors observe the object as a complete set that makes them more compact and time-efficient, but have no ability to analyze specific detail on the object.`
- 전역 기술자는 잡음과 가려짐에도 강건하지 못하다. `And they are not so robust to occlusion and noise, and necessitate segmentation [17] .`

본 연구에서는 가려짐은 필연적이고, 동작 시간은 중요 하지 않지만 강건성과 인지 성능이 중요하다는 가정하에 지역 기술자를 사용한다. `Within the work, local descriptors were observed, assuming that the presence of occlusion is possible and that it is not necessary to focus on the speed of the process execution, but on the robustness and success of the recognition.`

기술자요구 사항 `In order for the descriptors, whether global or local, to yield satisfactory results should have the following characteristics [18]:`
1. Robust on transformations - transformations such as translation and change of orientation should not affect the final result, ie the number of correspondence found.
2. Robust noise - mild variation of noise should not drastically jeopardize the value of the calculated descriptor
3. Robust at resolution - it is expected that for different density of point clouds the descriptors should be similar / same if the geometry around the observed point is the same.

지역 기술자는 특징화 되는 방법에 따라 다시 3개로 나누어 진다. `Furthermore, local descriptors can be divided into three categories with respect to the way they are characterized by: signatures, histograms, and hybrids.`
- signatures
- histograms
- hybrids

###### signature

local reference frame이 변하면 안됨. 제한적 공간에서만 사용 가능 `Descriptors with a signature function for calculation require an invariant local reference frame, ie a coordinate system that will only work in a limited area of space, and the calculation of key points in the local area. `

표현력은 좋지만 잡음에 강건하지 못하다. `The characteristics of such descriptors are very descriptive but very sensitive to noise.`

###### histogram

지역 기하학적 특징 정보 활용, 대부분 노멀 정보와 지역 좌표계와 관련된 정보들 `Descriptors in the histogram category use local geometric features, mostly the relation between normal angles that close in relation to the local coordinate system (LRF). `

표현력은 약하지만, 잡음에 강건하다. `Such descriptors are less descriptive, but are therefore very robust in noise.`

###### Hybrid

혼합 방식은 표현력도 좋고, 잡음에도 강건하다. `Hybrid local descriptors use signatures and histograms that make them descriptive, but also robust. `

단점은 노멀과 키포인트 계산이 필요 하므로 느리다. `Negatively, it requires a calculation of normal and key points that may slow down the program.`


기술자는 일련의 ㅌㅌㅌ다 .`For a better understanding, descriptors can be interpreted as a numerical value attributed to the influence of geometry around the observed point.`

For example, if the geometry of the model corresponds to the geometry of the scene, the calculated descriptors will have very similar or same numerical values meaning the correspondence, or the similarity between the model and the scene or any two point clouds. [Image 2.23]
![](https://i.imgur.com/5nRq0g7.png)


Table 3 shows some of the local descriptors available within the PCL and their basic properties

![](https://i.imgur.com/ZQlZJga.png)

###### 연구결과 [19]

센서에 따라 성능이 차이가 큼 (레이져/CAD  > Kinect)
```
In the literature [19] a study of the performance of the descriptors was made depending on the quality of the data set. For example, data obtained from laser scanners or CAD models are categorized as high quality while the data obtained from Kinect is low or medium. All data sets contain occlusion, and are in the disordered state.
```

결론적으로 descriptors의 선택은 데이터의 질에 달려 있다. `It can be concluded that the choice of descriptors largely depends on the quality of the data used. `

본 논문에서는 키넥트를 사용하였으므로 **hybrid SHOT**를 사용하기로 하였다. ` For the purpose of this paper, where a Kinect v2 camera is used that provides low to mid-quality data, a hybrid SHOT local descriptor will be selected. `

색상 정보를 사용할수 있는 **CSHOT (color - SHOT)**도 고려 하고있다. ` In order to take advantage of the RGB sensor, an upgraded version of the SHOT descriptor, CSHOT (color - SHOT) will be considered.`

#### 2.4.4.1 SHOT Descriptor

The SHOT (Signature of Histograms of Orientations) is a hybrid local descriptor for which the estimate of the required key point of the normal, which makes it descriptive enough and at the same time robust to noise and occlusion. 

Like most descriptors, it is characterized by its robustness on translation and rotation.

It contains a spherical structure around the central point (key point) for which the local coordinate system is defined. 

The spherical structure is divided into 32 volumes in the direction of azimuth, height and radial axes, then for each volume, a 3D histogram, or series of numeric calculations is calculated values based on geometric features within the volume. 

In the end, all calculated histograms are linked to obtain the final descriptor value.

#### 2.4.4.2 CSHOT Descriptor

Color - SHOT is an upgraded version of the SHOT descriptor where for each volume of accounts an additional histogram is based on the RGB information inside the scene. 

This opens the possibility for achieving more positive correspondence and greater robustness for noise and occlusion.

Although many studies indicate better CSHOT results than SHOT [20] [21], Except for the time requirement for the calculation, the experiment within the work found worse results.

Poor results can be attributed to the absence of additional light, which means that color consistency is not achieved in all parts of the scene due to the effect of shading and different illumination.


### 2.4.5 ICP registration

정합의 개념은 두개의 동일한 점군을 정렬시키는 것이다. `The concept of registration in a 3D vision refers to the alignment of two same clouds of points called the source and the target whose goal is fixed and the source is mobile. `

최종적으로는 **변환행렬**을 구하는 것이다. `The idea is to minimize the distance of the model with the same object on the scene and ultimately get their matrix transformation as a difference in translation and rotation[22]. `

ICP알고리즘 수행을 위한 요구 사항은 **최초 위치**를 계산 하는것이다. 최초 위치는 기술자와  their comparisons을 통해 구해진다. ` In order for the ICP (Iterative closest points) registration task to be performed, it requires the calculation of the initial position of the object on the scene, which is the results of the task of the descriptors and their comparisons.`

ICP 정합은 초기 변환에러를 보정하기 위한 일종의 **fine positioning**로도 볼수 있다. `Thus ICP registration can be considered as a fine positioning that will correct the initial transformation error.`

이 알고리즘은 소스와 타겟의 짝지어진 위치의 에러 **갯수**를 카운팅 하여 동작 한다. `The algorithm works by counting the error of the position of each pair of points between two point clouds, i.e., the source and the target. `

최종 변환은 짝지어진 점의 에러에 기반하여 결정 된다. ` The final transformation is determined based on the error of all calculated pairs of points. `

이름에서 알수 있듯이 반복적으로 수행하면서 두 점군의 차이를 줄여 나간다. `From the name itself, the algorithm operates iteratively where at each step the difference between the two clouds is reduced.`

알고리즘은 몇가지 단점이 있다. ` The algorithm contains several basic criteria:`
1. The maximum number of iterations defined by the user
2. The maximum allowable error between the two clouds defined by the user
3. The maximum allowable sum of the Euclidean distance squares defined by the user

지정 횟수만큼 수행을 해도 정합도가 맞지 않는다면 ... `If in the maximum number of iterations the desired value of fitness function is not achieved, the algorithm rejects the result and the program moves from scratch.`

기술자 연산과 단리 ICP는 매우 빠른 알고리즘이고 좋은 결과를 뽑아 낸다. `Unlike computing and comparison of the descriptors, ICP is a very fast algorithm which ultimately does not have a major impact on the duration of the entire program but greatly optimizes the final matrix of transformation.`

ICP는 충분히 좋은 성능을 보이고 있다. `The ICP registration algorithm is already sufficiently self-sufficient to produce useful applications, such as literature research [23]`

![](https://i.imgur.com/uAVRtTu.png)

### 2.4.6 Final transformation matrix

기준 모델과 물체간의 위치 차이를 결정하기 위하서는  transformation을 알아야 한다. `To determine the difference in the position of the reference model and object on the scene, it is necessary to know their transformation with respect to the origin of the camera. `

두 transformation을 알게 되면 회전과 움직임 정도를 알수 있다. `By knowing these two transformations it is possible to determine the difference in the shift and rotation between the reference and the recognized object. `

모델과 물체 사이의 거리는 중앙을 보고 알수 있다. `The distance between the model and object is observed from the centroid or the center of gravity. `

The following figure shows the necessary transformations to get information about the scene's location.

![](https://i.imgur.com/ZeftuXf.png)

- The symbol $$T^O_{K 1}$$  means the transformation of the reference coordinate system (coordinate system of model) O_1 with respect to Kinect, $$T^_O K 2$$ the transformation of the identified object with respect to Kinect, 
- while T O 2 1 the transformation between the two objects on the scene.
- The transformation T O 2 1 is sent to the robot by TCP connection as the final result and can be calculated from the expression:

> 추가 내용 있음 


### 2.4.9 Advantages and Disadvantages of the Local Features Method

A more complex approach to the problem has successfully overcome some of the basic limitations of the previous method, such as the inability to correctly define the object that is to be recognized in an unstructured environment and the problem of merging two objects that are close to one cluster. 

Although descriptors of leading dimensions describing local geometry, are invariant to rotation and translation, testing has shown that larger rotations can drastically affect a smaller final number of correspondence. 

Such a phenomenon may arise due to the inconsistencies of the normals explained in section 2.4.2 and / or the key points explained in section 2.4.3 between the scene and the model. 

The problem can be solved by multiple rotation of models that are stored in the data base and are parallely compared to the scene, or by getting better quality data, but it is necessary to examine it with the experiment.

Because of the variation of the repeatability of the key points, the inaccurate initialization of the initial position obtained from the correspondence is possible, which again affects the poorer positioning by ICP registration, such occurrences are more frequently present in symmetric objects as will be shown later by testing.

Advantages:
- The ability to recognize and locate the desired object in an unstructured environment 
- Multiple recognition of the same objects in one program's iteration 
- Building a data base made up of different objects 
- A large selection of descriptors and methods for calculating key points 
- More access evaluating correspondence to obtain robustness

Disadvantages:
- Process-challenging method
- A large number of mutually dependent parameters that need to be matched
- Large rotations reduce the number of correspondence

---

# 3. IMPLEMENTATION OF RECOGNITION METHOD WITH PCL ASSISTANCE 


## 3.1 Preprocessing


### 3.1.1 Passthrough filter

### 3.1.2 Voxel filter

### 3.1.3 Statistical outlier removal filter

### 3.1.4 Reconstruction of the surface

### 3.1.5 Removing Background - RANSAC Segmentation

### 3.1.6 Removing the background - RGB filtration


## 3.2 Local feature method

Normal, key points and ultimately descriptors are counted for model and scene.

### 3.2.1 Calculation of normal

관점은 기본값으로 센서의 중앙으로 세팅되어 있음 `By default, the viewpoint is in the camera sensor center, which can lead to the problem described in the image (def), so a new view is defined by the .setViewPoint function away from the scene.`

### 3.2.2 Calculating Key Points


### 3.2.3 Calculation of descriptors


### 3.2.4 Comparison and evaluation of descriptors

To compare the descriptors described in section 2.4.4, 
1. include the header `#include <pcl / kdtree / kdtree_flann.h>`, 
2. and then turn on the class `pcl ::KdTreeFLANN <DescriptorType>`.

Use kd-tree to compare scenes and models descriptors. 

The squared distance interval for a SHOT descriptor is 0 to 1, where zero represents the ideal case and the matching of only those descriptors that are exactly the same. 

In order to achieve a greater number of correspondence, and to be composed of as many descriptors as possible, a square distance of 0.335f has been set. 

Due to the possibility of false correspondence, it is necessary to further evaluate them with the Hough3D algorithm, but
before that it is necessary to define a local reference coordinate system for key points of the model and scene.

To implement the local coordinate reference system, 
1. include the header `#include <pcl / features / board.h>`, 
2. and then invoke the class `pcl ::BOARDLocalReferenceFrameEstimation <PointType, NormalType,RFType>`.

The radius to define the local coordinate system must be similar to the radius for the descriptor calculation. 

After defining the local coordinate system, the correlation of the Hough3D algorithm is evaluated. 

To implement the Hough3D algorithm, 
1. include the header `#include <pcl /recognition / cg / hough_3d.h>`, 
2. and then invite the class `pcl ::Hough3DGrouping <PointType, PointType, RFType, RFType>`.

Care must be taken to select these parameters for evaluating descriptors.

What is the value of the .setHoughBinSize function ? 
- Smaller will also be a smaller number of correspondence, but they will be more accurate.
- If the value is set too high, a number of false correspondence is possible resulting in the recognition of the wrong object on the scene. 

What is the .setHoughThreshold parameter ?
- For a higher values, you can expect more accurate correspondence. 
- But for a smaller values, there may be more correspondence, but may appear incorrect one. 

Finally from the resulting vector of correspondences between scene and model, the transformation matrix of initial position of the object on the scene is calculated. 

If there are multiple objects at the same time in the scene, the output will also consist of multiple transformation matrices, which is suitable for working with multiple robots.

