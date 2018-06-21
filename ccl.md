# Clustering 

> 컴퓨터 비젼 알고리즘으로 2D 이미지 대상 설명임 

## 2. Connected-component labeling

CCL 알고리즘은 이미지를 일정한 구역들로 나누고 인접한 구역들끼리의 유사성을 판단하여 같은 label로 묶음

인접 기준 : 픽실 기준 
- 4연결 : 좌우상하
- 8연결 : 좌우상하 + 각 대각선 



연결요소라벨링(connected component labeling)는 크게 두가지 알고리즘이 존재
- 재귀알고리즘
- 반복알고리즘 

### 2.1 동작 과정 

#### A. 재귀 알고리즘 
TBD..

#### B. 반복 알고리즘 

절차 
- 1차(위 - 아래) : 객체에 라벨을 부여하고 라벨에 대응하는 등가표(eqivalent table)를 작성 
- 2차(왼쪽 - 오른쪽) :등가표를 적당히 조정하고(resolve) 이에 맞추어서 이미지의 객체에 부여하는 라벨 번호도 조정

```
물체가 있는 픽셀을 만나면 위/왼쪽 체크 

-IF 위 AND 왼쪽에 라벨이 없으면
 - 새 라벨 부여, 등가표 추가

-IF 위 AND 왼쪽에 라벨이 있으면
 - 같은 번호 : 해당 라벨 부여 
 - 다른 전호 : 낮은 라벨 부여, 등가표 조정 O

IF 위 OR 왼쪽에 라벨이 있으면 (한쪽만)
 - 해당 라벨 부여, 등가표 조정 X
```

|![image](https://user-images.githubusercontent.com/17797922/40970433-6bcba7fa-686f-11e8-806c-31d2c6ca5138.png)|![image](https://user-images.githubusercontent.com/17797922/40970448-7d901b7e-686f-11e8-904e-28e9c49f7849.png)|![image](https://user-images.githubusercontent.com/17797922/40970476-9b0e5a6c-686f-11e8-8482-b505a4e2e841.png)|
|-|-|-|
|픽셀(7)의 위(3)/왼쪽(6)에 라벨 없음<br> . 새 라벨`1` 부여 <br>. 등가표 추가|픽셀(9)의 위(5)/왼쪽(8)에 라벨 없음<br> . 새 라벨`2` 부여 <br>. 등가표 추가|픽셀(11,13,14)의 한쪽에 라벨 있음<br> . 해당 라벨 부여 <br>. 등가표 추가 없음|
|![image](https://user-images.githubusercontent.com/17797922/40970486-a5fbf59c-686f-11e8-973d-e788014ff323.png)|![image](https://user-images.githubusercontent.com/17797922/40970512-b4eb804a-686f-11e8-91f0-f65fb8cadf38.png)||
|픽셀(15)의 위(10)/왼쪽(14)에 모두 라벨 있음<br> . 가장 작은 라벨 부여 <br>. 등가표 변경(2-1??)|등가표를 보고 2와 1이 같음 파악 <br> 2를 1로 변경 ||




논문[8]에서는 데카르트 좌표계에서 2차원 격자로 나눠 CCL을 적용



> 본 연구에서는 앞서 지면 추출 단계에서 생성된 2차원 극좌표 격자에 CCL을 적용하여 기존 결과를 재사용하는 방법을 취하였다. 센서특성에 더 적합한 방법이며, 재사용성을 이용해 메모리나 연산시간 면에서 더 효과적이다.

### 코드 

I think you can use scipy's [connected_components](http://docs.scipy.org/doc/scipy-0.17.0/reference/generated/scipy.sparse.csgraph.connected_components.html) and scikit-learn's [kneighbors_graph](http://scikit-learn.org/stable/modules/generated/sklearn.neighbors.kneighbors_graph.html) together. Does this produce what you're looking for?

```python
from sklearn import neighbors
from scipy.sparse import csgraph
adj = neighbors.kneighbors_graph(X, N_k)
n_components, labels = csgraph.connected_components(adj)
```


---

## 참고 

- [A Review of World’s Fastest Connected Component Labeling Algorithms: Speed and Energy Estimation](https://hal.inria.fr/hal-01081962/document): 2014

- [8] H. Himmelsbach, Felix v. Hundelshausen and H.-J. Wuensche, “Fast Segmentation of 3D Point Clouds for Ground Vehicles,” Intelligent Vehicles Symposium, San Diego, CA, USA, June 2010.


- 코드 : https://www.codeproject.com/Articles/336915/Connected-Component-Labeling-Algorithm

- [[VB.Net 영상처리] 일지 10 : Connected component Labeling....영상 인식의 세번째](http://m.blog.daum.net/shksjy/198?np_nil_b=2): 추천 


- Connected Component Labelling
    - [Pixel neighbourhoods and connectedness](http://aishack.in/tutorials/pixel-neighbourhoods-connectedness/)
    - [Connected Component Labelling](http://aishack.in/tutorials/connected-component-labelling/)
    - [Labelling connected components - Example](http://aishack.in/tutorials/labelling-connected-components-example/)


- [Connected component labeling](https://blogs.mathworks.com/steve/2007/05/11/connected-component-labeling-part-5/): 매트랩

- [Find number of connected components of a k-nearest neighbours graph?](https://stackoverflow.com/questions/36294229/find-number-of-connected-components-of-a-k-nearest-neighbours-graph)

- [3D Connected Component in Cython](
https://codereview.stackexchange.com/questions/189816/3d-connected-component-in-cython)

- [CONNECT - Find 3D Connected Components](https://wrf.ecse.rpi.edu//pmwiki/Research/ConnectedComponents?from=Main.ConnectedComponents), [코드

https://wrf.ecse.rpi.edu/pmwiki/pmwiki.php/Research/ConnectedComponentsImplementation




