#  DBSCAN(Density-based spatial clustering of applications with noise) 

> 개선 버젼 = OPTICS (It does not need the epsilon parameter (except for performance reasons with index support)

Euclidean clustering, or DBSCAN. 

전체 클러스터 수는 모르지만 일부 정보`(min_samples, max_dist)`를 알고 있을때 사용 `You may not know how many clusters to expect, but you do know something about how the points should be clustered (min_samples, max_dist). `


Each cluster has 3 levels of members: 
- core member, 
- edge member, 
- outlier.

차이점 : 
- K Means나 Hierarchical 클러스터링의 경우 군집간의 거리를 이용하여 클러스터링을 하는 방법인데, 
- 밀도 기반의 클러스터링은 점이 세밀하게 몰려 있어서 밀도가 높은 부분을 클러스터링 하는 방식이다. 
    - 쉽게 설명하면, 어느점을 기준으로 반경 x내에 점이 n개 이상 있으면 하나의 군집으로 인식하는 방식이다.


장점
- K Means와 같이 클러스터의 수를 정하지 않아도 됨
- 클러스터의 밀도에 따라서 클러스터를 서로 연결하기 때문에 기하학적인 모양을 갖는 군집도 잘 찾을 수 있음


단점 
- 많은 연산을 수행하기에 K 평균에 비해 그 속도가 느림
- 반지름과 임계치 설정에 많은 영향을 받는다
- 그리고, 유클리드 제곱거리를 사용하는 모든 데이터 모델의 공통적인 단점인, 'Curse Of dimensionality 또한 존재
 - 이는 2차원이나 3차원 등 차원수가 낮은 데이터세트에는 문제가 되지 않지만, 
 - 고차원 데이터세트로 갈수록 필요한 학습 데이터 양이 급증하는 문제점이며, 이 때문에 많은 연산이 필요해진다는 단점이 있다.

###### 예 : minPts = 4, 반경 = epsilon

> A low minPts means it will build more clusters from noise, so don't choose it too small.
> [How can I choose eps and minPts (two parameters for DBSCAN algorithm) for efficient results?](https://www.researchgate.net/post/How_can_I_choose_eps_and_minPts_two_parameters_for_DBSCAN_algorithm_for_efficient_results)

|![image](https://user-images.githubusercontent.com/17797922/40961916-f2976c78-68de-11e8-9696-aff088b189ce.png)|![image](https://user-images.githubusercontent.com/17797922/40962055-6e81f6fa-68df-11e8-9617-4846be50bfec.png)|![image](https://user-images.githubusercontent.com/17797922/40962080-7ff69e7c-68df-11e8-8ca7-163465efa6ea.png)|
|-|-|-|
|p=코어포인트<br>. 반경안에 점 4개 존재|P2=경계점(Border point)<br>. 반경안에 점 4개 없음<br>. But, Core point에는 속함|P4=Noise Point<br>. 반경안에 점 4개 없음 <br>.And, Core point에 속하지 않음<br>  (어느군집에도 안속함) |
|![image](https://user-images.githubusercontent.com/17797922/40962067-744f3674-68df-11e8-8602-67df0a739c69.png)|![image](https://user-images.githubusercontent.com/17797922/40962073-7afb11d2-68df-11e8-8b3a-81ad25a242fe.png)|![image](https://user-images.githubusercontent.com/17797922/40961898-e1623212-68de-11e8-8cec-c20eaf8bb93b.png)|
|P=코어포인트<br> P3 = 코어포인트|두 코어 포인트를 연결하여 <br> 하나의 군집으로 처리|**정리**|


--- 
## 코드 

[scikit Manual ](http://scikit-learn.org/stable/auto_examples/cluster/plot_dbscan.html#sphx-glr-auto-examples-cluster-plot-dbscan-py)


```python

from sklearn.cluster import DBSCAN


# Compute DBSCAN 
## fit : Perform DBSCAN clustering from features or distance matrix.
## fit_predict : Performs clustering on X and returns cluster labels.

#db = DBSCAN(eps=0.3, min_samples=10).fit(data)
db = DBSCAN(eps=0.3, min_samples=10)
db.fit(data)
core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
core_samples_mask[db.core_sample_indices_] = True

# 정보 출력
labels = db.labels_

# Number of clusters in labels, ignoring noise if present.
n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

print("클러스터수 : {}".format(n_clusters_))


print("고유한 레이블 : {}".format(np.unique(labels)))
# -1 = 잡음 포인트 

print("클러스터별 포인트 수 : {}".format(np.bincount(labels+1)))
# 잡음 포인트와 클러스터에 속한 포인트수 계산 
# bincount는 음수를 받을 수 없어서 labels에 1을 더함
# 반환값의 첫 번쨰 원소는 압음 포인트의 수 

from collections import Counter
print(Counter(db.labels_))
# 클러스터 ID : 클러스터내 point수


# 시각화 
%matplotlib inline

X = data

# #############################################################################
# Plot result
import matplotlib.pyplot as plt

# Black removed and is used for noise instead.
unique_labels = set(labels)
colors = [plt.cm.Spectral(each)
          for each in np.linspace(0, 1, len(unique_labels))]
for k, col in zip(unique_labels, colors):
    if k == -1:
        # Black used for noise.
        col = [0, 0, 0, 1]

    class_member_mask = (labels == k)

    xy = X[class_member_mask & core_samples_mask]
    plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
             markeredgecolor='k', markersize=1)# 14

    xy = X[class_member_mask & ~core_samples_mask]
    plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=tuple(col),
             markeredgecolor='k', markersize=1)

plt.title('Estimated number of clusters: %d' % n_clusters_)
plt.show()
```

---

## 참고 

- 조대협, [클러스터링 #3 - DBSCAN (밀도 기반 클러스터링)](http://bcho.tistory.com/1205)


- M. Ester, H.-P. Kriegel, J. Sander, X. Xu, and others. A density-based algorithm for discovering clusters in large spatial databases with noise. In Kdd, volume 96, pages 226–231, 1996.


- [Clustering Algorithms:K-means and DBSCAN](https://docs.google.com/presentation/d/1o_rTjzkK7_q672rociNBu11R5dEDlACtrWrfR34FQ3s/edit#slide=id.p): ppt, Python코드 포함








