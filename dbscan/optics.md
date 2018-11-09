# OPTICS (Ordering points to identify the clustering structure )

![](https://upload.wikimedia.org/wikipedia/commons/thumb/f/f9/OPTICS.svg/712px-OPTICS.svg.png)

목적 : OPTICS is an algorithm for finding density-based[1] clusters in spatial data. 

논문 :[OPTICS: Ordering Points To Identify the Clustering Structure](http://www.dbs.ifi.lmu.de/Publikationen/Papers/OPTICS.pdf), Mihael Ankerst, Markus M. Breunig, Hans-Peter Kriegel and Jörg Sander. 1999

Its basic idea is similar to DBSCAN,[3] but it addresses one of DBSCAN's major weaknesses: 
- the problem of detecting meaningful clusters in data of varying density. 

In order to do so, the points of the database are (linearly) ordered such that points which are spatially closest become neighbors in the ordering. 

Additionally, a special distance is stored for each point that represents the density that needs to be accepted for a cluster in order to have both points belong to the same cluster. 

OPTIC 사용하는 경우 
- radius 파라미터가 확실치 않은 경우 
- radius 파라미터 need multiple values 인경우 

단점 
- 속도가 느림 : priority heap사용, nearest neighbor queries사용으로 인해 
- OPTICS won’t produce a strict partitioning

This is represented as a **dendrogram**.
- A dendrogram is a diagram representing a tree.

[Density-Based Clustering Exercises + Solutions](http://engdashboard.blogspot.com/2017/09/density-based-clustering-exercises.html): 2017

~~[DBSCAN and OPTICS clustering](https://www.vitavonni.de/blog/201211/2012110201-dbscan-and-optics-clustering.html)~~: 2012

[wiki-pedia](https://en.wikipedia.org/wiki/OPTICS_algorithm)

[OPTICS : Ordering Points To Identify Clustering Algorithm Video | Clustering Analysis - ExcelR](https://www.youtube.com/watch?v=zAbnJ7kERXk): youtube, 20


## 구현 코드 

[sklearn.cluster.OPTICS](http://scikit-learn.org/dev/modules/generated/sklearn.cluster.OPTICS.html) : [Sample-code](https://scikit-learn.org/dev/auto_examples/cluster/plot_optics.html)
- OPTICS is still experimental and only available on master (2018.11.08)
- OPTIC변형이 구현되어 있음 
    - This implementation deviates from the original OPTICS by first performing k-nearest-neighborhood searches on all points to identify core sizes, 
    - then computing only the distances to unprocessed points when constructing the cluster order. 
    - Note that we do not employ a heap to manage the expansion candidates, so the time complexity will be O(n^2).

```
git clone https://github.com/scikit-learn/scikit-learn.git
sudo python setup.py install
make 
"""
import sklearn
print(sklearn.__version__)
"""
```

[OPTICS Clustering](https://github.com/aonghus/optics-cluster) : code-python, 쥬피터 

[CyOPTICS clustering](https://github.com/dvida/cyoptics-clustering): 속도 개선 버젼, cython기


[OPTICS](http://www.chemometria.us.edu.pl/download/optics.py): python 2009 by Brian H. Clowers, Pacific Northwest National Laboratory.

개선 버젼 : Hoodsquare: Modeling and recommending neighborhoods in location-based social networks : [[코드-python]](https://github.com/amyxzhang/OPTICS-Automatic-Clustering)