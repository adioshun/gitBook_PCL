# [How to use a KdTree to search](http://pointclouds.org/documentation/tutorials/kdtree_search.php#kdtree-search)

> python : https://github.com/strawlab/python-pcl/blob/master/examples/kdtree.py
> https://github.com/strawlab/python-pcl/blob/master/examples/official/kdtree/kdtree_search.py

- KdTree를 이용하여 특정 포인트나 위치에서 k-NN 을 찾는 법을 살펴 본다. `In this tutorial we will go over how to use a KdTree for finding the K nearest neighbors of a specific point or location, `

- and then we will also go over how to find all neighbors within some radius specified by the user (in this case random).

## Theoretical primer

A k-d tree, or k-dimensional tree, is a data structure used in computer science for organizing some number of points in a space with k dimensions. 

It is a binary search tree with other constraints imposed on it. 

K-d trees are very useful for range and nearest neighbor searches. 

For our purposes we will generally only be dealing with point clouds in three dimensions, so all of our k-d trees will be three-dimensional. 

Each level of a k-d tree splits all children along a specific dimension, using a hyperplane that is perpendicular to the corresponding axis. 
- At the root of the tree all children will be split based on the first dimension (i.e. if the first dimension coordinate is less than the root it will be in the left-sub tree and if it is greater than the root it will obviously be in the right sub-tree). 
- Each level down in the tree divides on the next dimension, returning to the first dimension once all others have been exhausted. 

They most efficient way to build a k-d tree is to use a partition method like the one Quick Sort uses to place the median point at the root and everything with a smaller one dimensional value to the left and larger to the right. 

You then repeat this procedure on both the left and right sub-trees until the last trees that you are to partition are only composed of one element.


|![](http://pointclouds.org/documentation/tutorials/_images/2d_kdtree.png)|![](http://pointclouds.org/documentation/tutorials/_images/nn_kdtree.gif)|
|-|-|
|This is an example of a 2-dimensional k-d tree|This is a demonstration of hour the Nearest-Neighbor search works|

> - N 차원의 공간에서 임의의 영역에 포함되는 점들을 효율적으로 찾기 위한 트리.
> - 노드 추가와 검색은 상당히 빠르나, 삭제가 느리다. 즉 동적으로 움직이는 점들을 관리하는 데에는 적당하지 않다.



---

- [포인트 클라우드에서 누가 누가 빠른가? KDTree](https://blog.naver.com/laonple/221207919855)





