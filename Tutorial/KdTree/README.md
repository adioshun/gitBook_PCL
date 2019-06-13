### [page365 블로그](http://blog.daum.net/pg365/140)

k-d tree(k-dimensional tree)는 k-차원의 공간 내에서 점들을 구성하기 위한 공간 분할 자료구조로 binary tree의 특수한 경우입니다. K-d tree는 다차원의 탐색 키를 사용하는 탐색 알고리즘에 유용하게 사용됩니다. 예를 들자면, SIFT나 SURF로 추출한 특징점들은 높은 차원의 벡터로 표시되는데 이 특징점들을 DB에 저장된 특징점들과 비교하여 유사도가 가장 높은 것을 찾기위해 사용됩니다.

 

Insertion

k-d tree에 노드를 삽입하는 방법은 binary search tree와 유사하게 동작합니다. binary search tree의 insertion을 참고하면 이해가 쉽습니다.

 

Search

k-d tree에서 노드를 검색하는 방법은 주로 두 가지 방법을 사용합니다. 첫번째가 range search 방법으로 찾고자 하는 키 값의 범위를 정하고 이 범위에 포함되는 노드를 찾게 됩니다. 이 탐색 방법도 binary search tree에서 탐색 방법과 유사합니다. 두번째는 nearest neighbour search 방법으로 주어진 키 값에 가장 근접한 노드를 찾는 것으로 구현이 좀 까다롭습니다. 여기서 사용한 방법은 hyper cube를 사용하여 노드의 양쪽으로 공간을 분할해 가면서 찾는 방법입니다.

 

Deletion

노드를 tree에서 직접 삭제하는 것은 좀 까다롭습니다.  좀 쉬운 대안으로 노드에 삭제 플래그를 만들고 노드가 삭제될 때 삭제 플래그를 true로 만드는 방법이 있을 것입니다. 여기서는 노드의 삭제는 구현되어있지 않습니다.






---

[ppt_search](http://www.pointclouds.org/assets/rss2011/06_search.pdf): RSS2011
[Introductory guide to Information Retrieval using kNN and KDTree](https://www.analyticsvidhya.com/blog/2017/11/information-retrieval-using-kdtree/)
[K-d 트리](http://3dmpengines.tistory.com/1352)
[kd-tree](http://www.snisni.net/98)
[알고리즘 배우기 - Week5. Geometric Search - kd trees](http://algs4.tistory.com/68)
[코세라](https://ko.coursera.org/lecture/ml-clustering-and-retrieval/kd-tree-representation-S0gfp)

