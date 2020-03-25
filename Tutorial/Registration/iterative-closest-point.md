# [How to use iterative closest point](http://pointclouds.org/documentation/tutorials/iterative_closest_point.php#iterative-closest-point)


여러개의 점군들을 ICP를 이용하여 차례대로 정합 하는 방법을 다루고 있음`This document demonstrates using the Iterative Closest Point algorithm in order to incrementally register a series of point clouds two by two.`

키 아이디어는 첫번째 점군을 기준으로 나머지 점군의좌표계를 변경 `The idea is to transform all the clouds in the first cloud’s frame.`

This is done by finding the best transform between each consecutive cloud, and accumulating these transforms over the whole set of clouds.

Your data set should consist of clouds that have been roughly pre-aligned in a common frame (e.g. in a robot’s odometry or map frame) and overlap with one another.

We provide a set of clouds at github.com/PointCloudLibrary/data/tree/master/tutorials/pairwise/.



