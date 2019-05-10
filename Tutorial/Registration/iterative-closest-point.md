# [How to use iterative closest point](http://pointclouds.org/documentation/tutorials/iterative_closest_point.php#iterative-closest-point)

- 기능 :  can determine if one PointCloud is just a rigid transformation of another by minimizing the distances between the points of two pointclouds and rigidly transforming them.



ICP(iterative closest point) 알고리즘과 같이 반복적으로 서로 위치가 일치해야 하는 정합 타겟들의 편차를 통계적으로 줄여나가는 기술들이 개발되었다. ICP는 각 장면의 정합 타겟 위치 간 편차를 통계적으로 줄이는 방향으로 각 장면 데이터의 좌표 변환 행렬을 구할 수 있다. 
1) 정합 타겟들 정의
2) 각 장면 스캔 데이터의 정합 타겟 간 최소오차가 되도록 스캔 데이터를 이동/회전 변환
3) 정합 결과 확인 및 2) 번 과정 반복