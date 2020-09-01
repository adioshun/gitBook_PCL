이곳은 [PCL Tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/index.html#)의 내용을 간략하게 번역 정리 하기 위해 만들었습니다. 

> Radu Bogdan Rusu, Steve Cousins,"[3D is here: Point Cloud Library (PCL)](http://www.pointclouds.org/assets/pdf/pcl_icra2011.pdf)", ICRA2011

## PCL Modules 

- pcl_filters - 3D 점군 데이터에서 이상값과 노이즈 제거 등의 필터링
- pcl_features - 점군 데이터로부터 3D 특징 추정 (feature estimation) 을 위한 수많은 자료 구조와 방법들 
- pcl_keypoints - Keypoint (or interest point) 을 검출하는 알고리즘 구현 (BRISK, Harris Corner, NARF, SIFT, SUSAN 등)
- pcl_registration - 여러 데이터셋을 합쳐 큰 모델로 만드는 registration 작업 (ICP 등)
- pcl_kdtree - 빠른 최근거리 이웃을 탐색하는 FLANN 라이브러리를 사용한 kdtree 자료 구조
- pcl_octree - 점군 데이터로부터 계층 트리 구조를 구성하는 방법
- pcl_segmentation - 점군으로부터 클러스터들로 구분하는 알고리즘들
- pcl_sample_consensus - 선, 평면, 실린더 등의 모델 계수 추정을 위한 RANSAC 등의 알고리즘들
- pcl_surface - 3D 표면 복원 기법들 (meshing, convex hulls, Moving Least Squares 등)
- pcl_range_image - range image (or depth map) 을 나타내고 처리하는 방법
- pcl_io - OpenNI 호환 depth camera 로부터 점군 데이터를 읽고 쓰는 방법
- pcl_visualization - 3D 점군 데이터를 처리하는 알고리즘의 결과를 시각화



