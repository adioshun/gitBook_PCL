# [Removing outliers using a Conditional or RadiusOutlier removal](http://pointclouds.org/documentation/tutorials/remove_outliers.php#remove-outliers)

## Conditional removal

x,y,z값을 GT, GE, LT, LE, EQ.조건에 맞추어 제거 


## Radius Outlier removal

- 가장 간단한 방식 `The radius-based outlier removal is the simplest method of all.`

- 지정된 반경안에 지정된 수 만의 포인트가 없을 경우 outlier로 판단 `You must specify a search radius and the minimum number of neighbors than a point must have to avoid being labelled as outlier`


> **[중요]** 현재 정상 동작 하지 않는다고 합니다. 파라미터를 바꾸어도 결과가 '0'이라고 하네요. [[참고]](https://github.com/strawlab/python-pcl/issues/211) - 2018.06.11

## Statistical Outlier removal


> Removing outliers using a StatisticalOutlierRemoval filter 챕터 참고 

