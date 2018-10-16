# Concatenate the points of two Point Clouds

cpp = http://pointclouds.org/documentation/tutorials/concatenate_clouds.php#concatenate-clouds


In this tutorial we will learn how to concatenate the points of two different point clouds. 

요구사항 : The constraint imposed here is that the type and number of fields in the two datasets have to be equal. 

We will also learn how to concatenate the fields (e.g., dimensions) of two different point clouds. 

The constraint imposed here is that the number of points in the two datasets has to be equal.



-p 옵션 

```cpp
cloud_c  = cloud_a;
cloud_c += cloud_b;
```

-f 옵션

```cpp
concatenateFields (cloud_a, n_cloud_b, p_n_cloud_c)
```




---
ps. extract 는 python의 cloud.extract(ind)사용 