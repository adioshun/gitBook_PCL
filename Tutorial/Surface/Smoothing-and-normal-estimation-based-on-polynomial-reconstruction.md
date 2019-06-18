# [Smoothing and normal estimation based on polynomial reconstruction](http://pointclouds.org/documentation/tutorials/resampling.php#moving-least-squares)


> Upsampleing의 방법으로 활용 가능 

This tutorial explains how a Moving Least Squares (MLS) surface reconstruction method can be used to smooth and resample noisy data. 


Some of the data irregularities (caused by small distance measurement errors) are very hard to remove using statistical analysis. 

To create complete models, glossy surfaces as well as occlusions in the data must be accounted for. In situations where additional scans are impossible to acquire, a solution is to use a resampling algorithm, which attempts to recreate the missing parts of the surface by higher order polynomial interpolations between the surrounding data points. 


By performing resampling, these small errors can be corrected and the “double walls” artifacts resulted from registering multiple scans together can be smoothed.

![](http://pointclouds.org/documentation/tutorials/_images/resampling_1.jpg)


부가적인 장점은 두개의 점군을 Registration후에 미묘한 정합 에러로 인해 발생 할수 있는 문제를 Smoothing 조정 할수 있다. 결과적으로 Normal생성시 오르쪽 처럼 좀더 잘 생성 된다. `On the left side of the figure above, we see the effect or estimating surface normals in a dataset comprised of two registered point clouds together. Due to alignment errors, the resultant normals are noisy. On the right side we see the effects of surface normal estimation in the same dataset after it has been smoothed with a Moving Least Squares algorithm. 


Plotting the curvatures at each point as a measure of the eigenvalue relationship before and after resampling, we obtain:

---

# [PCL/OpenNI tutorial 2: Cloud processing (Surface smoothing)](http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_2:_Cloud_processing\_\(basic\))

As stated, depth sensors are not very accurate, and the resulting clouds have measurement errors, outliers, holes in surfaces, etc. Surfaces can be reconstructed by means of an algorithm, that iterates through all points and interpolates the data, trying to guess how the original surface was. Like with upsampling, PCL uses the MLS algorithm and class. Performing this step is important, because the resulting cloud's normals will be more accurate.