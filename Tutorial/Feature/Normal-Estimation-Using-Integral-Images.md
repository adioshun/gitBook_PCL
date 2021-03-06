# [Normal Estimation Using Integral Images](http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php#normal-estimation-using-integral-images)


```cpp
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

// Normal Estimation Using Integral Images
// http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php


int
main ()
{
     // load point cloud
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
     pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud);

     // estimate normals
     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

     pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
     ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
     ne.setMaxDepthChangeFactor(0.02f);
     ne.setNormalSmoothingSize(10.0f);
     ne.setInputCloud(cloud);
     ne.compute(*normals);

     // visualize normals
     pcl::visualization::PCLVisualizer viewer("PCL Viewer");
     viewer.setBackgroundColor (0.0, 0.0, 0.5);
     viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);

     while (!viewer.wasStopped ())
     {
     viewer.spinOnce ();
     }
     return 0;
}
```

[Normal Estimation Using Integral Images-PCL-Python](https://github.com/strawlab/python-pcl/blob/master/examples/official/Features/NormalEstimationUsingIntegralImages.py) : Normal estimation on organized clouds(RGB-D)
