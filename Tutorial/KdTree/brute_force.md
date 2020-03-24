```cpp


#include <vector>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/brute_force.h>


//https://github.com/PointCloudLibrary/pcl/blob/master/apps/src/test_search.cpp

int
main (int argc, char** argv)
{


  // *.PCD 파일 읽기 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile ("cloud_cluster_0_white.pcd", *cloud);

  pcl::search::BruteForce<pcl::PointXYZRGB> brute_force;
  brute_force.setInputCloud (cloud);

//기준점(searchPoint) 설정 방법(3000번째 포인트)
  pcl::PointXYZRGB searchPoint = cloud->points[3000];

  int K = 10;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  if ( brute_force.nearestKSearchT (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    //시각적 확인을 위하여 색상 변경 (0,255,0)
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
    {
      cloud->points[pointIdxNKNSearch[i]].r = 0;
      cloud->points[pointIdxNKNSearch[i]].g = 255;
      cloud->points[pointIdxNKNSearch[i]].b = 0;
    }
  }

  // 탐색된 점의 수 출력 
  std::cout << "K = 10 ：" << pointIdxNKNSearch.size() << std::endl;


  float radius = 0.02; //탐색할 반경 설정(Set the search radius)
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  

  if ( brute_force.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  {
    //시각적 확인을 위하여 색상 변경 (0,0,255)
    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
      for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
        cloud->points[pointIdxRadiusSearch[i]].r = 0;
        cloud->points[pointIdxRadiusSearch[i]].g = 0;
        cloud->points[pointIdxRadiusSearch[i]].b = 255;
        }
  }

  // 탐색된 점의 수 출력 
  std::cout << "Radius 0.02 nearest neighbors: " << pointIdxRadiusSearch.size() << std::endl;

  // 생성된 포인트클라우드 저장 
  pcl::io::savePCDFile<pcl::PointXYZRGB>("brute_force_AllinOne.pcd", *cloud);

  return 0;
}

```
