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


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile ("cloud_cluster_0_white.pcd", *cloud);

  pcl::PointXYZRGB query = cloud->points[3000];


  pcl::search::KdTree<pcl::PointXYZRGB> tree;


  std::vector<int> kd_indices;
  std::vector<float> kd_distances;
  std::vector<int> bf_indices;
  std::vector<float> bf_distances;

  double start, stop;
  double kd_setup;
  double kd_search;
  double bf_setup;
  double bf_search;

  int k = 10;
  float radius = 10;

  if (k > 0)
  {
    start = pcl::getTime ();
    tree.setInputCloud (cloud);
    stop = pcl::getTime ();
    std::cout << "setting up kd tree: " << (kd_setup = stop - start) << std::endl;

    start = pcl::getTime ();
    tree.nearestKSearchT (query, k, kd_indices, kd_distances);
    stop = pcl::getTime ();
    std::cout << "single search with kd tree; " << (kd_search = stop - start) << " :: " << kd_indices[0] << " , " << kd_distances [0] << std::endl;

    pcl::search::BruteForce<pcl::PointXYZRGB> brute_force;
    start = pcl::getTime ();
    brute_force.setInputCloud (cloud);
    stop = pcl::getTime ();
    std::cout << "setting up brute force search: " << (bf_setup = stop - start) << std::endl;

    start = pcl::getTime ();
    brute_force.nearestKSearchT (query, k, bf_indices, bf_distances);
    stop = pcl::getTime ();
    std::cout << "single search with brute force; " << (bf_search = stop - start) << " :: " << bf_indices[0] << " , " << bf_distances [0] << std::endl;
    std::cout << "amortization after searches: " << (kd_setup - bf_setup) / (bf_search - kd_search) << std::endl;
  }
  else
  {
    start = pcl::getTime ();
    tree.setInputCloud (cloud);
    stop = pcl::getTime ();
    std::cout << "setting up kd tree: " << (kd_setup = stop - start) << std::endl;

    start = pcl::getTime ();
    tree.radiusSearch (query, radius, kd_indices, kd_distances, k);
    stop = pcl::getTime ();
    std::cout << "single search with kd tree; " << (kd_search = stop - start) << " :: " << kd_indices[0] << " , " << kd_distances [0] << std::endl;

    pcl::search::BruteForce<pcl::PointXYZRGB> brute_force;
    start = pcl::getTime ();
    brute_force.setInputCloud (cloud);
    stop = pcl::getTime ();
    std::cout << "setting up brute force search: " << (bf_setup = stop - start) << std::endl;

    start = pcl::getTime ();
    brute_force.radiusSearch (query, radius, bf_indices, bf_distances, k);
    stop = pcl::getTime ();
    std::cout << "single search with brute force; " << (bf_search = stop - start) << " :: " << bf_indices[0] << " , " << bf_distances [0] << std::endl;
    std::cout << "amortization after searches: " << (kd_setup - bf_setup) / (bf_search - kd_search) << std::endl;
  }

  if (kd_indices.size () != bf_indices.size ())
  {
    std::cerr << "size of results do not match " <<kd_indices.size () << " vs. " << bf_indices.size () << std::endl;
  }
  else
  {
    std::cerr << "size of result: " <<kd_indices.size () << std::endl;
    for (std::size_t idx = 0; idx < kd_indices.size (); ++idx)
    {
      if (kd_indices[idx] != bf_indices[idx] && kd_distances[idx] != bf_distances[idx])
      {
        std::cerr << "results do not match: " << idx << " nearest neighbor: "
                << kd_indices[idx] << " with distance: " << kd_distances[idx] << " vs. "
                << bf_indices[idx] << " with distance: " << bf_distances[idx] << std::endl;
      }
    }
  }
  return (0);
}
```
