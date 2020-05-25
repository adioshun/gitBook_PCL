# [The PCL Registration API](http://pointclouds.org/documentation/tutorials/registration_api.php#registration-api)




## 1. 개요 

- 정의 : The problem of consistently aligning various 3D point cloud data views into a complete model is known as **registration**
- 목적 :  find the **relative positions** and **orientations** of the separately acquired views in a **global coordinate** framework
- 분류 
    - Global Registration : 초기값 불필요 
    - Local Registration : 초기값 필요(Global Regstration사용), eg. ICP registration 
- 절차 : subsequent processing steps such as **segmentation** and object **reconstruction** can be applied.
- The algorithmic work in the PCL registration library is motivated by 
    - 대응점 찾기 `finding correct point correspondences in the given input datasets, `
    - 강체 변환(회전, 이동) 예측 `and estimating rigid transformations that can rotate and translate each individual dataset into a consistent global coordinate framework.`
- 필요 기능 `PCL contains a set of powerful algorithms that allow`
    - 대응점 평가 방법` the estimation of multiple sets of correspondences, `
    - 나쁜 대응점 제거 방 `as well as methods for rejecting bad correspondences, `
    - 변환 파악 `and estimating transformations in a robust manner from them. `
    
    
    
    
## 2. An overview of pairwise registration

- 목표 : transformation matrix (4x4) 계산
    - representing the rotation and translation

![](http://pointclouds.org/documentation/tutorials/_images/block_diagram_single_iteration.jpg)

절차 The computational steps for two datasets are straightforward:

- from a set of points, identify interest points (i.e., keypoints) that best represent the scene in both datasets;
- at each keypoint, compute a feature descriptor;
- from the set of feature descriptors together with their XYZ positions in the two datasets, 
    - estimate a set of correspondences, based on the similarities between features and positions;
- given that the data is assumed to be noisy, not all correspondences are valid, so reject those bad correspondences that contribute negatively to the registration process;
- from the remaining set of good correspondences, estimate a motion transformation.


## 3. Registration modules

### 3.1 Keypoints

A keypoint is an interest point that has a “special property” in the scene, 
- eg) like the corner of a book, or the letter “P” on a book that has written “PCL” on it. 

There are a number of different keypoints available in PCL like 
- NARF, 
- SIFT 
- FAST

또는 모든 포인트를 모두 사용할수도 있다. `Alternatively you can take every point, or a subset, as keypoints as well. `
- 문제는 너무 많은 포인트 수이다. `The problem with “feeding two kinect datasets into a correspondence estimation” directly is that you have 300k points in each frame, so there can be 300k^2 correspondences.`



### 3.2 Feature descriptors

Based on the keypoints found we have to extract features , where we assemble the information and generate vectors to compare them with each other.

Again there is a number of feature options to choose from, for example 
- NARF
- FPFH
- BRIEF
- SIFT

### 3.3 Correspondences estimation

Given two sets of feature vectors coming from two acquired scans we have to find corresponding features to find overlapping parts in the data.

분류 기준 : 사용하는 Feature의 타입에 따라 correspondences찾는 방법이 다름 `Depending on the feature type we can use different methods to find the correspondences.`

#### A. For point matching 
For _point matching_ (using the points’ xyz-coordinates as features) different methods exist for organized and unorganized data:
- brute force matching,
- kd-tree nearest neighbor search (FLANN),
- searching in the image space of organized data, and
- searching in the index space of organized data.

#### B. For feature matching 

For _feature matching_ (not using the points’ coordinates, but certain features) only the following methods exist:
- brute force matching and
- kd-tree nearest neighbor search (FLANN).


In addition to the search, two types of correspondence estimation are distinguished:
-   Direct correspondence estimation (default) searches for correspondences in cloud B for every point in cloud A .  
-   “Reciprocal” correspondence estimation searches for correspondences from cloud A to cloud B, and from B to A and only use the intersection.


### 3.4 Correspondences rejection

질이 않좋은 correspondences는 제거 하여야 한다. `Naturally, not all estimated correspondences are correct. Since wrong correspondences can negatively affect the estimation of the final transformation, they need to be rejected. `

방법 `This could be done using`
- RANSAC or 
- by trimming down the amount and using only a certain percent of the found correspondences.

A special case are one to many correspondences where one point in the model corresponds to a number of points in the source. These could be filtered by using only the one with the smallest distance or by checking for other matchings near by.



### 3.5 Transformation estimation

The last step is to actually compute the transformation.

- evaluate some error metric based on correspondence
- estimate a (rigid) transformation between camera poses (motion estimate) and minimize error metric
- optimize the structure of the points
    - Examples: - SVD for motion estimate; - Levenberg-Marquardt with different kernels for motion estimate;
- use the rigid transformation to rotate/translate the source onto the target, and potentially run an internal ICP loop with either all points or a subset of points or the keypoints
- iterate until some convergence criterion is met


- eg. Normal Distributions Transform (NDT) 

## 4. Example pipelines


### 4.1 Iterative Closest Point

- Search for correspondences.
- Reject bad correspondences.
- Estimate a transformation using the good correspondences.
- Iterate.

### 4.2 Feature based registration

- use SIFT Keypoints (pcl::SIFT…something)
- use FPFH descriptors (pcl::FPFHEstimation) at the keypoints 
    - see our tutorials for that, [like](http://www.pointclouds.org/media/rss2011.html)
- get the FPFH descriptors and estimate correspondences using `pcl::CorrespondenceEstimation`
- reject bad correspondences using one or many of the `pcl::CorrespondenceRejectionXXX methods`
- finally get a transformation as mentioned above


## API Feature Registration 

```cpp
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/iss_3d.h>  //ISS Keypoint
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;


////////////////////////////////////////////////////////////////////////////////
void
estimateKeypoints (const PointCloud<PointXYZ>::Ptr &src, 
                   const PointCloud<PointXYZ>::Ptr &tgt,
                   PointCloud<PointXYZ> &keypoints_src,
                   PointCloud<PointXYZ> &keypoints_tgt)
{
	UniformSampling<PointXYZ> uniform;
	uniform.setRadiusSearch (1);  // 1m

	uniform.setInputCloud (src);
	uniform.filter (keypoints_src);

	uniform.setInputCloud (tgt);
	uniform.filter (keypoints_tgt);
}


////////////////////////////////////////////////////////////////////////////////
double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (! pcl_isfinite((*cloud)[i].x))
			continue;

		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	return resolution;
}

void
estimate_iss_Keypoints (const PointCloud<PointXYZ>::Ptr &src, 
                   const PointCloud<PointXYZ>::Ptr &tgt,
                   PointCloud<PointXYZ> &keypoints_src,
                   PointCloud<PointXYZ> &keypoints_tgt)
{
	// ISS keypoint detector object.
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector_src;
	detector_src.setInputCloud(src);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_src(new pcl::search::KdTree<pcl::PointXYZ>);
	detector_src.setSearchMethod(kdtree_src);
	double resolution_src = computeCloudResolution(src);
	std::cout << "resolution: "<< resolution_src << std::endl;    
	// Set the radius of the spherical neighborhood used to compute the scatter matrix.
	detector_src.setSalientRadius(6 * resolution_src);
	// Set the radius for the application of the non maxima supression algorithm.
	detector_src.setNonMaxRadius(4 * resolution_src);
	// Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
	detector_src.setMinNeighbors(5);
	// Set the upper bound on the ratio between the second and the first eigenvalue.
	detector_src.setThreshold21(0.975);
	// Set the upper bound on the ratio between the third and the second eigenvalue.
	detector_src.setThreshold32(0.975);
	// Set the number of prpcessing threads to use. 0 sets it to automatic.
	detector_src.setNumberOfThreads(4);

	detector_src.compute(keypoints_src);



	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector_tgt;
	detector_tgt.setInputCloud(tgt);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_tgt(new pcl::search::KdTree<pcl::PointXYZ>);
	detector_tgt.setSearchMethod(kdtree_tgt);
	double resolution_tgt = computeCloudResolution(tgt);
	std::cout << "resolution: "<< resolution_tgt << std::endl;    
	// Set the radius of the spherical neighborhood used to compute the scatter matrix.
	detector_tgt.setSalientRadius(6 * resolution_tgt);
	// Set the radius for the application of the non maxima supression algorithm.
	detector_tgt.setNonMaxRadius(4 * resolution_tgt);
	// Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
	detector_tgt.setMinNeighbors(5);
	// Set the upper bound on the ratio between the second and the first eigenvalue.
	detector_tgt.setThreshold21(0.975);
	// Set the upper bound on the ratio between the third and the second eigenvalue.
	detector_tgt.setThreshold32(0.975);
	// Set the number of prpcessing threads to use. 0 sets it to automatic.
	detector_tgt.setNumberOfThreads(4);

	detector_tgt.compute(keypoints_tgt);

}



////////////////////////////////////////////////////////////////////////////////
void
estimateNormals (const PointCloud<PointXYZ>::Ptr &src, 
                 const PointCloud<PointXYZ>::Ptr &tgt,
                 PointCloud<Normal> &normals_src,
                 PointCloud<Normal> &normals_tgt)
{
	NormalEstimation<PointXYZ, Normal> normal_est;
	normal_est.setInputCloud (src);
	normal_est.setRadiusSearch (0.5);  // 50cm
	normal_est.compute (normals_src);

	normal_est.setInputCloud (tgt);
	normal_est.compute (normals_tgt);
}

////////////////////////////////////////////////////////////////////////////////
void
estimateFPFH (const PointCloud<PointXYZ>::Ptr &src, 
              const PointCloud<PointXYZ>::Ptr &tgt,
              const PointCloud<Normal>::Ptr &normals_src,
              const PointCloud<Normal>::Ptr &normals_tgt,
              const PointCloud<PointXYZ>::Ptr &keypoints_src,
              const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
              PointCloud<FPFHSignature33> &fpfhs_src,
              PointCloud<FPFHSignature33> &fpfhs_tgt)
{
	FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud (keypoints_src);
	fpfh_est.setInputNormals (normals_src);
	fpfh_est.setRadiusSearch (1); // 1m
	fpfh_est.setSearchSurface (src);
	fpfh_est.compute (fpfhs_src);

	fpfh_est.setInputCloud (keypoints_tgt);
	fpfh_est.setInputNormals (normals_tgt);
	fpfh_est.setSearchSurface (tgt);
	fpfh_est.compute (fpfhs_tgt);
}

////////////////////////////////////////////////////////////////////////////////
void
findCorrespondences (const PointCloud<FPFHSignature33>::Ptr &fpfhs_src,
                     const PointCloud<FPFHSignature33>::Ptr &fpfhs_tgt,
                     Correspondences &all_correspondences)
{
	CorrespondenceEstimation<FPFHSignature33, FPFHSignature33> est;
	est.setInputCloud (fpfhs_src);
	est.setInputTarget (fpfhs_tgt);
	est.determineReciprocalCorrespondences (all_correspondences);
}

////////////////////////////////////////////////////////////////////////////////
void
rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
                          const PointCloud<PointXYZ>::Ptr &keypoints_src,
                          const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
                          Correspondences &remaining_correspondences)
{
	CorrespondenceRejectorDistance rej;
	rej.setInputSource<PointXYZ> (keypoints_src);
	rej.setInputTarget<PointXYZ> (keypoints_tgt);
	rej.setMaximumDistance (1);    // 1m
	rej.setInputCorrespondences (all_correspondences);
	rej.getCorrespondences (remaining_correspondences);
}

void
computeTransformation (const PointCloud<PointXYZ>::Ptr &keypoints_src,
              			const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
						const CorrespondencesPtr &good_correspondences,
						Eigen::Matrix4f &transform)
{						  
	for (int i = 0; i < good_correspondences->size (); ++i)
		std::cerr << good_correspondences->at (i) << std::endl;

	// Obtain the best transformation between the two sets of keypoints given the remaining correspondences
	pcl::registration::TransformationEstimationSVD<PointXYZ, PointXYZ> trans_est;
	trans_est.estimateRigidTransformation (*keypoints_src, *keypoints_tgt, *good_correspondences, transform);

}

/* ---[ */
int
main (int argc, char** argv)
{
	// Parse the command line arguments for .pcd files
	pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);
	//std::vector<int> p_file_indices;
	pcl::io::loadPCDFile<pcl::PointXYZ> ("bun0.pcd", *src);
	pcl::io::loadPCDFile<pcl::PointXYZ> ("bun4.pcd", *tgt);

	// Get an uniform grid of keypoints
	PointCloud<PointXYZ>::Ptr keypoints_src (new PointCloud<PointXYZ>), 
							keypoints_tgt (new PointCloud<PointXYZ>);

	//estimateKeypoints (src, tgt, *keypoints_src, *keypoints_tgt);
	estimate_iss_Keypoints (src, tgt, *keypoints_src, *keypoints_tgt);
	print_info ("Found %lu and %lu keypoints for the source and target datasets.\n", keypoints_src->points.size (), keypoints_tgt->points.size ());

	// Compute normals for all points keypoint
	PointCloud<Normal>::Ptr normals_src (new PointCloud<Normal>), 
							normals_tgt (new PointCloud<Normal>);
	estimateNormals (src, tgt, *normals_src, *normals_tgt);
	print_info ("Estimated %lu and %lu normals for the source and target datasets.\n", normals_src->points.size (), normals_tgt->points.size ());

	// Compute FPFH features at each keypoint
	PointCloud<FPFHSignature33>::Ptr fpfhs_src (new PointCloud<FPFHSignature33>), 
									fpfhs_tgt (new PointCloud<FPFHSignature33>);
	estimateFPFH (src, tgt, normals_src, normals_tgt, keypoints_src, keypoints_tgt, *fpfhs_src, *fpfhs_tgt);

	// Find correspondences between keypoints in FPFH space
	CorrespondencesPtr all_correspondences (new Correspondences), 
						good_correspondences (new Correspondences);
	findCorrespondences (fpfhs_src, fpfhs_tgt, *all_correspondences);

	// Reject correspondences based on their XYZ distance
	rejectBadCorrespondences (all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);

	Eigen::Matrix4f transform;
	computeTransformation ( keypoints_src, keypoints_tgt, good_correspondences, transform);

	std::cout << transform << std::endl;         

	//Transform the data and write it to disk
	//pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
	//PointCloud<PointXYZ> output;
	//transformPointCloud (*src, *output, transform);
	//savePCDFileBinary ("source_transformed.pcd", *output);
}
/* ]--- */
```

## Sample Consensus Initial Alignment (SAC-IA)

```cpp
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;


////////////////////////////////////////////////////////////////////////////////
void
estimateKeypoints (const PointCloud<PointXYZ>::Ptr &src, 
                   const PointCloud<PointXYZ>::Ptr &tgt,
                   PointCloud<PointXYZ> &keypoints_src,
                   PointCloud<PointXYZ> &keypoints_tgt)
{
	UniformSampling<PointXYZ> uniform;
	uniform.setRadiusSearch (1);  // 1m

	uniform.setInputCloud (src);
	uniform.filter (keypoints_src);

	uniform.setInputCloud (tgt);
	uniform.filter (keypoints_tgt);
}



////////////////////////////////////////////////////////////////////////////////
void
estimateNormals (const PointCloud<PointXYZ>::Ptr &src, 
                 const PointCloud<PointXYZ>::Ptr &tgt,
                 PointCloud<Normal> &normals_src,
                 PointCloud<Normal> &normals_tgt)
{
	NormalEstimation<PointXYZ, Normal> normal_est;
	normal_est.setInputCloud (src);
	normal_est.setRadiusSearch (0.5);  // 50cm
	normal_est.compute (normals_src);

	normal_est.setInputCloud (tgt);
	normal_est.compute (normals_tgt);
}

////////////////////////////////////////////////////////////////////////////////
void
estimateFPFH (const PointCloud<PointXYZ>::Ptr &src, 
              const PointCloud<PointXYZ>::Ptr &tgt,
              const PointCloud<Normal>::Ptr &normals_src,
              const PointCloud<Normal>::Ptr &normals_tgt,
              const PointCloud<PointXYZ>::Ptr &keypoints_src,
              const PointCloud<PointXYZ>::Ptr &keypoints_tgt,
              PointCloud<FPFHSignature33> &fpfhs_src,
              PointCloud<FPFHSignature33> &fpfhs_tgt)
{
	FPFHEstimation<PointXYZ, Normal, FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud (keypoints_src);
	fpfh_est.setInputNormals (normals_src);
	fpfh_est.setRadiusSearch (1); // 1m
	fpfh_est.setSearchSurface (src);
	fpfh_est.compute (fpfhs_src);

	fpfh_est.setInputCloud (keypoints_tgt);
	fpfh_est.setInputNormals (normals_tgt);
	fpfh_est.setSearchSurface (tgt);
	fpfh_est.compute (fpfhs_tgt);
}


/* ---[ */
int
main (int argc, char** argv)
{
	// Parse the command line arguments for .pcd files
	pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);
	//std::vector<int> p_file_indices;
	pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud_bin_0.pcd", *src);
	pcl::io::loadPCDFile<pcl::PointXYZ> ("cloud_bin_2.pcd", *tgt);

	// Get an uniform grid of keypoints
	PointCloud<PointXYZ>::Ptr keypoints_src (new PointCloud<PointXYZ>), 
							keypoints_tgt (new PointCloud<PointXYZ>);

	estimateKeypoints (src, tgt, *keypoints_src, *keypoints_tgt);
	print_info ("Found %lu and %lu keypoints for the source and target datasets.\n", keypoints_src->points.size (), keypoints_tgt->points.size ());

	// Compute normals for all points keypoint
	PointCloud<Normal>::Ptr normals_src (new PointCloud<Normal>), 
							normals_tgt (new PointCloud<Normal>);
	estimateNormals (src, tgt, *normals_src, *normals_tgt);
	print_info ("Estimated %lu and %lu normals for the source and target datasets.\n", normals_src->points.size (), normals_tgt->points.size ());

	// Compute FPFH features at each keypoint
	PointCloud<FPFHSignature33>::Ptr fpfhs_src (new PointCloud<FPFHSignature33>), 
									fpfhs_tgt (new PointCloud<FPFHSignature33>);
	estimateFPFH (src, tgt, normals_src, normals_tgt, keypoints_src, keypoints_tgt, *fpfhs_src, *fpfhs_tgt);

	// Initialize Sample Consensus Initial Alignment (SAC-IA)
	pcl::SampleConsensusInitialAlignment<PointXYZ, PointXYZ, FPFHSignature33> reg;
	reg.setMinSampleDistance (0.05f);
	reg.setMaxCorrespondenceDistance (0.2);
	reg.setMaximumIterations (1000);

	reg.setInputCloud (src);
	reg.setInputTarget (tgt);
	reg.setSourceFeatures (fpfhs_src);
	reg.setTargetFeatures (fpfhs_tgt);

	// Register
	pcl::PointCloud<pcl::PointXYZ> Final;   
	reg.align (Final);

	std::cout << "has converged:" << reg.hasConverged() << " score: " <<   // ?�확???�합?�면 1(True)
	reg.getFitnessScore() << std::endl;

	Eigen::Matrix4f transformation = reg.getFinalTransformation ();
	std::cout << transformation << std::endl;                // 변???�렬 출력 
}
/* ]--- */

```




