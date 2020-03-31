# [3D Object Recognition based on Correspondence Grouping](http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping)

This tutorial aims at explaining how to perform 3D Object Recognition based on the pcl_recognition module. 

Specifically, it explains how to use Correspondence Grouping algorithms in order to cluster the set of point-to-point correspondences obtained after the 3D descriptor matching stage into model instances that are present in the current scene. 

For each cluster, representing a possible model instance in the scene, the Correspondence Grouping algorithms also output the transformation matrix identifying the 6DOF pose estimation of that model in the current scene.

```cpp
#include <stdlib.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/kdtree/kdtree_flann.h>  
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
 
typedef pcl::PointXYZ PointType; 
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;
 
std::string model_filename_;
std::string scene_filename_;
 
// 시각화 설정 
bool show_keypoints_(true);
bool show_correspondences_(true);
//Algorithm params
float model_ss_(0.0114425);//7.5f); //
float scene_ss_(0.0305134);//20.0f);//
float rf_rad_(0.0152567);//10.0f);  //
float descr_rad_(0.0228851);//15.0f);//
float cg_size_(0.0152567);//10.0f);  //
float cg_thresh_(5.0f);

int main(int argc, char *argv[])
{

  // 3D Object Recognition based on Correspondence Grouping
  // http://pointclouds.org/documentation/tutorials/correspondence_grouping.php#correspondence-grouping
  // 

  // PCL/OpenNI tutorial 5: 3D object recognition (pipeline)
  // http://robotica.unileon.es/index.php/PCL/OpenNI_tutorial_5:_3D_object_recognition_(pipeline)

	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>()); //Define model point cloud types, and feature points
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
 
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());
 

  // 파일 읽기 
	pcl::io::loadPCDFile("milk.pcd", *model);
	pcl::io::loadPCDFile("milk_cartoon_all_small_clorox.pcd", *scene);
  // https://github.com/PointCloudLibrary/pcl/blob/master/test/milk.pcd?raw=true
  // https://github.com/PointCloudLibrary/pcl/blob/master/test/milk_cartoon_all_small_clorox.pcd?raw=true
 
	// 노멀 계산 Compute Normals
	//Calculate the normals for each point of the model and scene cloud, using the 10 nearest neighbors at each point 
  //(this parameter seems pretty good, a lot of data sets, not just the data sets provided).
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch(10);
	norm_est.setNumberOfThreads(4); //User Error 1001에러 해결용 : argument to num_threads clause must be positive
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals); 
	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);
 
	// 다운 샘플링 Downsample Clouds to Extract keypoints
	// It then samples each cloud to find a small number of key points and 
  // then correlates these key points to the 3D descriptor for performing key-point matching and determining point-to-point correspondence.

	pcl::UniformSampling<PointType> uniform_sampling;
	uniform_sampling.setInputCloud(model);
	uniform_sampling.setRadiusSearch(model_ss_);
	uniform_sampling.filter(*model_keypoints);
	std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

 	uniform_sampling.setInputCloud(scene);
	uniform_sampling.setRadiusSearch(scene_ss_);
	uniform_sampling.filter(*scene_keypoints);
	std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;
	
	// Compute Descriptor for keypoints
	// The next phase involves correlating the 3D descriptor to each model and scene key points.
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch(descr_rad_);

	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);
	descr_est.setNumberOfThreads(4);
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);
 
	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);
	descr_est.setNumberOfThreads(4);
	descr_est.setSearchSurface(scene);
	descr_est.compute(*scene_descriptors);
 	
	//Find Model-Scene Correspondences with KdTree
	//Now we need to determine the point-to-point correspondence between the model descriptor and the scene descriptor.
	//For this purpose, the program is used：pcl：`KdTreeFLANN <pcl :: KdTreeFLANN>`，Its input cloud has been set to the cloud containing the model descriptor.
	//For each descriptor associated with a scene keypoint, it efficiently finds the most similar model descriptor based on Euclidean distance,
	//And add that pair to：pcl：`Correspondences <pcl :: Correspondences>`vector
	//(Only if the two descriptors are sufficiently similar, ie their squared distance is less than the threshold, set to 0.25).
	//
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());
 
	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud(model_descriptors);
 
	////  For each scene keypoint descriptor, 
	////find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < scene_descriptors->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!std::isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
 
	//
	//  Actual Clustering
	//The last stage of the pipeline is the actual clustering of previously found correspondences
	//The default algorithm is：pcl：`Hough3DGrouping <pcl::Hough3DGrouping>`，It is based on the Hough Voting process
	//Note that this algorithm needs to associate a local reference frame (LRF) for each keypoint belonging to the cloud, these keypoints being passed as parameters!
	//In this example, we use before calling clustering algorithm：pcl：`BOARDLocalReferenceFrameEstimation <pcl::BOARDLocalReferenceFrameEstimation> estimator
	//显式计算LRF集。
	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;
 
	//  Using Hough3D
  // F. Tombari and L. Di Stefano: “Object recognition in 3D scenes with occlusions and clutter by Hough voting”, 4th Pacific-Rim Symposium on Image and Video Technology, 2010.
  //  Compute (Keypoints) Reference Frames only for Hough
  pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
  pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

  pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
  rf_est.setFindHoles(true);
  rf_est.setRadiusSearch(rf_rad_);

  rf_est.setInputCloud(model_keypoints);
  rf_est.setInputNormals(model_normals);
  rf_est.setSearchSurface(model);
  rf_est.compute(*model_rf);

  rf_est.setInputCloud(scene_keypoints);
  rf_est.setInputNormals(scene_normals);
  rf_est.setSearchSurface(scene);
  rf_est.compute(*scene_rf);

	//  Clustering
  pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
  clusterer.setHoughBinSize(cg_size_);
  clusterer.setHoughThreshold(cg_thresh_);
  clusterer.setUseInterpolation(true);
  clusterer.setUseDistanceWeight(false);

  clusterer.setInputCloud(model_keypoints);
  clusterer.setInputRf(model_rf);
  clusterer.setSceneCloud(scene_keypoints);
  clusterer.setSceneRf(scene_rf);
  clusterer.setModelSceneCorrespondences(model_scene_corrs);

  //clusterer.cluster (clustered_corrs);
  clusterer.recognize(rototranslations, clustered_corrs);


	/*
  It is not necessary to calculate the LRF explicitly before calling the clustering algorithm. If the cloud extracted to the clustering algorithm has no associated set of LRFs,
	Hough 3D Grouping automatically calculates them before performing clustering. In particular, this can happen when invoking the identification (or cluster) method without setting the LRF：
	In this case, you need to specify the radius of LRF clustering algorithm as an additional parameter (setLocalRfSearchRadius using Method)。
	*/

	//
	//  Output results
	//
	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;
 
		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);
 
		printf("\n");
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		printf("\n");
		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}
 
	//  Visualization
	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	viewer.addPointCloud(scene, "scene_cloud");//可视化场景点云
 
	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());
 
 
	if (show_correspondences_ || show_keypoints_)
	{
		  //We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
		pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
 
		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
		viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
	}
 
	if (show_keypoints_)
	{
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
		viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
 
		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
	}
 
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);
 
		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;
 
		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());
 
		if (show_correspondences_)
		{
			for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;
				PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
				PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);
 
				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
			}
		}
	}
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
 
	system("pause");
	return (0);
}


```
