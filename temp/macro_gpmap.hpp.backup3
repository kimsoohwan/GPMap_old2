#ifndef _MACRO_GPMAP_HPP_
#define _MACRO_GPMAP_HPP_

// STL
#include <string>
#include <vector>

// GP
#include "gp.h"						// LogFile
using GP::LogFile;

// GPMap
#include "util/data_types.hpp"		// PointNormalCloudPtrList
#include "util/timer.hpp"				// CPU_Times
#include "common/common.hpp"			// getMinMaxPointXYZ
#include "octree/octree_gpmap.hpp"	// OctreeGPMap
#include "octree/octree_container.hpp"		// OctreeGPMapContainer
#include "bcm/bcm.hpp"							// BCM
#include "bcm/gaussian.hpp"					// GaussianDistribution

namespace GPMap {

// Train
template<template<typename> class MeanFunc, 
			template<typename> class CovFunc, 
			template<typename> class LikFunc,
			template <typename, 
						 template<typename> class,
						 template<typename> class,
						 template<typename> class> class InfMethod>
void gpmap_training(const double		BLOCK_SIZE, 
						  const size_t		NUM_CELLS_PER_AXIS,
						  const size_t		MIN_NUM_POINTS_TO_PREDICT,
						  typename GP::GaussianProcess<float, MeanFunc, CovFunc, LikFunc, InfMethod>::Hyp	&logHyp,				// hyperparameters
						  const pcl::PointCloud<pcl::PointNormal>::ConstPtr	&pAllPointNormalCloud,	// observations
						  const float														gap,							// gap
						  const int															maxIter,						// number of iterations for training before update, 100
						  const int															numRandomBlocks)			// number of randomly selected blocks (<=0 for all), 100
{
	// log file
	LogFile logFile;

	// gpmap with Gaussian Distribution leaf nodes
	pcl::PointXYZ min_pt, max_pt;
	getMinMaxPointXYZ<pcl::PointNormal>(*pAllPointNormalCloud, min_pt, max_pt);

	// gpmap
	typedef OctreeGPMapContainer<GaussianDistribution>	LeafT;
	typedef OctreeGPMap<MeanFunc, CovFunc, LikFunc, InfMethod, LeafT> OctreeGPMapT;
	const bool FLAG_INDEPENDENT_TEST_POSITIONS = true;
	OctreeGPMapT gpmap(BLOCK_SIZE, NUM_CELLS_PER_AXIS, MIN_NUM_POINTS_TO_PREDICT, FLAG_INDEPENDENT_TEST_POSITIONS);

	// set bounding box
	logFile << "[0] Set bounding box" << std::endl << std::endl;
	gpmap.defineBoundingBox(min_pt, max_pt);

	// set input cloud
	logFile << "[1] Set input cloud" << std::endl << std::endl;
	gpmap.setInputCloud(pAllPointNormalCloud, gap);

	// add points from the input cloud
	logFile << "[2] Add points from the input cloud" << std::endl;
	logFile << gpmap.addPointsFromInputCloud() << std::endl << std::endl;

	// train
	logFile << "[4] Learning hyperparameters" << std::endl;
	logFile << "- Train - Max Iterations: "		<< maxIter				<< std::endl;
	logFile << "- Train - Num Random Blocks: "	<< numRandomBlocks << std::endl;
	GP::DlibScalar nlZ = gpmap.train(logHyp, maxIter, numRandomBlocks);
	logFile << "- nlZ: " << nlZ << std::endl << std::endl;

	// trained hyperparameters
	int j = 0;
	for(int i = 0; i < logHyp.mean.size(); i++)		logFile << "- mean[" << i << "] = " << expf(logHyp.mean(i)) << std::endl;
	for(int i = 0; i < logHyp.cov.size();  i++)		logFile << "- cov["  << i << "] = " << expf(logHyp.cov(i))  << std::endl;
	for(int i = 0; i < logHyp.lik.size();  i++)		logFile << "- lik["  << i << "] = " << expf(logHyp.lik(i))  << std::endl;
}


// Batch
template<template<typename> class MeanFunc, 
			template<typename> class CovFunc, 
			template<typename> class LikFunc,
			template <typename, 
						 template<typename> class,
						 template<typename> class,
						 template<typename> class> class InfMethod>
void gpmap_batch(const double		BLOCK_SIZE, 
					  const size_t		NUM_CELLS_PER_AXIS,
					  const size_t		MIN_NUM_POINTS_TO_PREDICT,
					  const typename GP::GaussianProcess<float, MeanFunc, CovFunc, LikFunc, InfMethod>::Hyp	&logHyp,	// hyperparameters
					  const pcl::PointCloud<pcl::PointNormal>::ConstPtr		&pAllPointNormalCloud,							// observations
					  const float															gap,													// gap
					  const int																maxIterBeforeUpdate,					// number of iterations for training before update
					  const std::string													&strPCDFilePathWithoutExtension)	// save file path
{
	// log file
	LogFile logFile;

	// times
	CPU_Times	t_update_training;
	CPU_Times	t_update_predict;
	CPU_Times	t_update_combine;

	// get bounding box
	pcl::PointXYZ min_pt, max_pt;
	getMinMaxPointXYZ<pcl::PointNormal>(*pAllPointNormalCloud, min_pt, max_pt);

	// gpmap with Gaussian Distribution leaf nodes
	typedef OctreeGPMapContainer<GaussianDistribution>	LeafT;
	typedef OctreeGPMap<MeanFunc, CovFunc, LikFunc, InfMethod, LeafT> OctreeGPMapT;
	const bool FLAG_INDEPENDENT_TEST_POSITIONS = true;
	OctreeGPMapT gpmap(BLOCK_SIZE, NUM_CELLS_PER_AXIS, MIN_NUM_POINTS_TO_PREDICT, FLAG_INDEPENDENT_TEST_POSITIONS);

	// set bounding box
	logFile << "[0] Set bounding box" << std::endl << std::endl;
	gpmap.defineBoundingBox(min_pt, max_pt);

	// set input cloud
	logFile << "[1] Set input cloud" << std::endl << std::endl;
	gpmap.setInputCloud(pAllPointNormalCloud, gap);

	// add points from the input cloud
	logFile << "[2] Add points from the input cloud" << std::endl;
	logFile << gpmap.addPointsFromInputCloud() << std::endl << std::endl;

	// update using GPR
	logFile << "[3] Update using GPR" << std::endl;
	gpmap.update(logHyp, maxIterBeforeUpdate, t_update_training, t_update_predict, t_update_combine);
	logFile << "- Training hyp: " << t_update_training << std::endl << std::endl;
	logFile << "- Predict GPR:  " << t_update_predict  << std::endl << std::endl;
	logFile << "- Update BCM:   " << t_update_combine  << std::endl << std::endl;

	// save
	logFile << "[4] Save" << std::endl << std::endl;
	gpmap.saveAsPointCloud(strPCDFilePathWithoutExtension);
}

// Incremental
template<template<typename> class MeanFunc, 
			template<typename> class CovFunc, 
			template<typename> class LikFunc,
			template <typename, 
						 template<typename> class,
						 template<typename> class,
						 template<typename> class> class InfMethod>
void gpmap_incremental(const double		BLOCK_SIZE,							// block size
							  const size_t		NUM_CELLS_PER_AXIS,				// number of cells per each axie
							  const size_t		MIN_NUM_POINTS_TO_PREDICT,		// min number of points to predict
							  const bool		FLAG_INDEPENDENT_BCM,			// independent BCM or BCM
							  const typename GP::GaussianProcess<float, MeanFunc, CovFunc, LikFunc, InfMethod>::Hyp	&logHyp,				// hyperparameters
							  const std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr>	&pointNormalCloudList,	// observations
							  const float																gap,							// gap
							  const int																	maxIterBeforeUpdate,					// number of iterations for training before update
							  const std::string														&strPCDFilePathWithoutExtension)	// save file path
{
	// log file
	LogFile logFile;

	// times
	CPU_Times	t_elaped;
	CPU_Times	t_update_training;
	CPU_Times	t_update_predict;
	CPU_Times	t_update_combine;

	CPU_Times	t_add_point_total;
	CPU_Times	t_update_training_total;
	CPU_Times	t_update_predict_total;
	CPU_Times	t_update_combine_total;
	
	t_add_point_total.clear();
	t_update_training_total.clear();
	t_update_predict_total.clear();
	t_update_combine_total.clear();

	// get bounding box
	pcl::PointXYZ min_pt, max_pt;
	getMinMaxPointXYZ<pcl::PointNormal>(pointNormalCloudList, min_pt, max_pt);

	// gpmap with BCM leaf nodes
	typedef OctreeGPMapContainer<BCM>	LeafT;
	typedef OctreeGPMap<MeanFunc, CovFunc, LikFunc, InfMethod, LeafT> OctreeGPMapT;
	OctreeGPMapT gpmap(BLOCK_SIZE, NUM_CELLS_PER_AXIS, MIN_NUM_POINTS_TO_PREDICT, FLAG_INDEPENDENT_BCM);

	// set bounding box
	logFile << "[0] Set bounding box" << std::endl << std::endl;
	gpmap.defineBoundingBox(min_pt, max_pt);

	// for each observation
	for(size_t i = 0; i < pointNormalCloudList.size(); i++)
	{
		logFile << "==== Updating the GPMap with the point cloud #" << i << " ====" << std::endl;

		// set input cloud
		logFile << "[1] Set input cloud" << std::endl << std::endl;
		gpmap.setInputCloud(pointNormalCloudList[i], gap);

		// add points from the input cloud
		logFile << "[2] Add points from the input cloud" << std::endl;
		t_elaped = gpmap.addPointsFromInputCloud();
		logFile << t_elaped << std::endl << std::endl;
		t_add_point_total += t_elaped;

		// update using GPR
		logFile << "[3] Update using GPR" << std::endl;
		gpmap.update(logHyp, maxIterBeforeUpdate, t_update_training, t_update_predict, t_update_combine);
		logFile << "- Training hyp: " << t_update_training << std::endl << std::endl;
		logFile << "- Predict GPR:  " << t_update_predict  << std::endl << std::endl;
		logFile << "- Update BCM:   " << t_update_combine   << std::endl << std::endl;
		t_update_training_total		+= t_update_training;
		t_update_predict_total		+= t_update_predict;
		t_update_combine_total		+= t_update_combine;

		// save
		logFile << "[4] Save" << std::endl << std::endl;
		std::stringstream ss;
		ss << strPCDFilePathWithoutExtension << "_upto_" << i;
		gpmap.saveAsPointCloud(ss.str());

		// last
		//if(i == pointNormalCloudList.size() - 1) gpmap.saveAsPointCloud(strPCDFilePathWithoutExtension);
	}

	// total time
	logFile << "============= Total Time =============" << std::endl;
	logFile << "- Total"					<< std::endl << t_add_point_total + t_update_training_total + t_update_predict_total + t_update_combine_total << std::endl << std::endl;
	logFile << "- Total: Add point"	<< std::endl << t_add_point_total << std::endl << std::endl;
	logFile << "- Total: Update"		<< std::endl << t_update_training_total + t_update_predict_total + t_update_combine_total << std::endl << std::endl;
	logFile << "- Total: Update - Training hyp: " << t_update_training_total << std::endl << std::endl;
	logFile << "- Total: Update - Predict GPR:  " << t_update_predict_total  << std::endl << std::endl;
	logFile << "- Total: Update - Update BCM:   " << t_update_combine_total  << std::endl << std::endl;
}

}

#endif