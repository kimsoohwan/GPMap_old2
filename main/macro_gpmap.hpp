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

namespace GPMap {

template<typename PointT,
			template<typename> class MeanFunc, 
			template<typename> class CovFunc, 
			template<typename> class LikFunc,
			template <typename, 
						 template<typename> class,
						 template<typename> class,
						 template<typename> class> class InfMethod>
void macro_gpmap(const double		BLOCK_SIZE, 
					  const size_t		NUM_CELLS_PER_AXIS,
					  const bool		FLAG_INDEPENDENT_BCM,
					  const bool		FLAG_DUPLICATE_POINTS,
					  const size_t		MIN_NUM_POINTS_TO_PREDICT,
					  const typename GP::GaussianProcess<float, MeanFunc, CovFunc, LikFunc, InfMethod>::Hyp	&logHyp,				// hyperparameters
					  const typename pcl::PointCloud<PointT>::ConstPtr														&pAllPointCloud,	// observations
					  const float																										gap,					// gap for free points
					  const int																											maxIter,				// number of iterations for training before update
					  const std::string																								&strPCDFilePathWithoutExtension)	// save file path
{
	// log file
	LogFile logFile;

	// times
	CPU_Times	t_update_training;
	CPU_Times	t_update_predict;
	CPU_Times	t_update_combine;

	// get bounding box
	pcl::PointXYZ min_pt, max_pt;
	getMinMaxPointXYZ<PointT>(*pAllPointCloud, min_pt, max_pt);

	// gpmap
	OctreeGPMap<PointT, MeanFunc, CovFunc, LikFunc, InfMethod> gpmap(BLOCK_SIZE, 
																						  NUM_CELLS_PER_AXIS,
																						  FLAG_INDEPENDENT_BCM, 
																						  FLAG_DUPLICATE_POINTS,
																						  MIN_NUM_POINTS_TO_PREDICT);
	// set bounding box
	logFile << "[0] Set bounding box" << std::endl << std::endl;
	gpmap.defineBoundingBox(min_pt, max_pt);

	// set input cloud
	logFile << "[1] Set input cloud" << std::endl << std::endl;
	gpmap.setInputCloud(pAllPointCloud, gap);

	// add points from the input cloud
	logFile << "[2] Add points from the input cloud" << std::endl;
	logFile << gpmap.addPointsFromInputCloud() << std::endl << std::endl;

	// update using GPR
	logFile << "[3] Update using GPR" << std::endl;
	gpmap.update(logHyp, maxIter, t_update_training, t_update_predict, t_update_combine);
	logFile << "- Training hyp: " << t_update_training << std::endl << std::endl;
	logFile << "- Predict GPR:  " << t_update_predict  << std::endl << std::endl;
	logFile << "- Update BCM:   " << t_update_combine  << std::endl << std::endl;

	// save
	logFile << "[4] Save" << std::endl << std::endl;
	gpmap.saveAsPointCloud(strPCDFilePathWithoutExtension);
}

template<typename PointT,
			template<typename> class MeanFunc, 
			template<typename> class CovFunc, 
			template<typename> class LikFunc,
			template <typename, 
						 template<typename> class,
						 template<typename> class,
						 template<typename> class> class InfMethod>
void macro_gpmap(const double		BLOCK_SIZE, 
					  const size_t		NUM_CELLS_PER_AXIS,
					  const bool		FLAG_INDEPENDENT_BCM,
					  const bool		FLAG_DUPLICATE_POINTS,
					  const size_t		MIN_NUM_POINTS_TO_PREDICT,
					  const typename GP::GaussianProcess<float, MeanFunc, CovFunc, LikFunc, InfMethod>::Hyp	&logHyp,				// hyperparameters
					  const std::vector<typename pcl::PointCloud<PointT>::Ptr>											&pointCloudList,	// observations
					  const float																										gap,					// gap for free points
					  const int																											maxIter,				// number of iterations for training before update
					  const std::string																								&strPCDFilePathWithoutExtension)	// save file path
{
	// log file
	LogFile logFile;

	//logFile << "macro_gpmap" << std::endl;	
	//logFile << logHyp.cov(0) << std::endl;
	//logFile << logHyp.cov(1) << std::endl;
	//logFile << logHyp.lik(0) << std::endl;
	//logFile << logHyp.lik(1) << std::endl;

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
	getMinMaxPointXYZ<PointT>(pointCloudList, min_pt, max_pt);

	// gpmap
	OctreeGPMap<PointT, MeanFunc, CovFunc, LikFunc, InfMethod> gpmap(BLOCK_SIZE, 
																						  NUM_CELLS_PER_AXIS,
																						  FLAG_INDEPENDENT_BCM, 
																						  FLAG_DUPLICATE_POINTS,
																						  MIN_NUM_POINTS_TO_PREDICT);
	// set bounding box
	logFile << "[0] Set bounding box" << std::endl << std::endl;
	gpmap.defineBoundingBox(min_pt, max_pt);

	// for each observation
	for(size_t i = 0; i < pointCloudList.size(); i++)
	{
		logFile << "==== Updating the GPMap with the point cloud #" << i << " ====" << std::endl;

		// set input cloud
		logFile << "[1] Set input cloud" << std::endl << std::endl;
		gpmap.setInputCloud(pointCloudList[i], gap);

		// add points from the input cloud
		logFile << "[2] Add points from the input cloud" << std::endl;
		t_elaped = gpmap.addPointsFromInputCloud();
		logFile << t_elaped << std::endl << std::endl;
		t_add_point_total += t_elaped;

		// update using GPR
		logFile << "[3] Update using GPR" << std::endl;
		gpmap.update(logHyp, maxIter, t_update_training, t_update_predict, t_update_combine);
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