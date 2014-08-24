#ifndef _GPMAP_TO_OCTOMAP_HPP_
#define _GPMAP_TO_OCTOMAP_HPP_

// STL
#include <string>
#include <vector>
#include <algorithm>					// min, max
#include <fstream>
#include <limits>						// std::numeric_limits<float>::digits10
#include <cmath>						// logf

// PCL
#include <pcl/point_types.h>		// pcl::PointXYZ, pcl::Normal, pcl::PointNormal
#include <pcl/point_cloud.h>		// pcl::PointCloud

// Octomap
#include <octomap/octomap.h>
#include <octomap/octomap_timing.h>
#include <octomap/ColorOcTree.h>

// OpenGP
#include "GP.h"						// DlibScalar

// GPMap
#include "util/data_types.hpp"	// PointXYZCloud
#include "util/timer.hpp"			// Times
#include "util/color.hpp"			// Color
#include "plsc/plsc.hpp"			// PLSC
namespace GPMap {

//enum OctomapColor {
//	NO_COLOR = 0,
//	COLOR = 1
//};

class NO_COLOR {};
class COLOR {};
template <typename ColorT> struct OctomapType {};
template <> struct OctomapType<NO_COLOR>	{ typedef octomap::OcTree			OctomapT; };
template <> struct OctomapType<COLOR>		{ typedef octomap::ColorOcTree	OctomapT; };

template <typename ColorT>
class Octomap
{
protected:
	typedef typename OctomapType<ColorT>::OctomapT	MyOctomapT;
public:
	/** @brief Constructor */
	Octomap(const double resolution,
			  const bool	FLAG_SIMPLE_UPDATE = false)
		:	m_pOctree(new MyOctomapT(resolution)),
			FLAG_SIMPLE_UPDATE_(FLAG_SIMPLE_UPDATE)
	{
	}

	/** @brief Constructor */
	Octomap(const double				resolution,
			  const std::string		strFileName,
			  const bool				FLAG_SIMPLE_UPDATE = false)
		:	m_pOctree(new MyOctomapT(resolution)),
			FLAG_SIMPLE_UPDATE_(FLAG_SIMPLE_UPDATE)
	{
		// load octomap
		m_pOctree->readBinary(strFileName);
	}

	/** @brief			Constructor 
	  * @details		Load the octomap from a point cloud
	  */
	Octomap(const double										resolution,
			  const pcl::PointCloud<pcl::PointNormal>	&pointCloud,
			  const float										minMeanThreshold,
			  const float										maxVarThreshold,
			  const bool										FLAG_SIMPLE_UPDATE = false);

	/** @brief			Constructor 
	  * @details		Load the octomap from a point cloud
	  */
	Octomap(const double										resolution,
			  const pcl::PointCloud<pcl::PointNormal>	&pointCloud,
			  const float										minMeanThreshold,
			  const float										maxVarThreshold,
			  const float										minVarRangeForColor,
			  const float										maxVarRangeForColor,
			  const bool										FLAG_SIMPLE_UPDATE = false)
		:	m_pOctree(new MyOctomapT(resolution)),
			FLAG_SIMPLE_UPDATE_(FLAG_SIMPLE_UPDATE)
	{
		// get min, max
		float minMean, maxMean, minVar, maxVar;
		getMinMaxMeanVarOfOccupiedCells(pointCloud, minMeanThreshold, maxVarThreshold, minMean, maxMean, minVar, maxVar);

		// color
		Color color(minVarRangeForColor, maxVarRangeForColor);
		unsigned char r, g, b;

		// for each point
		for(size_t i = 0; i < pointCloud.points.size(); i++)
		{
			const pcl::PointNormal &point = pointCloud.points[i];
			if(point.normal_x >= minMeanThreshold &&	// mean
				point.normal_y <= maxVarThreshold)		// var
			{
				// set occupied
				m_pOctree->updateNode(static_cast<double>(point.x), 
											 static_cast<double>(point.y), 
											 static_cast<double>(point.z),
											 true);

				// color based on the variance
				color.rgb(point.normal_y, r, g, b);

				// random colors
				//r = rand() % 256;
				//g = rand() % 256;
				//b = rand() % 256;

				// set color
				m_pOctree->setNodeColor(point.x, point.y, point.z, r, g, b);
			}
		}
	}

	/** @brief	Update a node of the octomap
	  * @return	Elapsed time (user/system/wall cpu times)
	  */
	inline void updateNode(const double x, const double y, const double z, const float log_odds_update, bool lazy_eval = false)
	{
		m_pOctree->updateNode(x, y, z, log_odds_update, lazy_eval);
	}

	/** @brief	Update a node of the octomap
	  * @return	Elapsed time (user/system/wall cpu times)
	  */
	inline void updateNode(const double x, const double y, const double z, bool occupied, bool lazy_eval = false)
	{
		m_pOctree->updateNode(x, y, z, occupied, lazy_eval);
	}

	/** @brief	Update the octomap with a point cloud
	  * @return	Elapsed time (user/system/wall cpu times)
	  */
	template <typename PointT1, typename PointT2>
	CPU_Times
	update(const typename pcl::PointCloud<PointT1>	&pointCloud,
			 const PointT2										&sensorPosition,
			 const double										maxrange = -1)
	{
		// robot position
		octomap::point3d robotPosition(sensorPosition.x, sensorPosition.y, sensorPosition.z);

		// point cloud
		octomap::Pointcloud pc;
		for(size_t i = 0; i < pointCloud.size(); i++)
			pc.push_back(pointCloud.points[i].x, pointCloud.points[i].y, pointCloud.points[i].z);

		// timer - start
		CPU_Timer timer;

		// update
		if (FLAG_SIMPLE_UPDATE_)	m_pOctree->insertPointCloudRays(pc, robotPosition, maxrange);
		else								m_pOctree->insertPointCloud(pc, robotPosition, maxrange);

		// timer - end
		CPU_Times elapsed = timer.elapsed();

		// memory
		std::cout << "memory usage: "		<< m_pOctree->memoryUsage()		<< std::endl;
		std::cout << "leaf node count: " << m_pOctree->getNumLeafNodes() << std::endl;
		//if(tree->memoryUsage() > 900000000)
		if(m_pOctree->memoryUsage() > 500000000)
		{
			m_pOctree->toMaxLikelihood();
			m_pOctree->prune();
			std::cout << "after pruned - memory usage: "		<< m_pOctree->memoryUsage()		<< std::endl;
			std::cout << "after pruned - leaf node count: " << m_pOctree->getNumLeafNodes() << std::endl;
		}

		// return the elapsed time
		return elapsed;
	}

	/** @brief	Convert a GPMap to an Octree or a ColorOctree based on PLSC */
	void GPMap2Octomap(const pcl::PointCloud<pcl::PointNormal>	&pointCloud);

	/** @brief	Convert a GPMap to an Octree based on PLSC */
	void GPMap2Octomap(const pcl::PointCloud<pcl::PointNormal>	&pointCloud,
							 const float										minVarRangeForColor,
							 const float										maxVarRangeForColor);


	/** @brief	Save the octomap as a binary file */
	bool save(const std::string &strFileNameWithoutExtension)
	{
		// check
		if(!m_pOctree) return false;

		// file name
		std::string strFileName;

		// *.ot
		if(FLAG_SIMPLE_UPDATE_)		strFileName = strFileNameWithoutExtension + "_simple.ot";
		else								strFileName = strFileNameWithoutExtension + ".ot";
		m_pOctree->write(strFileName);

		// *.bt
		if(FLAG_SIMPLE_UPDATE_)		strFileName = strFileNameWithoutExtension + "_simple_ml.bt";
		else								strFileName = strFileNameWithoutExtension + "_ml.bt";
		m_pOctree->toMaxLikelihood();
		m_pOctree->prune();
		m_pOctree->writeBinary(strFileName);

		std::cout << std::endl;
		return true;
	}

	/** @brief	Evaluate the octomap */
	template <typename PointT1, typename PointT2>
	bool evaluate(const std::vector<typename pcl::PointCloud<PointT1>::Ptr>			&pPointCloudPtrList,
					  const std::vector<PointT2, Eigen::aligned_allocator<PointT2> >	&sensorPositionList,
					  unsigned int																&num_points,
					  unsigned int																&num_voxels_correct,
					  unsigned int																&num_voxels_wrong,
					  unsigned int																&num_voxels_unknown,
					  const double																maxrange = -1)
	{
		// check size
		assert(pPointCloudPtrList.size() == sensorPositionList.size());

		// check memory
		if(!m_pOctree) return false;

		// initialization
		num_points = 0;
		num_voxels_correct = 0;
		num_voxels_wrong = 0;
		num_voxels_unknown = 0;

		// for each observation
		for(size_t i = 0; i < pPointCloudPtrList.size(); i++)
		{
			// robot position
			octomap::point3d robotPosition(sensorPositionList[i].x, sensorPositionList[i].y, sensorPositionList[i].z);

			// point cloud
			num_points += pPointCloudPtrList[i]->size();
			octomap::Pointcloud pc;
			for(size_t j = 0; j < pPointCloudPtrList[i]->size(); j++)
				pc.push_back(pPointCloudPtrList[i]->points[j].x, pPointCloudPtrList[i]->points[i].y, pPointCloudPtrList[i]->points[j].z);

			// free/occupied cells
			octomap::KeySet free_cells, occupied_cells;
			m_pOctree->computeUpdate(pc, robotPosition, free_cells, occupied_cells, maxrange);
			
			// count free cells
			for(octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
			{
				octomap::OcTreeNode* n = m_pOctree->search(*it);
				if(n)
				{
					if(m_pOctree->isNodeOccupied(n))	num_voxels_wrong++;
					else										num_voxels_correct++;
				}
				else											num_voxels_unknown++;
			}
			
			// count occupied cells
			for(octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
			{
				octomap::OcTreeNode* n = m_pOctree->search(*it);
				if(n)
				{
					if(m_pOctree->isNodeOccupied(n))	num_voxels_correct++;
					else										num_voxels_wrong++;
				}
				else											num_voxels_unknown++;
			}
		}

		return true;
	}

	/** @brief		Train hyperparameters of PLSC
	  * @return		Minimum sum of log inference probabilities of occupied centers 
	 */
	GP::DlibScalar train(const pcl::PointCloud<pcl::PointNormal>::ConstPtr	&pPointCloud,
								float &PLSC_mean, float &PLSC_var, 
								const bool fConsiderBothOccupiedAndEmpty,
								const int maxIter, 
								const GP::DlibScalar minValue = 1e-15)
	{
		// set the reference point cloud
		m_pPointCloud = pPointCloud;
		m_fConsiderBothOccupiedAndEmpty = fConsiderBothOccupiedAndEmpty;

		// conversion from PLSC hyperparameters to a Dlib vector
		GP::DlibVector logDlib;
		logDlib.set_size(2);
		logDlib(0, 0) = logf(PLSC_mean);
		logDlib(1, 0) = logf(PLSC_var);

		// trainer
		GP::DlibScalar sumNegLOO = GP::TrainerUsingApproxDerivatives<Octomap>::train<GP::BOBYQA, GP::NoStopping>(logDlib,
																																			*this,
																																			maxIter, minValue);
		// conversion from a Dlib vector to PLSC hyperparameters
		PLSC_mean	= expf(logDlib(0, 0));
		PLSC_var		= expf(logDlib(1, 0));

		// set the static variables
		PLSC::mean	= PLSC_mean;
		PLSC::var	= PLSC_var;

		return sumNegLOO;
	}

	/** @brief		Operator for optimizing hyperparameters 
	  * @return		Sum of log inference probabilities of occupied centers 
	  *				instead of Sum of log LOO (Leave-One-Out) probabilites
	  */
	GP::DlibScalar operator()(const GP::DlibVector &logDlib) const
	{
		// Sum of negative log marginalizations of all leaf nodes
		GP::DlibScalar sum_neg_log_occupied(0);
		GP::DlibScalar sum_neg_log_empty(0);

		// convert a Dlib vector to PLSC hyperparameters
		PLSC::mean	= expf(logDlib(0, 0));
		PLSC::var	= expf(logDlib(1, 0));

		// for each point
		for(size_t i = 0; i < m_pPointCloud->points.size(); i++)
		{
			// point
			const pcl::PointNormal &point = m_pPointCloud->points[i];

			// PLSC
			const float occupied_probabiliy = PLSC::occupancy(point.normal_x, point.normal_y);

			// if occupied
			if(occupied_probabiliy > 0.5f)	sum_neg_log_occupied -= logf(occupied_probabiliy);

			// else empty
			else										sum_neg_log_empty -= logf(1.f - occupied_probabiliy);
		}

		// sum of negative log probability
		GP::DlibScalar sum_neg_log_probability;
		if(m_fConsiderBothOccupiedAndEmpty)		sum_neg_log_probability = sum_neg_log_occupied + sum_neg_log_empty;
		else												sum_neg_log_probability = sum_neg_log_occupied;

		// log file
		LogFile logFile;
		logFile << "(" << PLSC::mean << ", "  << PLSC::var  << ") : " << sum_neg_log_probability << std::endl;

		return sum_neg_log_probability;
	}

protected:
	void getMinMaxMeanVarOfOccupiedCells(const pcl::PointCloud<pcl::PointNormal>	&pointCloud,
													 float &minMean,		float &maxMean,
													 float &minVar,		float &maxVar) const
	{
		// min, max
		minMean	= std::numeric_limits<float>::max();
		maxMean	= std::numeric_limits<float>::min();
		minVar	= std::numeric_limits<float>::max();
		maxVar	= std::numeric_limits<float>::min();		

		// for each point
		for(size_t i = 0; i < pointCloud.points.size(); i++)
		{
			// point
			const pcl::PointNormal &point = pointCloud.points[i];

			// PLSC
			const float occupied_probabiliy = PLSC::occupancy(point.normal_x, point.normal_y);

			// if occupied
			if(occupied_probabiliy > 0.5f)
			{
				// min, max
				minMean	= std::min<float>(minMean,		point.normal_x);
				maxMean	= std::max<float>(maxMean,		point.normal_x);
				minVar	= std::min<float>(minVar,		point.normal_y);
				maxVar	= std::max<float>(maxVar,		point.normal_y);
			}
		}

		// log
		LogFile logFile;
		logFile << "Min Mean: " << std::setprecision(std::numeric_limits<float>::digits10) << std::scientific << minMean << std::endl;
		logFile << "Max Mean: " << std::setprecision(std::numeric_limits<float>::digits10) << std::scientific << maxMean << std::endl;
		logFile << "Min Var: "  << std::setprecision(std::numeric_limits<float>::digits10) << std::scientific << minVar  << std::endl;
		logFile << "Max Var: "  << std::setprecision(std::numeric_limits<float>::digits10) << std::scientific << maxVar  << std::endl;
	}

protected:
	/** @brief	Octomap */
	boost::shared_ptr<MyOctomapT>			m_pOctree;

	/** @brief	Flag for simple update */
	const bool FLAG_SIMPLE_UPDATE_;

	/** @brief	GPMap as a point cloud for training PLSC hyperparameters */
	pcl::PointCloud<pcl::PointNormal>::ConstPtr	m_pPointCloud;

	/** @brief	Whether to consider both occupied and empty cells or occupied cells only
	  *			when training PLSC hyperparameters */
	bool	m_fConsiderBothOccupiedAndEmpty;
};

/** @brief	Convert a GPMap to an Octree based on PLSC */
template <>
void Octomap<NO_COLOR>::GPMap2Octomap(const pcl::PointCloud<pcl::PointNormal>	&pointCloud)
{
	// for each point
	for(size_t i = 0; i < pointCloud.points.size(); i++)
	{
		// point
		const pcl::PointNormal &point = pointCloud.points[i];

		// PLSC
		const float occupied_probabiliy = PLSC::occupancy(point.normal_x, point.normal_y);

		// if occupied
		if(occupied_probabiliy > 0.5f)
		{
			// set occupied
			m_pOctree->updateNode(static_cast<double>(point.x), 
										 static_cast<double>(point.y), 
										 static_cast<double>(point.z),
										 true);
		}
	}
}

/** @brief	Convert a GPMap to a ColorOctree based on PLSC */
template <>
void Octomap<COLOR>::GPMap2Octomap(const pcl::PointCloud<pcl::PointNormal>	&pointCloud)
{
	// get min, max
	float minMean, maxMean, minVar, maxVar;
	getMinMaxMeanVarOfOccupiedCells(pointCloud, minMean, maxMean, minVar, maxVar);

	// color
	Color color(minVar, maxVar);
	unsigned char r, g, b;

	// for each point
	for(size_t i = 0; i < pointCloud.points.size(); i++)
	{
		// point
		const pcl::PointNormal &point = pointCloud.points[i];

		// PLSC
		const float occupied_probabiliy = PLSC::occupancy(point.normal_x, point.normal_y);

		// if occupied
		if(occupied_probabiliy > 0.5f)
		{
			// set occupied
			m_pOctree->updateNode(static_cast<double>(point.x), 
										 static_cast<double>(point.y), 
										 static_cast<double>(point.z),
										 true);

			// color based on the variance
			color.rgb(point.normal_y, r, g, b);

			// set color
			m_pOctree->setNodeColor(point.x, point.y, point.z, r, g, b);
		}
	}
}

/** @brief	Convert a GPMap to a ColorOctree based on PLSC */
template <>
void Octomap<COLOR>::GPMap2Octomap(const pcl::PointCloud<pcl::PointNormal>		&pointCloud,
											  const float											minVarRangeForColor,
											  const float											maxVarRangeForColor)
{
	// color
	Color color(minVarRangeForColor, maxVarRangeForColor);
	unsigned char r, g, b;

	// for each point
	for(size_t i = 0; i < pointCloud.points.size(); i++)
	{
		// point
		const pcl::PointNormal &point = pointCloud.points[i];

		// PLSC
		const float occupied_probabiliy = PLSC::occupancy(point.normal_x, point.normal_y);

		// if occupied
		if(occupied_probabiliy > 0.5f)
		{
			// set occupied
			m_pOctree->updateNode(static_cast<double>(point.x), 
										 static_cast<double>(point.y), 
										 static_cast<double>(point.z),
										 true);

			// color based on the variance
			color.rgb(point.normal_y, r, g, b);

			// set color
			m_pOctree->setNodeColor(point.x, point.y, point.z, r, g, b);
		}
	}
}

}

#endif