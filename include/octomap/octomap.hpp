#ifndef _GPMAP_TO_OCTOMAP_HPP_
#define _GPMAP_TO_OCTOMAP_HPP_

// STL
#include <string>

// PCL
#include <pcl/point_types.h>		// pcl::PointXYZ, pcl::Normal, pcl::PointNormal
#include <pcl/point_cloud.h>		// pcl::PointCloud

// Octomap
#include <octomap/octomap.h>
#include <octomap/octomap_timing.h>
#include <octomap/ColorOcTree.h>

// GPMap
#include "util/data_types.hpp"	// PointXYZCloud
#include "util/timer.hpp"			// boost::timer

namespace GPMap {

//void LeafNode2PointNormal
//
//void GPMap2Octomap
//{
//		octomap::OcTree tree(mapResolution);
//
//}

class Octomap
{
public:
	/** @brief Constructor */
	Octomap(const double resolution,
			  const bool	FLAG_SIMPLE_UPDATE = false)
		:	pOcTree(new octomap::OcTree(resolution)),
			FLAG_SIMPLE_UPDATE_(FLAG_SIMPLE_UPDATE)
	{
	}

	/** @brief	Update the octomap 
	  * @return	Elapsed time (user/system/wall cpu times)
	  */
	template <typename PointT1, typename PointT2>
	boost::timer::cpu_times
	update(const typename pcl::PointCloud<PointT1>	&cloud,
			 const PointT2										&sensorPosition,
			 const double										maxrange = -1,
			 const bool											fSimpleUpdate = false)
	{
		// robot position
		octomap::point3d robotPosition(sensorPosition.x, sensorPosition.y, sensorPosition.z);

		// point cloud
		octomap::Pointcloud pc;
		for(size_t i = 0; i < cloud.size(); i++)
			pc.push_back(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);

		// timer - start
		boost::timer::cpu_timer timer;

		// update
		if (FLAG_SIMPLE_UPDATE_)	pOcTree->insertPointCloudRays(pc, robotPosition, maxrange);
		else								pOcTree->insertPointCloud(pc, robotPosition, maxrange);

		// timer - end
		boost::timer::cpu_times elapsed = timer.elapsed();

		// memory
		std::cout << "memory usage: "		<< pOcTree->memoryUsage()		<< std::endl;
		std::cout << "leaf node count: " << pOcTree->getNumLeafNodes() << std::endl;
		//if(tree->memoryUsage() > 900000000)
		if(pOcTree->memoryUsage() > 500000000)
		{
			pOcTree->toMaxLikelihood();
			pOcTree->prune();
			std::cout << "after pruned - memory usage: "		<< pOcTree->memoryUsage()		<< std::endl;
			std::cout << "after pruned - leaf node count: " << pOcTree->getNumLeafNodes() << std::endl;
		}

		// return the elapsed time
		return elapsed;
	}

	/** @brief	Save the octomap as a binary file */
	bool save(const std::string &strFileNameWithoutExtension)
	{
		// check
		if(!pOcTree) return false;

		// file name
		std::string strFileName;
		if(FLAG_SIMPLE_UPDATE_)		strFileName = strFileNameWithoutExtension + "_simple_ml.bt";
		else								strFileName = strFileNameWithoutExtension + "_ml.bt";

		// perform maximum likelihood and prune
		pOcTree->toMaxLikelihood();
		pOcTree->prune();

		// save
		pOcTree->writeBinary(strFileName);
		std::cout << std::endl;

		return true;
	}

protected:
	/** @brief	Octomap */
	boost::shared_ptr<octomap::OcTree> pOcTree;

	/** @brief	Flag for simple update */
	const bool FLAG_SIMPLE_UPDATE_;
};

//template <typename PointT>
//void PCD2Octomap(const double								resolution,
//					  typename pcl::PointCloud<PointT>	&cloud,
//					  PointXYZCloud							&sensorPositions)
//{
//	// octomap
//	octomap::OcTree* tree = new octomap::OcTree(resolution);
//
//}

}

#endif