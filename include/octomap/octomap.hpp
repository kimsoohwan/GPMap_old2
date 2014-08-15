#ifndef _GPMAP_TO_OCTOMAP_HPP_
#define _GPMAP_TO_OCTOMAP_HPP_

// STL
#include <string>
#include <vector>

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

	/** @brief Constructor */
	Octomap(const double				resolution,
			  const std::string		strFileName,
			  const bool				FLAG_SIMPLE_UPDATE = false)
		:	pOcTree(new octomap::OcTree(resolution)),
			FLAG_SIMPLE_UPDATE_(FLAG_SIMPLE_UPDATE)
	{
		// load octomap
		pOcTree->readBinary(strFileName);
	}

	/** @brief	Update a node of the octomap
	  * @return	Elapsed time (user/system/wall cpu times)
	  */
	void updateNode(const float x, const float y, const float z, const float log_odds_update)
	{
		pOcTree->updateNode(octomap::point3d(pointNormals->points[i].x, 
											pointNormals->points[i].y,
											pointNormals->points[i].z),
											pointNormals->points[i].normal_z);

		HERE!!!!

	}

	/** @brief	Update the octomap with a point cloud
	  * @return	Elapsed time (user/system/wall cpu times)
	  */
	template <typename PointT1, typename PointT2>
	boost::timer::cpu_times
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
		if(!pOcTree) return false;

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
			pOcTree->computeUpdate(pc, robotPosition, free_cells, occupied_cells, maxrange);
			
			// count free cells
			for(octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it)
			{
				octomap::OcTreeNode* n = pOcTree->search(*it);
				if(n)
				{
					if(pOcTree->isNodeOccupied(n))	num_voxels_wrong++;
					else										num_voxels_correct++;
				}
				else											num_voxels_unknown++;
			}
			
			// count occupied cells
			for(octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it)
			{
				octomap::OcTreeNode* n = pOcTree->search(*it);
				if(n)
				{
					if(pOcTree->isNodeOccupied(n))	num_voxels_correct++;
					else										num_voxels_wrong++;
				}
				else											num_voxels_unknown++;
			}
		}

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