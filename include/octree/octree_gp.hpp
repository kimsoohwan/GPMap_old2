#ifndef _OCTREE_GAUSSIAN_PROCESS_HPP_
#define _OCTREE_GAUSSIAN_PROCESS_HPP_

// STL
#include <cmath>		// floor, ceil

// PCL
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

// Eigen
#include <Eigen/Dense>

// OpenGP
#include "GP.h"

// GPMap
#include "util/utility.hpp"		// PointXYZVector
#include "data/test_data.hpp"		// meshGrid

namespace GPMap {

class LeafNode : public pcl::octree::OctreeContainerDataTVector<int>
{
public:
	void setData(const int &data_arg)
	{
		// if the index is negative, just create the node
		if(data_arg < 0) return;

		// add to the int vector
		pcl::octree::OctreeContainerDataTVector<int>::setData(data_arg);
	}

	void predict()
	{
		std::cout << leafDataTVector_.size() << ": ";
	}
};

template<typename PointT, 
			//typename LeafT = pcl::octree::OctreeContainerDataTVector<int>,
			typename LeafT = LeafNode,
			typename BranchT = pcl::octree::OctreeContainerEmpty<int>,
			typename OctreeT = pcl::octree::OctreeBase<int, LeafT, BranchT> >
class OctreePointCloud_GP : public pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>
{
protected:
	TYPE_DEFINE_MATRIX(float);

public:
   /** @brief Constructor
	 *  @param resolution: octree resolution at lowest octree level
    */
   OctreePointCloud_GP(const double blockSize, const size_t nCellsPerBlock, const pcl::PointXYZ &min_pt, const pcl::PointXYZ &max_pt)
		: pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>(blockSize),
		  fBoundingBoxInitialized(false),
		  m_nCellsPerBlock(nCellsPerBlock),
		  m_pXs(new Matrix(nCellsPerBlock*nCellsPerBlock*nCellsPerBlock, 3))
   {
		assert(nCellsPerBlock > 0);

		// bounding box
		double minX, minY, minZ, maxX, maxY, maxZ;
		minX = floor(static_cast<double>(min_pt.x)/blockSize)*blockSize;
		minY = floor(static_cast<double>(min_pt.y)/blockSize)*blockSize;
		minZ = floor(static_cast<double>(min_pt.z)/blockSize)*blockSize;
		//maxX = ceil(static_cast<double>(max_pt.x)/blockSize)*blockSize;
		//maxY = ceil(static_cast<double>(max_pt.y)/blockSize)*blockSize;
		//maxZ = ceil(static_cast<double>(max_pt.z)/blockSize)*blockSize;
		float minValue = std::numeric_limits<float>::epsilon () * 512.0f;
		maxX = max_pt.x + minValue;
		maxY = max_pt.y + minValue;
		maxZ = max_pt.z + minValue;
		//defineBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
		//defineBoundingBox(0, 0, 0, blockSize+minValue, blockSize+minValue, blockSize+minValue);
		defineBoundingBox(min_pt.x, min_pt.y, min_pt.z, max_pt.x, max_pt.y, max_pt.z);

		std::cout << min_pt.x << ", " << min_pt.y << ", " << min_pt.z << std::endl;
		std::cout << minX << ", " << minY << ", " << minZ << std::endl;
		std::cout << max_pt.x << ", " << max_pt.y << ", " << max_pt.z << std::endl;
		std::cout << maxX << ", " << maxY << ", " << maxZ << std::endl;

		// set the test positions at (0, 0, 0)
		meshGrid(Eigen::Vector3f(0.f, 0.f, 0.f), 
					nCellsPerBlock, 
					static_cast<float>(blockSize) / static_cast<float>(nCellsPerBlock),
					m_pXs);
   }

	/** @brief Empty class constructor */
	virtual ~OctreePointCloud_GP()
	{
	}


	/** @brief		Update the GPMap with new observations
	  * @details	Refer to pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>
	  *				::setInputCloud(const PointCloudConstPtr &cloud_arg, const IndicesConstPtr &indices_arg = IndicesConstPtr ())
	  *				where assertion is activated when this->leafCount_!=0.
	  */
	void update(const PointCloudConstPtr	&pCloud,
					const IndicesConstPtr		&pIndices = IndicesConstPtr())
	{
		// reset
		reset();

		// set the input cloud
		input_	= pCloud;
		indices_	= pIndices;

		// define the bounding box
		if(!fBoundingBoxInitialized)
		{
			//defineBoundingBox();
			fBoundingBoxInitialized = true;
		}
		else
		{
			// min max of the new observations
			PointT min_pt, max_pt;
			pcl::getMinMax3D(*pCloud, min_pt, max_pt);

			// adopt the bounding box
			adoptBoundingBoxToPoint(min_pt);
			adoptBoundingBoxToPoint(max_pt);
		}

		// add point
		addPointsFromInputCloudWithoutAssertion();
	}

	/** @brief Predict the GPMap */
	void predict()
	{
		// create empty neigboring blocks if necessary
		createEmptyNeigboringBlocks();

		// leaf node iterator
		LeafNodeIterator iter(*this);

		// for each leaf node
		while(*++iter)
		{
			// predict
			LeafNode *pLeafNode = static_cast<LeafNode*>(iter.getCurrentOctreeNode());
			pLeafNode->predict();

			// key
			const pcl::octree::OctreeKey &key = iter.getCurrentOctreeKey();

			// get indices
			std::vector<int> indexVector;
			for(int deltaX = -1; deltaX <= 1; deltaX++)
				for(int deltaY = -1; deltaY <= 1; deltaY++)
					for(int deltaZ = -1; deltaZ <= 1; deltaZ++)
					{
						getData(pcl::octree::OctreeKey(key.x+deltaX, key.y+deltaY, key.z+deltaZ), indexVector);
					}

			std::cout << indexVector.size() << ", ";
		}

		//genLeafNodeCenterFromOctreeKey (const OctreeKey & key, PointT & point) const
	}

	/** @brief Get occupied voxel centers */
	/** @brief		Get occupied voxel center points
	  * @details	Refer to pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::getOccupiedVoxelCenters(AlignedPointTVector &voxelCenterList_arg) const
	  *				which only accept a vector of PointT points, not pcl::PointXYZ.
	  *				Thus, even if the octree has pcl::PointNormals, the center point should also be pcl::PointXYZ.
	  */
	size_t getOccupiedVoxelCenters(PointXYZVector	&voxelCenterPointXYZVector,
											const bool			fRemoveIsolatedVoxels) const
	{
		// clear the vector
		voxelCenterPointXYZVector.clear();

		// shift key
		pcl::octree::OctreeKey key(0, 0, 0);

		// search for occupied voxels recursively
		return getOccupiedVoxelCentersRecursive(this->rootNode_, key, voxelCenterPointXYZVector, fRemoveIsolatedVoxels);
	}

protected:
	/** @brief Reset the points in each voxel */
	void reset()
	{
		// leaf node iterator
		LeafNodeIterator iter(*this);

		// for each leaf node
		while(*++iter)
		{
			// predict
			LeafNode *pLeafNode = static_cast<LeafNode*>(iter.getCurrentOctreeNode());
			pLeafNode->reset();
		}
	}

	/** @brief		Add points from input cloud
	  * @details	Refer to pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::addPointsFromInputCloud()
	  *				which activates assertion this->leafCount_!=0.
	  *				Thus, even if the octree has pcl::PointNormals, the center point should also be pcl::PointXYZ.
	  */
	void addPointsFromInputCloudWithoutAssertion()
	{
		// assert (this->leafCount_==0);

		// delete the previous point cloud

		// add the new point cloud
		if(indices_)
		{
			for(std::vector<int>::const_iterator current = indices_->begin (); current != indices_->end (); ++current)
			{
				if(isFinite(input_->points[*current]))
				{
					assert( (*current>=0) && (*current < static_cast<int>(input_->points.size ())));
					
					// add points to octree
					this->addPointIdx(*current);
				}
			}
		}
		else
		{
			for(size_t i = 0; i < input_->points.size (); i++)
			{
				if (isFinite(input_->points[i]))
				{
					// add points to octree
					this->addPointIdx(static_cast<int>(i));
				}
			}
		}
	}

	/** @brief Create empty neighboring blocks for each occupied block if necessary */
	void createEmptyNeigboringBlocks()
	{
		// leaf node iterator
		LeafNodeIterator iter(*this);

		// for each leaf node
		while(*++iter)
		{
			// key
			const pcl::octree::OctreeKey &key = iter.getCurrentOctreeKey();

			// add -1 index to the neighboring leaf nodes
			for(int deltaX = -1; deltaX <= 1; deltaX++)
				for(int deltaY = -1; deltaY <= 1; deltaY++)
					for(int deltaZ = -1; deltaZ <= 1; deltaZ++)
					{
						if(deltaX == 0 && deltaY == 0 && deltaZ == 0) continue;
						addData(pcl::octree::OctreeKey(key.x+deltaX, key.y+deltaY, key.z+deltaZ), -1);
					}
		}
	}


	/** @brief Get the maximum octree key */
	void getMaxKey(pcl::octree::OctreeKey &key) const
	{
		// calculate unsigned integer octree key
		//key.x = static_cast<unsigned int>((this->maxX_ - this->minX_) / this->resolution_);
		//key.y = static_cast<unsigned int>((this->maxY_ - this->minY_) / this->resolution_);
		//key.z = static_cast<unsigned int>((this->maxZ_ - this->minZ_) / this->resolution_);
		key = maxKey_;
	}

	/** @brief Get the point indices in the leaf node corresponding the octree key */
	bool getData(const pcl::octree::OctreeKey &key, std::vector<int> &indexVector) const
	{
		// leaf node corresponding the octree key
		LeafT* pLeaf = findLeaf(key);

		// if the leaf node exists, add point indices to the vector
		if(pLeaf)
		{
			pLeaf->getData(indexVector);
			return true;
		}
		return false;
	}

	/** @brief Get the min max points of the voxel corresponding the octree key */
	void genVoxelBounds(const pcl::octree::OctreeKey &key, Eigen::Vector3f &min_pt, Eigen::Vector3f &max_pt) const 
	{
		// calculate voxel bounds
		min_pt(0) = static_cast<float>(static_cast<double>(key.x) * this->resolution_ + this->minX_);
		min_pt(1) = static_cast<float>(static_cast<double>(key.y) * this->resolution_ + this->minY_);
		min_pt(2) = static_cast<float>(static_cast<double>(key.z) * this->resolution_ + this->minZ_);

		max_pt(0) = static_cast<float>(static_cast<double>(key.x + 1) * this->resolution_ + this->minX_);
		max_pt(1) = static_cast<float>(static_cast<double>(key.y + 1) * this->resolution_ + this->minY_);
		max_pt(2) = static_cast<float>(static_cast<double>(key.z + 1) * this->resolution_ + this->minZ_);
	}

	/** @brief		Get the center point of the voxel corresponding the octree key
	  * @details	Refer to pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::genLeafNodeCenterFromOctreeKey(const OctreeKey & key, PointT & point) const
	  *				which only accept PointT point, not pcl::PointXYZ.
	  *				Thus, even if the octree has pcl::PointNormals, the center point should also be pcl::PointXYZ.
	  */
	void genLeafNodeCenterPointXYZ(const pcl::octree::OctreeKey &key, pcl::PointXYZ &point) const
	{
		// define point to leaf node voxel center
		point.x = static_cast<float>((static_cast<double>(key.x) + 0.5) * this->resolution_ + this->minX_);
		point.y = static_cast<float>((static_cast<double>(key.y) + 0.5) * this->resolution_ + this->minY_);
		point.z = static_cast<float>((static_cast<double>(key.z) + 0.5) * this->resolution_ + this->minZ_);
	}

	/** @brief		Get the occupied voxel center points
	  * @details	Refer to pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::getOccupiedVoxelCentersRecursive(const BranchNode* node_arg, const OctreeKey& key_arg, AlignedPointXYZVector &voxelCenterList_arg) const
	  *				which only accept a vector of PointT points, not pcl::PointXYZ.
	  *				Thus, even if the octree has pcl::PointNormals, the center point should also be pcl::PointXYZ.
	  */
	size_t getOccupiedVoxelCentersRecursive(const BranchNode	*node,
														 const pcl::octree::OctreeKey		&key,
														 PointXYZVector						&voxelCenterPointXYZVector,
														 const bool								fRemoveIsolatedVoxels) const
	{
		// voxel count
		size_t voxelCount = 0;
		
		// iterate over all children
		for(unsigned char childIdx = 0; childIdx < 8; childIdx++)
		{
			if (!this->branchHasChild(*node, childIdx)) continue;
			
			const pcl::octree::OctreeNode *childNode;
			childNode = this->getBranchChildPtr(*node, childIdx);
			
			// generate new key for current branch voxel
			pcl::octree::OctreeKey newKey;
			newKey.x = (key.x << 1) | (!!(childIdx & (1 << 2)));
			newKey.y = (key.y << 1) | (!!(childIdx & (1 << 1)));
			newKey.z = (key.z << 1) | (!!(childIdx & (1 << 0)));
			
			// for each node type
			switch(childNode->getNodeType())
			{
				// if this node is a branch node, go deeper recursively
				case pcl::octree::BRANCH_NODE:
				{
					// recursively proceed with indexed child branch
					voxelCount += getOccupiedVoxelCentersRecursive(static_cast<const BranchNode*>(childNode), newKey, voxelCenterPointXYZVector, fRemoveIsolatedVoxels);
					break;
				}
				
				// if this node is a leaf node, check if it is not isolated and add the center point
				case pcl::octree::LEAF_NODE:
				{
					// if it is an isolated voxel, do not add its center point
					if(fRemoveIsolatedVoxels && !isNotIsolatedVoxel(newKey)) break;

					// calculate the center point and add it to the vector
					pcl::PointXYZ newPoint;
					genLeafNodeCenterPointXYZ(newKey, newPoint);
					voxelCenterPointXYZVector.push_back(newPoint);
					voxelCount++;
					break;
				}
				
				default:
					break;
			}
		}
		
		return voxelCount;
	}

	bool isNotIsolatedVoxel(const pcl::octree::OctreeKey &key) const
	{
		// if it is on the boundary
		// TODO: maxKey_
		if(key.x == 0 || key.y == 0 || key.z == 0 ||
			key.x >= maxKey_.x || key.y >= maxKey_.y || key.z >= maxKey_.z) return true;

		// check if the node is surrounded with occupied nodes
		if(!existLeaf(key.x+1, key.y,   key.z  ))		return true;
		if(!existLeaf(key.x-1, key.y,   key.z  ))		return true;
		if(!existLeaf(key.x,   key.y+1, key.z  ))		return true;
		if(!existLeaf(key.x,   key.y-1, key.z  ))		return true;
		if(!existLeaf(key.x,   key.y,   key.z+1))		return true;
		if(!existLeaf(key.x,   key.y,   key.z-1))		return true;

		return false;
	}

protected:
	///** @brief		Check if this is first observation or not */
	bool				fBoundingBoxInitialized;

	/** @brief		Number of cells per a block
	  * @details	Note that each block(voxel) has a number of cells.
	  *				The block size corresponds to the resolution of voxels in pcl::octree::OctreePointCloud
	  */
	const double m_nCellsPerBlock;

	/** @brief		Test inputs of a block whose minimum point is (0, 0, 0) */
	MatrixPtr	m_pXs;
};
 

}

#endif