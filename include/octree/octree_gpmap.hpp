#ifndef _OCTREE_GPMAP_HPP_
#define _OCTREE_GPMAP_HPP_

// STL
#include <cmath>			// floor, ceil
#include <vector>

// PCL
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

// Eigen
#include <Eigen/Dense>

//// Boost
//#include "boost/tuple/tuple.hpp"

// OpenGP
#include "GP.h"

// GPMap
#include "util/util.hpp"						// min max
#include "octree/octree_container.hpp"		// LeafNode
#include "data/test_data.hpp"					// meshGrid
#include "plsc/plsc.hpp"						// PLSC

namespace GPMap {

template<typename PointT, 
			typename LeafT		= LeafNode,
			typename BranchT	= pcl::octree::OctreeContainerEmpty<int>,
			typename OctreeT	= pcl::octree::OctreeBase<int, LeafT, BranchT> >
class OctreeGPMap : public pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>
{
public:
	public:
	/** @brief Constructor
	*  @param resolution: octree resolution at lowest octree level
    */
   OctreeGPMap(const double				blockSize, 
					const size_t				nCellsPerAxis, 
					const bool					fVarianceVector,
					const bool					fDuplicatePointIndexToNeighboringVoxels)
		: pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>(blockSize),
		  m_blockSize(resolution_),
		  m_cellSize(blockSize/static_cast<double>(nCellsPerAxis)),
		  m_nCellsPerAxis(nCellsPerAxis),
		  m_nCellsPerBlock(nCellsPerAxis*nCellsPerAxis*nCellsPerAxis),
		  m_fVarianceVector(fVarianceVector),
		  m_fDuplicatePointIndexToNeighboringVoxels(fDuplicatePointIndexToNeighboringVoxels),
		  m_pXs(new Matrix(m_nCellsPerBlock, 3))
   {
		assert(nCellsPerAxis > 0);

		// set the test positions at (0, 0, 0)
		meshGrid(Eigen::Vector3f(0.f, 0.f, 0.f), 
					m_nCellsPerAxis, 
					static_cast<float>(blockSize) / static_cast<float>(m_nCellsPerAxis),
					m_pXs);
   }

	/** @brief Empty class constructor */
	virtual ~OctreeGPMap()
	{
	}

	/** @brief Define bounding box for octree
	* @note Bounding box cannot be changed once the octree contains elements.
	* @param[in] min_pt lower bounding box corner point
	* @param[in] max_pt upper bounding box corner point
	*/
	template <typename GeneralPointT>
	void defineBoundingBox(const GeneralPointT &min_pt, const GeneralPointT &max_pt)
	{
		defineBoundingBox(min_pt.x, min_pt.y, min_pt.z, 
								max_pt.x, max_pt.y, max_pt.z);
	}

	/** @brief Define bounding box for octree
	* @note Bounding box cannot be changed once the octree contains elements.
	* @param[in] minX X coordinate of lower bounding box corner
	* @param[in] minY Y coordinate of lower bounding box corner
	* @param[in] minZ Z coordinate of lower bounding box corner
	* @param[in] maxX X coordinate of upper bounding box corner
	* @param[in] maxY Y coordinate of upper bounding box corner
	* @param[in] maxZ Z coordinate of upper bounding box corner
	*/
	void defineBoundingBox(double minX, double minY, double minZ,
								  double maxX, double maxY, double maxZ)
	{
		minX = floor(static_cast<double>(minX)/m_blockSize - 1.f)*m_blockSize;
		minY = floor(static_cast<double>(minY)/m_blockSize - 1.f)*m_blockSize;
		minZ = floor(static_cast<double>(minZ)/m_blockSize - 1.f)*m_blockSize;
		maxX = ceil (static_cast<double>(maxX)/m_blockSize + 1.f)*m_blockSize;
		maxY = ceil (static_cast<double>(maxY)/m_blockSize + 1.f)*m_blockSize;
		maxZ = ceil (static_cast<double>(maxZ)/m_blockSize + 1.f)*m_blockSize;
		pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::defineBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
	}

   /** @brief Provide a pointer to the input data set.
	  * @details	Refer to pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>
	  *				::setInputCloud(const PointCloudConstPtr &cloud_arg, const IndicesConstPtr &indices_arg = IndicesConstPtr ())
	  *				where assertion is activated when this->leafCount_!=0.
     * @param[in] pCloud the const boost shared pointer to a PointCloud message
     * @param[in] pIndices the point indices subset that is to be used from \a cloud - if 0 the whole point cloud is used
    */
   void setInputCloud(const PointCloudConstPtr	&pCloud,
							 const float					gap = 0.f,
							 const pcl::PointXYZ			&sensorPosition = pcl::PointXYZ(),
							 const IndicesConstPtr		&pIndices = IndicesConstPtr())
   {
		//assert(this->leafCount_==0);

		// set the input cloud
		input_	= pCloud;
		indices_	= pIndices;

		// gap and sensor position for generating empty points
		m_gap = gap;
		m_sensorPosition = sensorPosition;
	}

	/** @brief Add points from input point cloud to octree.
	  * @details	Refer to pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::addPointsFromInputCloud()
	  *				where assertion is activated when this->leafCount_!=0.
	  */
	void addPointsFromInputCloud()
	{
		// assert (this->leafCount_==0);

		// reset the previous point indices in each voxel
		reset();

		// min max of the new observations
		PointT min_pt, max_pt;
		pcl::getMinMax3D(*input_, min_pt, max_pt);
		min_pt.x -= m_blockSize;
		min_pt.y -= m_blockSize;
		min_pt.z -= m_blockSize;
		max_pt.x += m_blockSize;
		max_pt.y += m_blockSize;
		max_pt.z += m_blockSize;

		// adopt the bounding box
		adoptBoundingBoxToPoint(min_pt);
		adoptBoundingBoxToPoint(max_pt);

		// add the new point cloud
		if(indices_)
		{
			for(std::vector<int>::const_iterator current = indices_->begin (); current != indices_->end (); ++current)
			{
				if(isFinite(input_->points[*current]))
				{
					assert( (*current>=0) && (*current < static_cast<int>(input_->points.size())));
					
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

		// total number of indices
		if(m_fDuplicatePointIndexToNeighboringVoxels)	assert(totalNumOfPointIndices() == static_cast<size_t>(27)*(input_->size()));
		else															assert(totalNumOfPointIndices() == input_->size());
	}

	/** @brief		Update the GPMap with new observations */
	void update()
	{
		// if a point index is duplicated to 
		if(m_fDuplicatePointIndexToNeighboringVoxels)
		{
			// leaf node iterator
			LeafNodeIterator iter(*this);

			// for each leaf node
			Eigen::Vector3f min_pt;
			Indices indexVector;
			while(*++iter)
			{
				// key
				const pcl::octree::OctreeKey &key = iter.getCurrentOctreeKey();

				// min point
				genVoxelMinPoint(key, min_pt);

				// collect indices
				indexVector.clear();
				getData(key, indexVector);
				assert(indexVector.size() > 0);

				// leaf node
				LeafNode *pLeafNode = static_cast<LeafNode *>(iter.getCurrentOctreeNode());

				// predict
				predict(indexVector, min_pt, pLeafNode);
			}
		}
		else
		{
			// create empty neigboring blocks if necessary
			createEmptyNeigboringBlocks();

			// leaf node iterator
			LeafNodeIterator iter(*this);

			// for each leaf node
			Eigen::Vector3f min_pt;
			std::vector<int> indexVector;
			while(*++iter)
			{
				// key
				const pcl::octree::OctreeKey &key = iter.getCurrentOctreeKey();

				// min point
				genVoxelMinPoint(key, min_pt);

				// collect indices
				indexVector.clear();
				for(int deltaX = -1; deltaX <= 1; deltaX++)
					for(int deltaY = -1; deltaY <= 1; deltaY++)
						for(int deltaZ = -1; deltaZ <= 1; deltaZ++)
						{
							getData(pcl::octree::OctreeKey(key.x+deltaX, key.y+deltaY, key.z+deltaZ), indexVector);
						}
				assert(indexVector.size() > 0);

				// leaf node
				LeafNode *pLeafNode = static_cast<LeafNode *>(iter.getCurrentOctreeNode());

				// predict
				predict(indexVector, min_pt, pLeafNode);
			}
		}
	}

	inline bool isCellNotOccupied(const VectorPtr &pMean, const MatrixPtr &pVariance, const size_t ix, const size_t iy, const size_t iz, const float threshold) const
	{
		const size_t idx(xyz2idx(m_nCellsPerAxis, ix, iy, iz));
		return PLSC(pMean(idx), pVariance(idx, 0)) < threshold;
	}

	inline bool isNotIsolatedCell(const VectorPtr &pMean, const MatrixPtr &pVariance, const size_t ix, const size_t iy, const size_t iz, const float threshold, const bool fRemoveIsolatedCells, size_t &idx) const
	{
		// current index
		idx = xyz2idx(m_nCellsPerAxis, ix, iy, iz);
		
		// check neighboring cells
		if(fRemoveIsolatedCells)
		{
			// last index
			const size_t lastIdx(m_nCellsPerAxis-1);

			if(ix == 0 || iy == 0 || iz == 0 ||
				ix >= lastIdx || iy >= lastIdx || iz >= lastIdx) return true;

			// check if the node is surrounded with occupied nodes
			if(isCellNotOccupied(pMean, pVariance, ix+1, iy,   iz  , threshold))		return true;
			if(isCellNotOccupied(pMean, pVariance, ix-1, iy,   iz  , threshold))		return true;
			if(isCellNotOccupied(pMean, pVariance, ix,   iy+1, iz  , threshold))		return true;
			if(isCellNotOccupied(pMean, pVariance, ix,   iy-1, iz  , threshold))		return true;
			if(isCellNotOccupied(pMean, pVariance, ix,   iy,   iz+1, threshold))		return true;
			if(isCellNotOccupied(pMean, pVariance, ix,   iy,   iz-1, threshold))		return true;
		}

		return isCellNotOccupied(pMean, pVariance, ix, iy, iz, threshold);
	}

	/** @brief Get occupied cell centers */
	size_t getOccupiedCellCenters(PointXYZVector		&cellCenterPointXYZVector,
											const float			threshold,
											const bool			fRemoveIsolatedCells) const
	{
		// clear the vector
		cellCenterPointXYZVector.clear();

		// leaf node iterator
		LeafNodeIterator iter(*this);

		// for each leaf node
		VectorPtr pMean;
		MatrixPtr pVariance;
		while(*++iter)
		{
			// key
			const pcl::octree::OctreeKey &key = iter.getCurrentOctreeKey();

			// min point
			genVoxelMinPoint(key, min_pt);

			// leaf node
			LeafNode *pLeafNode = static_cast<LeafNode *>(iter.getCurrentOctreeNode());

			// mean, variance
			pLeafNode->get(pMean, pVariance);
			if(pVariance->cols() != 1)
			{
				MatrixPtr pTempVariance(new Matrix(pVariance->rows(), 1));
				pTempVariance->noalias() = pVariance->diagonal();
				pVariance.reset(pTempVariance);
			}

			// check if each cell is occupied
			size_t i;
			for(size_t ix = 0; ix < m_nCellsPerAxis; ix++)
				for(size_t iy = 0; iy < m_nCellsPerAxis; iy++)
					for(size_t iz = 0; iz < m_nCellsPerAxis; iz++)
						if(isNotIsolatedCell(pMean, pVariance, ix, iy, iz, threshold, i, fRemoveIsolatedCells))
							cellCenterPointXYZVector.push_back(pcl::PointXYZ(m_pXs(i, 0) + min_pt.x, 
																							 m_pXs(i, 1) + min_pt.y,
																							 m_pXs(i, 2) + min_pt.z));
		}
	}

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

	/** @brief Get the total number of point indices stored in each voxel */
	size_t totalNumOfPointIndices()
	{
		// size
		size_t n(0);

		// leaf node iterator
		LeafNodeIterator iter(*this);

		// for each leaf node
		while(*++iter)
		{
			// get size
			LeafNode *pLeafNode = static_cast<LeafNode*>(iter.getCurrentOctreeNode());
			n += pLeafNode->getSize();
		}

		return n;
	}

	double getCellSize() const
	{
		return m_cellSize;
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
			// reset
			LeafNode *pLeafNode = static_cast<LeafNode*>(iter.getCurrentOctreeNode());
			pLeafNode->reset();
		}
	}

	/** @brief		Add a point from input cloud to the corresponding voxel and neighboring ones
	  * @details	Refer to pcl::octree::OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::addPointIdx (const int pointIdx_arg)
	  *				which add the point to the corresponding voxel only.
	  */
	void addPointIdx(const int pointIdx)
	{
		// check the index range
		assert(pointIdx < static_cast<int>(input_->points.size()));
	
		// point
		const PointT& point = input_->points[pointIdx];
		
		// make sure bounding box is big enough
		if(m_fDuplicatePointIndexToNeighboringVoxels)
		{
			PointT min_pt(point), max_pt(point);
			min_pt.x -= m_blockSize;
			min_pt.y -= m_blockSize;
			min_pt.z -= m_blockSize;
			max_pt.x += m_blockSize;
			max_pt.y += m_blockSize;
			max_pt.z += m_blockSize;
			adoptBoundingBoxToPoint(min_pt);
			adoptBoundingBoxToPoint(max_pt);
		}
		else
			adoptBoundingBoxToPoint(point);
		
		// key
		pcl::octree::OctreeKey key;
		genOctreeKeyforPoint(point, key);
		
		// add point to octree at key
		if(m_fDuplicatePointIndexToNeighboringVoxels)
		{
			for(int deltaX = -1; deltaX <= 1; deltaX++)
				for(int deltaY = -1; deltaY <= 1; deltaY++)
					for(int deltaZ = -1; deltaZ <= 1; deltaZ++)
					{
						assert(static_cast<int>(key.x) + deltaX >= 0);
						assert(static_cast<int>(key.y) + deltaY >= 0);
						assert(static_cast<int>(key.z) + deltaZ >= 0);
						this->addData(pcl::octree::OctreeKey(static_cast<unsigned int>(key.x+deltaX), 
																		 static_cast<unsigned int>(key.y+deltaY),
																		 static_cast<unsigned int>(key.z+deltaZ)),
																		 pointIdx);
					}
		}
		else
			this->addData(key, pointIdx);
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
#ifdef _DEBUG
						Eigen::Vector3f min_pt, max_pt;
						genVoxelBounds(static_cast<unsigned int>(key.x+deltaX), 
											static_cast<unsigned int>(key.y+deltaY),
											static_cast<unsigned int>(key.z+deltaZ), 
											min_pt, max_pt);
						assert(min_pt.x() >= minX_ && min_pt.y() >= minY_ && min_pt.z() >= minZ_);
						assert(max_pt.x() <= maxX_ && max_pt.y() <= maxY_ && max_pt.z() <= maxZ_);
#endif
						if(deltaX == 0 && deltaY == 0 && deltaZ == 0) continue;
						addData(pcl::octree::OctreeKey(static_cast<unsigned int>(key.x+deltaX), 
																 static_cast<unsigned int>(key.y+deltaY),
																 static_cast<unsigned int>(key.z+deltaZ)),
																 -1);
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
		LeafNode* pLeafNode = findLeaf(key);

		// if the leaf node exists, add point indices to the vector
		if(pLeafNode)
		{
			pLeafNode->getData(indexVector);
			return true;
		}
		return false;
	}

	/** @brief Get the min max points of the voxel corresponding the octree key */
	void genVoxelBounds(const pcl::octree::OctreeKey &key, Eigen::Vector3f &min_pt, Eigen::Vector3f &max_pt) const 
	{
		// calculate voxel bounds
		genVoxelMinPoint(key, min_pt);
		genVoxelMaxPoint(key, max_pt);
	}

	/** @brief Get the min points of the voxel corresponding the octree key */
	inline void genVoxelMinPoint(const pcl::octree::OctreeKey &key, Eigen::Vector3f &min_pt) const 
	{
		// calculate voxel bounds
		min_pt(0) = static_cast<float>(static_cast<double>(key.x) * this->resolution_ + this->minX_);
		min_pt(1) = static_cast<float>(static_cast<double>(key.y) * this->resolution_ + this->minY_);
		min_pt(2) = static_cast<float>(static_cast<double>(key.z) * this->resolution_ + this->minZ_);
	}

	/** @brief Get the max points of the voxel corresponding the octree key */
	inline void genVoxelMaxPoint(const pcl::octree::OctreeKey &key, Eigen::Vector3f &max_pt) const 
	{
		// calculate voxel bounds
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

	/** @details	The leaf node has only index vector, 
	  *				no information about the point cloud or min/max boundary of the voxel.
	  *				Thus, prediction is done in OctreeGPMap not in LeafNode.
	  *				But the result will be dangled to LeafNode for further BCM update.
	  */
	void predict(const Indices				&indexVector, 
					 Eigen::Vector3f			&min_pt,
					 LeafNode					*pLeafNode)
	{
		// training data
		DerivativeTrainingData derivativeTrainingData;
		MatrixPtr pX, pXd; VectorPtr pYYd;
		generateTraingData(input_, indexVector, m_sensorPosition, m_gap, pX, pXd, pYYd);
		derivativeTrainingData.set(pX, pXd, pYYd);

		// test data
		TestData testData;
		MatrixPtr pXs(new Matrix(m_nCellsPerBlock, 3));
		Matrix minValue(1, 3); 
		minValue << min_pt.x(), min_pt.y(), min_pt.z();
		pXs->noalias() = (*m_pXs) + minValue.replicate(m_nCellsPerBlock, 1);
		testData.set(pXs);

		// hyperparameters
		const float ell(0.5f);
		const float sigma_f(1.5f);
		const float sigma_n(0.1f);
		const float sigma_nd(0.2f);

		GPType::Hyp		logHyp;
		logHyp.cov(0) = log(ell);
		logHyp.cov(1) = log(sigma_f);
		logHyp.lik(0) = log(sigma_n);
		logHyp.lik(1) = log(sigma_nd);

#if EIGEN_VERSION_AT_LEAST(3,2,0)
		// train
		GPType::train<GP::BOBYQA, GP::NoStopping>(logHyp, derivativeTrainingData, 10000);

		// predict
		GPType::predict(logHyp, derivativeTrainingData, testData, m_fVarianceVector); // 5000
		//GPType::predict(logHyp, derivativeTrainingData, testData, m_fVarianceVector, pXs->rows());
#else
	#error
#endif

		// update
		pLeafNode->update(testData.pMu(), testData.pSigma());
	}

protected:
	/** @brief		Flag for duplicating a point index to neighboring voxels 
	  * @details	If it is duplicated, prediction will be easy without considering neighboring voxels,
	  *				but the total memory size for indices will be 27 times bigger. */
	const bool		m_fDuplicatePointIndexToNeighboringVoxels;
	const bool		m_fVarianceVector;

	/** @brief Size of each block (voxel) */
	double			&m_blockSize;
	const double	m_cellSize;
	
	/** @brief		Number of cells per a block
	  * @details	Note that each block(voxel) has a number of cells.
	  *				The block size corresponds to the resolution of voxels in pcl::octree::OctreePointCloud
	  */
	const size_t	m_nCellsPerAxis;
	const size_t	m_nCellsPerBlock;

	/** @brief For generating empty points */
	float				m_gap;
	pcl::PointXYZ	m_sensorPosition;

	/** @brief		Test inputs of a block whose minimum point is (0, 0, 0) */
	MatrixPtr	m_pXs;
};
 

}

#endif