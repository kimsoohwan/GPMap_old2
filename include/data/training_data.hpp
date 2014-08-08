#ifndef _GPMAP_TRAINING_DATA_HPP_
#define _GPMAP_TRAINING_DATA_HPP_

// PCL
#include <pcl/point_types.h>		// pcl::PointXYZ, pcl::Normal, pcl::PointNormal
#include <pcl/point_cloud.h>		// pcl::PointCloud

// GPMap
#include "util/data_types.hpp" // PointXYZVector, Matrix, MatrixPtr, Vector, VectorPtr

namespace GPMap {

template <typename PointT>
inline void genEmptyPoint(const float				gap,
								  const pcl::PointXYZ	&sensorPosition, 
								  const PointT				&hitPoint, 
								  pcl::PointXYZ			&emptyPoint)
{
	// vector from a hit point to the origin (sensorPosition)
	pcl::PointXYZ P_O(sensorPosition.x - hitPoint.x,
							sensorPosition.y - hitPoint.y,
							sensorPosition.z - hitPoint.z);
	const float norm = sqrt(P_O.x*P_O.x + P_O.y*P_O.y + P_O.z*P_O.z);
	P_O.x *= gap/norm;
	P_O.y *= gap/norm;
	P_O.z *= gap/norm;

	// empty point
	emptyPoint.x = hitPoint.x + P_O.x;
	emptyPoint.y = hitPoint.y + P_O.y;
	emptyPoint.z = hitPoint.z + P_O.z;
}

/** @brief Generate empty points just before hit points */
template <typename PointT>
pcl::PointCloud<pcl::PointXYZ>::Ptr
genEmptyPointCloud(const float														gap,
						 const pcl::PointXYZ												&sensorPosition,
						 const typename pcl::PointCloud<PointT>::ConstPtr		&pHitPointCloud,
						 const IndicesConstPtr											&pIndices = IndicesConstPtr())
{
	// memory allocation
	pcl::PointCloud<pcl::PointXYZ>::Ptr pEmptyPointCloud(new pcl::PointCloud<pcl::PointXYZ>());

	// empty point
	pcl::PointXYZ emptyPoint;

	// if the indices are specified,
	if(pIndices)
	{
		for(Indices::const_iterator iter = pIndices->begin(); iter != pIndices->end(); ++iter)
		{
			// assert index range
			assert( (*iter >= 0) && (*iter < static_cast<int>(pHitPointCloud->points.size())));

			// calculate empty point
			genEmptyPoint(gap, sensorPosition, pHitPointCloud->points[*iter], emptyPoint);

			// empty point
			pEmptyPointCloud->push_back(emptyPoint);
		}
	}
	else
	{
		// add empty points just before the hit pints
		for(size_t i = 0; i < pHitPointCloud->size(); i++)
		{
			// calculate empty point
			genEmptyPoint(gap, sensorPosition, pHitPointCloud->points[i], emptyPoint);

			// empty point
			pEmptyPointCloud->push_back(emptyPoint);
		}
	}

	return pEmptyPointCloud;
}

/** @brief Generate empty points just before hit points */
template <typename PointT>
void genEmptyPointCloudList(const float																	gap,
									 const PointXYZVector														&sensorPositionList,
									 const std::vector<typename pcl::PointCloud<PointT>::Ptr>		&hitPointCloudList,
									 PointXYZCloudPtrList														&emptyPointCloudList)
{
	// size check
	assert(hitPointCloudList.size() == sensorPositionList.size());
	emptyPointCloudList.clear();

	// for each hit point cloud, create an empty point list
	for(size_t i = 0; i < hitPointCloudList.size(); i++)
		emptyPointCloudList.push_back(genEmptyPointCloud<PointT>(gap, sensorPositionList[i], hitPointCloudList[i]));
}

/** @brief Generate empty points just before hit points */
void combinePointCloudList(const PointXYZCloudPtrList		&pointCloudList1,
									const PointXYZCloudPtrList		&pointCloudList2,
									PointXYZCloudPtrList				&pointCloudList)
{
	// size check
	assert(pointCloudList1.size() == pointCloudList2.size());
	pointCloudList.clear();

	// for each hit/empty point cloud, add them to the output list
	for(size_t i = 0; i < pointCloudList1.size(); i++)
	{
		pointCloudList.push_back(pointCloudList1[i]);
		pointCloudList.push_back(pointCloudList2[i]);
	}
}

/** @brief Generate training data from a point cloud
  * @Todo	Optimization or using getMatrixXfMap()
  */
void generateTraingData(const PointNormalCloudConstPtr		&pPointNormalCloud,
								const Indices								&indices,
								const pcl::PointXYZ						&sensorPosition,
								const float									gap,
								MatrixPtr &pX, MatrixPtr &pXd, VectorPtr &pYYd)
{
	// assume all surface normals are finite

	// K: NN by NN, NN = N + Nd*D
	// 
	// for example, when D = 3
	//                  | f(x) | df(xd)/dx_1, df(xd)/dx_2, df(xd)/dx_3
	//                  |  N   |     Nd            Nd           Nd
	// ---------------------------------------------------------------
	// f(x)        : N  |  FF  |     FD1,         FD2,         FD3
	// df(xd)/dx_1 : Nd |   -  |    D1D1,        D1D2,        D1D3  
	// df(xd)/dx_2 : Nd |   -  |      - ,        D2D2,        D2D3  
	// df(xd)/dx_3 : Nd |   -  |      - ,          - ,        D3D3

	// some constants
	const size_t D = 3;								// number of dimensions
	const size_t N = indices.size();				// hit points, empty points
	const size_t Nd = N;								// surface normals
	const bool fCreateEmptyPoint(gap > 0.f);	// create empty points or not

	// memory allocation
	if(fCreateEmptyPoint)
	{
		pX.reset(new Matrix(2*N, 3));				// hit/empty points
		pYYd.reset(new Vector(2*N + Nd*D));		// hit/empty points, surface normals
	}
	else
	{
		pX.reset(new Matrix(N, 3));				// hit points
		pYYd.reset(new Vector(N + Nd*D));		// hit points, surface normals
	}
	pXd.reset(new Matrix(N, 3));					// surface normals

	// assignment
	pcl::PointXYZ emptyPoint;
	int i = 0;
	for(Indices::const_iterator iter = indices.begin(); iter != indices.end(); ++iter)
	{
		// index
		assert(*iter >= 0 && *iter < static_cast<int>(N));

		// point normal
		const pcl::PointNormal pointNormal(pPointNormalCloud->points[*iter]);

		// check finite
		assert(pcl_isfinite(pointNormal.x) &&
				 pcl_isfinite(pointNormal.y) &&
				 pcl_isfinite(pointNormal.z) &&
				 pcl_isfinite(pointNormal.normal_x) &&
				 pcl_isfinite(pointNormal.normal_y) &&
				 pcl_isfinite(pointNormal.normal_z) &&
				 pointNormal.curvature > 0);

		// hit point
		(*pX)(i, 0)		= pointNormal.x;
		(*pX)(i, 1)		= pointNormal.y;
		(*pX)(i, 2)		= pointNormal.z;
		(*pYYd)(i)		= 0.f;

		if(fCreateEmptyPoint)
		{
			// empty point
			genEmptyPoint(gap, sensorPosition, pointNormal, emptyPoint);
			(*pX)(N+i, 0)	= emptyPoint.x;
			(*pX)(N+i, 1)	= emptyPoint.y;
			(*pX)(N+i, 2)	= emptyPoint.z;
			(*pYYd)(N+i)	= gap;
		}

		// surface normal
		(*pXd)(i, 0)			= pointNormal.x;
		(*pXd)(i, 1)			= pointNormal.y;
		(*pXd)(i, 2)			= pointNormal.z;
		if(fCreateEmptyPoint)
		{
			(*pYYd)(2*N+i)			= pointNormal.normal_x;
			(*pYYd)(2*N+Nd+i)		= pointNormal.normal_y;
			(*pYYd)(2*N+2*Nd+i)	= pointNormal.normal_z;
		}
		else
		{
			(*pYYd)(N+i)			= pointNormal.normal_x;
			(*pYYd)(N+Nd+i)		= pointNormal.normal_y;
			(*pYYd)(N+2*Nd+i)		= pointNormal.normal_z;
		}

		// next index
		i++;
	}
}

}

#endif