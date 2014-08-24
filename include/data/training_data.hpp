#ifndef _GPMAP_TRAINING_DATA_HPP_
#define _GPMAP_TRAINING_DATA_HPP_

// PCL
#include <pcl/point_types.h>		// pcl::PointXYZ, pcl::Normal, pcl::PointNormal
#include <pcl/point_cloud.h>		// pcl::PointCloud

// GPMap
#include "util/data_types.hpp"			// PointXYZVList, Matrix, MatrixPtr, Vector, VectorPtr
#include "util/util.hpp"					// isFinite
#include "features/surface_normal.hpp"	// norm

namespace GPMap {

inline bool genEmptyPoint(const pcl::PointNormal	&pointNormal,
								  const float					gap,
								  pcl::PointXYZ				&emptyPoint)
{
	// assume the point normal is finite
	if(!pcl::isFinite<pcl::PointNormal>(pointNormal)) return false;

	// length
	const float length = norm(pointNormal);

	// factor
	const float factor = gap/length;

	// empty point
	emptyPoint.x = pointNormal.x + factor*pointNormal.normal_x;
	emptyPoint.y = pointNormal.y + factor*pointNormal.normal_y;
	emptyPoint.z = pointNormal.z + factor*pointNormal.normal_z;

	return true;
}

inline bool genOccupiedPoint(const pcl::PointNormal	&pointNormal,
									  const float					gap,
									  pcl::PointXYZ				&emptyPoint)
{
	// assume the point normal is finite
	if(!pcl::isFinite<pcl::PointNormal>(pointNormal)) return false;

	// length
	const float length = norm(pointNormal);

	// factor
	const float factor = gap/length;

	// empty point
	emptyPoint.x = pointNormal.x - factor*pointNormal.normal_x;
	emptyPoint.y = pointNormal.y - factor*pointNormal.normal_y;
	emptyPoint.z = pointNormal.z - factor*pointNormal.normal_z;

	return true;
}

template <typename PointT>
inline bool genEmptyPoint(const PointT				&hitPoint, 
								  const pcl::PointXYZ	&sensorPosition, 
								  const float				gap,							  
								  pcl::PointXYZ			&emptyPoint)
{
	// check finite
	if(!pcl::isFinite<PointT>(hitPoint) || 
		!pcl::isFinite<pcl::PointXYZ>(sensorPosition))
		return false;

	// vector from a hit point to the origin (sensorPosition)
	pcl::PointXYZ P_O(sensorPosition.x - hitPoint.x,
							sensorPosition.y - hitPoint.y,
							sensorPosition.z - hitPoint.z);
	const float length = sqrt(P_O.x*P_O.x + P_O.y*P_O.y + P_O.z*P_O.z);
	if(length < std::numeric_limits<float>::epsilon()) return false;

	// factor
	const float factor = gap/length;

	// empty point
	emptyPoint.x = hitPoint.x + factor*P_O.x;
	emptyPoint.y = hitPoint.y + factor*P_O.y;
	emptyPoint.z = hitPoint.z + factor*P_O.z;

	return true;
}

template <typename PointT>
inline bool genOccupiedPoint(const PointT				&hitPoint, 
									  const pcl::PointXYZ	&sensorPosition, 
									  const float				gap,							  
									  pcl::PointXYZ			&occupiedPoint)
{
	// check finite
	if(!pcl::isFinite<PointT>(hitPoint) || 
		!pcl::isFinite<pcl::PointXYZ>(sensorPosition))
		return false;

	// vector from a hit point to the origin (sensorPosition)
	pcl::PointXYZ P_O(sensorPosition.x - hitPoint.x,
							sensorPosition.y - hitPoint.y,
							sensorPosition.z - hitPoint.z);
	const float length = sqrt(P_O.x*P_O.x + P_O.y*P_O.y + P_O.z*P_O.z);
	if(length < std::numeric_limits<float>::epsilon()) return false;

	// factor
	const float factor = gap/length;

	// occupied point
	occupiedPoint.x = hitPoint.x - factor*P_O.x;
	occupiedPoint.y = hitPoint.y - factor*P_O.y;
	occupiedPoint.z = hitPoint.z - factor*P_O.z;

	return true;
}

/** @brief Generate empty points just before hit points */
template <typename PointT>
pcl::PointCloud<pcl::PointXYZ>::Ptr
genEmptyPointCloud(const typename pcl::PointCloud<PointT>::ConstPtr		&pHitPointCloud,
						 const pcl::PointXYZ												&sensorPosition,
						 const float														gap,						 
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
			if(genEmptyPoint(pHitPointCloud->points[*iter], sensorPosition, gap, emptyPoint))
			{
				// empty point
				pEmptyPointCloud->push_back(emptyPoint);
			}
		}
	}
	else
	{
		// add empty points just before the hit pints
		for(size_t i = 0; i < pHitPointCloud->size(); i++)
		{
			// calculate empty point
			if(genEmptyPoint(pHitPointCloud->points[i], sensorPosition, gap, emptyPoint))
			{
				// empty point
				pEmptyPointCloud->push_back(emptyPoint);
			}
		}
	}

	return pEmptyPointCloud;
}

/** @brief Generate empty points just before hit points */
template <typename PointT>
void genEmptyPointCloudList(const std::vector<typename pcl::PointCloud<PointT>::Ptr>		&hitPointCloudList,
									 const PointXYZVList															&sensorPositionList,
									 const float																	gap,								 
									 PointXYZCloudPtrList														&emptyPointCloudList)
{
	// size check
	assert(hitPointCloudList.size() == sensorPositionList.size());
	emptyPointCloudList.clear();

	// for each hit point cloud, create an empty point list
	for(size_t i = 0; i < hitPointCloudList.size(); i++)
		emptyPointCloudList.push_back(genEmptyPointCloud<PointT>(hitPointCloudList[i], sensorPositionList[i], gap));
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

template <typename NormalT>
size_t numFiniteNormals(const pcl::PointCloud<NormalT>	&pointCloud,
							  const Indices							&indices)
{
	size_t count(0);
	for(Indices::const_iterator iter = indices.begin(); iter != indices.end(); ++iter)
	{
		// index
		assert(*iter >= 0 && *iter < static_cast<int>(pointCloud.points.size()));

		// check finite
		if(pcl::isFinite<NormalT>(pointCloud.points[*iter])) count++;
	}
	
	return count;
}

template <typename PointT>
size_t numFiniteNormals(const pcl::PointCloud<PointT>		&pointCloud,
							   const Indices							&indices,
								const pcl::PointXYZ					&sensorPosition)
{
	size_t count(0);
	pcl::PointXYZ dummyEmptyPoint;
	for(Indices::const_iterator iter = indices.begin(); iter != indices.end(); ++iter)
	{
		// index
		assert(*iter >= 0 && *iter < static_cast<int>(pointCloud.points.size()));

		// check finite
		if(genEmptyPoint<PointT>(pointCloud.points[*iter], sensorPosition, 1.0, dummyEmptyPoint)) count++;
	}
	
	return count;
}

/** @brief	Generate training data from a surface normal cloud and a sensor position
  * @Todo	Optimization or using getMatrixXfMap()
  */
template <typename PointT>
void generateTrainingData(const typename pcl::PointCloud<PointT>::ConstPtr		&pPointCloud,
								  const Indices													&indices,
								  const pcl::PointXYZ											&sensorPosition,
								  const float														gap,
								  MatrixPtr &pX, MatrixPtr &pXd, VectorPtr &pYYd)
{
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
	assert(gap > 0.f);
	const size_t D = 3;								// number of dimensions
	const size_t N  = numFiniteNormals(*pPointCloud, indices, sensorPosition);		// hit points, empty points
	const size_t Nd = 0;																				// no surface normals

	// memory allocation
	pX.reset(new Matrix(3*N, 3));		// hit/empty/occupied points
	pXd.reset(new Matrix(Nd, 3));		// no surface normals
	pYYd.reset(new Vector(3*N));		// hit/empty/occupied points

	// assignment
	pcl::PointXYZ emptyPoint;
	pcl::PointXYZ occupiedPoint;
	int i = 0;
	for(Indices::const_iterator iter = indices.begin(); iter != indices.end(); ++iter)
	{
		// index
		assert(*iter >= 0 && *iter < static_cast<int>(pPointCloud->points.size()));

		// point
		const PointT &point(pPointCloud->points[*iter]);

		// check finite
		if(!pcl::isFinite<PointT>(point)) continue;

		// hit point
		(*pX)(i, 0)		= point.x;
		(*pX)(i, 1)		= point.y;
		(*pX)(i, 2)		= point.z;
		(*pYYd)(i)		= 0.f;

		// empty point
		genEmptyPoint<PointT>(point, sensorPosition, gap, emptyPoint);
		(*pX)(N+i, 0)	= emptyPoint.x;
		(*pX)(N+i, 1)	= emptyPoint.y;
		(*pX)(N+i, 2)	= emptyPoint.z;
		(*pYYd)(N+i)	= -gap; // outside: negative distance

		// occupied point
		genOccupiedPoint<PointT>(point, sensorPosition, gap, emptyPoint);
		(*pX)(2*N+i, 0)	= occupiedPoint.x;
		(*pX)(2*N+i, 1)	= occupiedPoint.y;
		(*pX)(2*N+i, 2)	= occupiedPoint.z;
		(*pYYd)(2*N+i)		= gap; // inside: positive distance

		// next index
		i++;
	}
}

/** @brief	Generate training data from a surface normal cloud
  * @Todo	Optimization or using getMatrixXfMap()
  */
void generateTrainingData(const PointNormalCloudConstPtr		&pPointNormalCloud,
								  const Indices							&indices,
								  const float								gap,
								  MatrixPtr &pX, MatrixPtr &pXd, VectorPtr &pYYd)
{
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
	const bool fCreateEmptyPoint(gap > 0.f);	// create empty points or not
	const size_t D = 3;								// number of dimensions
	const size_t N  = numFiniteNormals(*pPointNormalCloud, indices);	// hit points, empty points, occupied points
	const size_t Nd = fCreateEmptyPoint ? 0 : N;	// surface normals

	// memory allocation
	if(fCreateEmptyPoint)
	{
		pX.reset(new Matrix(3*N, 3));				// hit/empty/occupied points
		pXd.reset(new Matrix(Nd, 3));				// -
		pYYd.reset(new Vector(3*N + Nd*D));		// hit/empty/occupied points
	}
	else
	{
		pX.reset(new Matrix(N, 3));				// hit points
		pXd.reset(new Matrix(Nd, 3));				// surface normals
		pYYd.reset(new Vector(N + Nd*D));		// hit points, surface normals
	}

	// assignment
	pcl::PointXYZ emptyPoint;
	pcl::PointXYZ occupiedPoint;
	int i = 0;
	for(Indices::const_iterator iter = indices.begin(); iter != indices.end(); ++iter)
	{
		// index
		assert(*iter >= 0 && *iter < static_cast<int>(pPointNormalCloud->points.size()));

		// point normal
		const pcl::PointNormal &pointNormal(pPointNormalCloud->points[*iter]);

		// check finite
		if(!pcl::isFinite<pcl::PointNormal>(pointNormal)) continue;

		// hit point
		(*pX)(i, 0)		= pointNormal.x;
		(*pX)(i, 1)		= pointNormal.y;
		(*pX)(i, 2)		= pointNormal.z;
		(*pYYd)(i)		= 0.f;

		// empty point, occupied point
		if(fCreateEmptyPoint)
		{
			// empty point
			genEmptyPoint(pointNormal, gap, emptyPoint);
			(*pX)(N+i, 0)	= emptyPoint.x;
			(*pX)(N+i, 1)	= emptyPoint.y;
			(*pX)(N+i, 2)	= emptyPoint.z;
			(*pYYd)(N+i)	= -gap; // outside: negative distance

			// occupied point
			genOccupiedPoint(pointNormal, gap, occupiedPoint);
			(*pX)(2*N+i, 0)	= occupiedPoint.x;
			(*pX)(2*N+i, 1)	= occupiedPoint.y;
			(*pX)(2*N+i, 2)	= occupiedPoint.z;
			(*pYYd)(2*N+i)		= gap; // inside: positive distance
		}
		// surface normal
		else
		{
			(*pXd)(i, 0)			= pointNormal.x;
			(*pXd)(i, 1)			= pointNormal.y;
			(*pXd)(i, 2)			= pointNormal.z;
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