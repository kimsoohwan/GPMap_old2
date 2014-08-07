#ifndef _SURFACE_NORMAL_VECTOR_HPP_
#define _SURFACE_NORMAL_VECTOR_HPP_

// STL
#include <cmath>
#include <vector>

// PCL
#include <pcl/point_types.h>					// pcl::PointXYZ, pcl::Normal, pcl::PointNormal
#include <pcl/point_cloud.h>					// pcl::PointCloud
#include <pcl/point_cloud.h>					// pcl::PointCloud
#include <pcl/kdtree/kdtree_flann.h>		// pcl::search::KdTree
#include <pcl/features/normal_3d.h>			// pcl::NormalEstimation
#include <pcl/features/normal_3d_omp.h>	// pcl::NormalEstimationOMP
#include <pcl/surface/mls.h>					// pcl::MovingLeastSquares

// GPMap
#include "util/data_types.hpp"					// PointXYZVector
#include "remove_NAN.hpp"

namespace GPMap {


// pcl::Normal: float normal[3], curvature
pcl::PointCloud<pcl::Normal>::Ptr 
estimateSurfaceNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr		&pPointCloud,
							  const pcl::PointXYZ									&sensorPosition,
							  const bool												bSearchNearestNeighbor,
							  const float												param)
{
	// surface normal vectors
	// Create the normal estimation class, and pass the input dataset to it
#ifdef OPENMP
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
#else
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
#endif

	// Set the input points
	ne.setInputCloud(pPointCloud);

	// Set the view point
	ne.setViewPoint(sensorPosition.x, sensorPosition.y, sensorPosition.z);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	// Use a FLANN-based KdTree to perform neighborhood searches
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	ne.setSearchMethod(tree);

	// Use all neighbors in a sphere of radius
	// Specify the size of the local neighborhood to use when computing the surface normals
	if(bSearchNearestNeighbor)		ne.setKSearch(param);
	else									ne.setRadiusSearch(param);

	// Set the search surface (i.e., the points that will be used when search for the input points?neighbors)
	ne.setSearchSurface(pPointCloud);

	// Compute the surface normals
	pcl::PointCloud<pcl::Normal>::Ptr pNormals(new pcl::PointCloud<pcl::Normal>);
	ne.compute(*pNormals);

	return pNormals;
}

// pcl::PointNormal: float x, y, znormal[3], curvature
pcl::PointCloud<pcl::PointNormal>::Ptr 
estimateSurfaceNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr		&pPointCloud, 
							  const pcl::PointXYZ									&sensorPosition,
							  const bool												bSearchNearestNeighbor,
							  const float												param,
							  std::vector<int>										&index)
{
	// estimate surface normals
	pcl::PointCloud<pcl::Normal>::Ptr pNormals = estimateSurfaceNormals(pPointCloud, sensorPosition, bSearchNearestNeighbor, param);

	// concatenate
	pcl::PointCloud<pcl::PointNormal>::Ptr pPointNormals(new pcl::PointCloud<pcl::PointNormal>());
	pcl::concatenateFields<pcl::PointXYZ, pcl::Normal, pcl::PointNormal>(*pPointCloud, *pNormals, *pPointNormals);

	// extract NaN
	removeNaNNormalsFromPointCloud<pcl::PointNormal>(*pPointNormals, *pPointNormals, index);

	return pPointNormals;
}

// pcl::PointNormal: float x, y, znormal[3], curvature
pcl::PointCloud<pcl::PointNormal>::Ptr 
smoothAndNormalEstimation(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr	&cloud,
								  const double													radius)
{
	// Smoothing and normal estimation based on polynomial reconstruction
	// Moving Least Squares (MLS) surface reconstruction method can be used to smooth and resample noisy data

	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());

	// Init object (second point type is for the normals, even if unused)
#ifdef OPENMP
	pcl::MovingLeastSquaresOMP<pcl::PointXYZ, pcl::PointNormal> mls;
#else
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
#endif

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(kdtree);
	mls.setSearchRadius(radius);
	// void 	setPointDensity (int desired_num_points_in_radius);
	// void 	setDilationVoxelSize (float voxel_size)
	// void 	setDilationIterations (int iterations);
	// void 	setSqrGaussParam (double sqr_gauss_param)
	// void 	setPolynomialOrder (int order)
	// void 	setPolynomialFit (bool polynomial_fit)

	// Reconstruct
	// PCL v1.6
#if 1
	mls.setComputeNormals(true);

	// Output has the pcl::PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>());
	mls.process(*mls_points);
	return mls_points;
#else
	mls.reconstruct(*pPointCloud);

	// Output has the pcl::PointNormal type in order to store the normals calculated by MLS
	pPointNormals = mls.getOutputNormals();
	//mls.setOutputNormals(mls_points);
#endif
}

template <typename NormalT>
void normalizeSurfaceNormals(typename pcl::PointCloud<NormalT>::Ptr &pPointCloud)
{
	float length;
	for(unsigned int i = 0; i < pPointCloud->size(); i++)
	{
		length = sqrt(pPointCloud->points[i].normal_x * pPointCloud->points[i].normal_x
				      + pPointCloud->points[i].normal_y * pPointCloud->points[i].normal_y
					   + pPointCloud->points[i].normal_z * pPointCloud->points[i].normal_z);
		pPointCloud->points[i].normal_x /= length;
		pPointCloud->points[i].normal_y /= length;
		pPointCloud->points[i].normal_z /= length;
	}
}

// check if n is consistently oriented towards the viewpoint and flip otherwise
// angle between Psensor - Phit and Normal should be less than 90 degrees
// dot(Psensor - Phit, Normal) > 0
void flipSurfaceNormals(const pcl::PointXYZ							&sensorPosition,
								pcl::PointCloud<pcl::PointNormal>		&pointNormals)
{
	pcl::PointCloud<pcl::PointNormal>::iterator iter;
	for(iter	= pointNormals.begin(); iter != pointNormals.end(); iter++)
	{
		if((sensorPosition.x - iter->x) * iter->normal_x + 
		   (sensorPosition.y - iter->y) * iter->normal_y + 
		   (sensorPosition.z - iter->z) * iter->normal_z < 0)
		{
			iter->normal_x *= -1.f;
			iter->normal_y *= -1.f;
			iter->normal_z *= -1.f;
		}
	}
}

class ByNearestNeighbors {};
class ByMovingLeastSquares {};

template <typename EstimateMethod>
void estimateSurfaceNormals(const vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>			&pointClouds, 
									 const PointXYZVector												&sensorPositionList,
									 const bool																fSearchRadius, // SearchRadius or SearchK
									 const float															param,			// radius or k
									 vector<pcl::PointCloud<pcl::PointNormal>::Ptr>				&pointNormalClouds);

template <>
void estimateSurfaceNormals<ByNearestNeighbors>(const vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>		&pointClouds,
																const PointXYZVector												&sensorPositionList,
																const bool															fSearchNearestK, // SearchRadius or SearchNearestK
																const float															param,			// radius or k
																vector<pcl::PointCloud<pcl::PointNormal>::Ptr>			&pointNormalClouds)
{
	// resize
	pointNormalClouds.resize(pointClouds.size());

	// for each point cloud
	for(size_t i = 0; i < pointClouds.size(); i++)
	{
		std::cout << "Estimate surface normals: " << i << " ... ";
		pointNormalClouds[i] = estimateSurfaceNormals(pointClouds[i], 
																	 sensorPositionList[i], 
																	 fSearchNearestK, 
																	 param, 
																	 std::vector<int>());
		std::cout << pointNormalClouds[i]->size() << " normals." << std::endl;
	}
}

template <>
void estimateSurfaceNormals<ByMovingLeastSquares>(const vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>		&pointClouds,
																  const PointXYZVector												&sensorPositionList,
																  const bool															fSearchNearestK, // SearchRadius or SearchNearestK
																  const float															param,			// radius or k
																  vector<pcl::PointCloud<pcl::PointNormal>::Ptr>			&pointNormalClouds)
{
	// resize
	pointNormalClouds.resize(pointClouds.size());

	// for each point cloud
	for(size_t i = 0; i < pointClouds.size(); i++)
	{
		std::cout << "Estimate surface normals: " << i << " ... ";
		pointNormalClouds[i] = smoothAndNormalEstimation(pointClouds[i], param);
		flipSurfaceNormals(sensorPositionList[i], *(pointNormalClouds[i]));
		std::cout << pointNormalClouds[i]->size() << " normals." << std::endl;
	}
}

}
#endif