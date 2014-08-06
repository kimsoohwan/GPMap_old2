#ifndef _POINT_CLOUD_INPUT_OUTPUT_HPP_
#define _POINT_CLOUD_INPUT_OUTPUT_HPP_

// STL
#include <string>
#include <vector>
#include <fstream>

// Boost
#include <boost/filesystem.hpp>

// PCL
#include <pcl/point_types.h>		// pcl::PointXYZ, pcl::Normal, pcl::PointNormal
#include <pcl/point_cloud.h>		// pcl::PointCloud
#include <pcl/common/common.h>	// pcl::getMinMax3D
#include <pcl/filters/filter.h>	// pcl::removeNaNFromPointCloud
#include <pcl/io/pcd_io.h>			// pcl::io::loadPCDFile
#include <pcl/io/ply_io.h>			// pcl::io::loadPLYFile

// GPMap
#include "util/utility.hpp"		// PointXYZVector

namespace GPMap {

template <typename PointT>
int loadPointCloud(const std::string								&strFilePath, 
						 typename pcl::PointCloud<PointT>::Ptr		&pCloud, 
						 const bool											fAccumulation = false)
{
	// boost path
	boost::filesystem::path p(strFilePath);

	// point cloud
	typename pcl::PointCloud<PointT>::Ptr pTempCloud(new pcl::PointCloud<PointT>());

	// load the file based on the file extension
	int result = -2;
	const std::string fileExtension(p.extension().string());
	if(fileExtension.compare(".pcd") == 0)		result = pcl::io::loadPCDFile<PointT>(strFilePath.c_str(), *pTempCloud);
	if(fileExtension.compare(".ply") == 0)		result = pcl::io::loadPLYFile<PointT>(strFilePath.c_str(), *pTempCloud);
	

	// error 
	switch(result)
	{
		case -2:
		{
			PCL_ERROR("Unknown file extension!");
			system("pause");
			break;
		}
		
		case -1:
		{
			PCL_ERROR("Couldn't read the file!");
			system("pause");
			break;
		}
	}

    //remove NaN Points
    pcl::removeNaNFromPointCloud(*pTempCloud, *pTempCloud, std::vector<int>());

	// accumulate
	if(fAccumulation)		*pCloud += *pTempCloud;
	else						pCloud = pTempCloud;

	return pCloud->size();
}

template <typename PointT>
void savePointCloud(const std::string												&strFilePath, 
						  const typename pcl::PointCloud<PointT>::ConstPtr		&pCloud,
						  const bool														fBinary = true)
{
	// boost path
	boost::filesystem::path p(strFilePath);

	// point cloud
	typename pcl::PointCloud<PointT>::Ptr pTempCloud(new pcl::PointCloud<PointT>());

	// save the file based on the file extension
	const std::string fileExtension(p.extension().string());
	if(fileExtension.compare(".pcd") == 0)		savePCDFile<PointT>(strFilePath.c_str(), *pCloud, fBinary);
	if(fileExtension.compare(".ply") == 0)		savePLYFile<PointT>(strFilePath.c_str(), *pCloud, fBinary);
}

template <typename PointT>
void loadPointClouds(std::vector<typename pcl::PointCloud<PointT>::Ptr>		&pPointClouds,
						   const std::vector<std::string>								&strFileNames, 
						   const std::string													&strPrefix = std::string(),
						   const std::string													&strSuffix = std::string())
{
	// resize
	pPointClouds.resize(strFileNames.size());

	// load files
	for(size_t i = 0; i < strFileNames.size(); i++)
	{
		// load a file
		PCL_INFO("Loading %s ... ", strFileNames[i].c_str());
		PCL_INFO("%d points.\n", loadPointCloud<PointT>(strPrefix + strFileNames[i] + strSuffix, pPointClouds[i]));
	}
}

template <typename PointT>
void savePointClouds(const std::vector<typename pcl::PointCloud<PointT>::Ptr>		&pPointClouds,
						   const std::vector<std::string>										&strFileNames, 
						   const std::string															&strPrefix = std::string(),
						   const std::string															&strSuffix = std::string(),				 
						   const bool																	fBinary = true)
{
	// save the files
	for(size_t i = 0; i < strFileNames.size(); i++)
	{
		PCL_INFO("Saving %s ... ", strFileNames[i].c_str());
		savePointCloud<PointT>(strPrefix + strFileNames[i] + strSuffix, pPointClouds[i], fBinary);
		PCL_INFO("done.\n");
	}
}


void loadSensorPositions(PointXYZVector							&sensorPositions, 
								 const std::vector<std::string>		&strFileNames, 
								 const std::string						&strPrefix = std::string(),
								 const std::string						&strSuffix = std::string())
{
	// resize
	sensorPositions.resize(strFileNames.size());

	// for each file name
	for(size_t i = 0; i < strFileNames.size(); i++)
	{
		// file name
		const std::string strFileName = strPrefix + strFileNames[i] + strSuffix;

		// open
		std::ifstream fin(strFileName);
		fin >> sensorPositions[i].x >> sensorPositions[i].y >> sensorPositions[i].z;
	}
}

template <typename PointT>
void getMinMax3DFromPointClouds(const std::vector<typename pcl::PointCloud<PointT>::Ptr>		&pPointClouds,
										  pcl::PointXYZ &min_pt, pcl::PointXYZ &max_pt)
{
	PointT min_pt_temp, max_pt_temp;

	// for each point cloud
	for(size_t i = 0; i < pPointClouds.size(); i++)
	{
		// get min max
		pcl::getMinMax3D(*pPointClouds[i], min_pt_temp, max_pt_temp);

		// compare
		if(i == 0)
		{
			min_pt.x = min_pt_temp.x;
			min_pt.y = min_pt_temp.y;
			min_pt.z = min_pt_temp.z;

			max_pt.x = max_pt_temp.x;
			max_pt.y = max_pt_temp.y;
			max_pt.z = max_pt_temp.z;
		}
		else
		{
			min_pt.x = min(min_pt.x, min_pt_temp.x);
			min_pt.y = min(min_pt.y, min_pt_temp.y);
			min_pt.z = min(min_pt.z, min_pt_temp.z);

			max_pt.x = max(max_pt.x, max_pt_temp.x);
			max_pt.y = max(max_pt.y, max_pt_temp.y);
			max_pt.z = max(max_pt.z, max_pt_temp.z);
		}
	}

}

}

#endif