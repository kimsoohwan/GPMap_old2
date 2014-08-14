#ifndef _POINT_CLOUD_INPUT_OUTPUT_HPP_
#define _POINT_CLOUD_INPUT_OUTPUT_HPP_

// STL
#include <string>
#include <vector>
#include <fstream>

// PCL
#include <pcl/point_types.h>		// pcl::PointXYZ, pcl::Normal, pcl::PointNormal
#include <pcl/point_cloud.h>		// pcl::PointCloud
#include <pcl/common/common.h>	// pcl::getMinMax3D
#include <pcl/filters/filter.h>	// pcl::removeNaNFromPointCloud
#include <pcl/io/pcd_io.h>			// pcl::io::loadPCDFile, savePCDFile
#include <pcl/io/ply_io.h>			// pcl::io::loadPLYFile, savePLYFile

// GPMap
#include "util/data_types.hpp"		// PointXYZVList
#include "util/filesystem.hpp"		// fileExtension

namespace GPMap {

template <typename PointT>
int loadPointCloud(const std::string								&strFilePath, 
						 typename pcl::PointCloud<PointT>::Ptr		&pCloud, 
						 const bool											fAccumulation = false)
{
	// point cloud
	typename pcl::PointCloud<PointT>::Ptr pTempCloud(new pcl::PointCloud<PointT>());

	// load the file based on the file extension
	int result = -2;
	const std::string strFileExtension(fileExtension(strFilePath));
	if(strFileExtension.compare(".pcd") == 0)		result = pcl::io::loadPCDFile<PointT>(strFilePath.c_str(), *pTempCloud);
	if(strFileExtension.compare(".ply") == 0)		result = pcl::io::loadPLYFile<PointT>(strFilePath.c_str(), *pTempCloud);
	

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
	// point cloud
	typename pcl::PointCloud<PointT>::Ptr pTempCloud(new pcl::PointCloud<PointT>());

	// save the file based on the file extension
	const std::string strFileExtension(fileExtension(strFilePath));
	if(strFileExtension.compare(".pcd") == 0)		pcl::io::savePCDFile<PointT>(strFilePath.c_str(), *pCloud, fBinary);
	if(strFileExtension.compare(".ply") == 0)		pcl::io::savePLYFile<PointT>(strFilePath.c_str(), *pCloud, fBinary);
}

template <typename PointT>
void loadPointClouds(std::vector<typename pcl::PointCloud<PointT>::Ptr>		&pPointClouds,
						   const StringList													&strFileNames, 
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
						   const StringList															&strFileNames, 
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


void loadSensorPositionList(PointXYZVList		&sensorPositionList, 
									 const StringList		&strFileNames, 
									 const std::string	&strPrefix = std::string(),
									 const std::string	&strSuffix = std::string())
{
	// resize
	sensorPositionList.resize(strFileNames.size());

	// for each file name
	for(size_t i = 0; i < strFileNames.size(); i++)
	{
		// file name
		const std::string strFileName = strPrefix + strFileNames[i] + strSuffix;

		// open
		std::ifstream fin(strFileName);
		fin >> sensorPositionList[i].x >> sensorPositionList[i].y >> sensorPositionList[i].z;
	}
}

}

#endif