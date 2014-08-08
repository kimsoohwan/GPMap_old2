#ifndef _TEST_OCTREE_GPMAP_HPP_
#define _TEST_OCTREE_GPMAP_HPP_

// Google Test
#include "gtest/gtest.h"

// GPMap
#include "io/io.hpp"								// loadPointClouds, savePointClouds, loadSensorPositionList
#include "common/common.hpp"					// getMinMax3DFromPointClouds
#include "octree/octree_gpmap.hpp"
using namespace GPMap;

class TestOctreeGPMap : public ::testing::Test,
								public OctreeGPMap<pcl::PointNormal>
{
public:
	TestOctreeGPMap()
		: BLOCK_SIZE(0.01),
		  NUM_CELLS_PER_AXIS(10),
		  INDEPENDENT_BCM(true), //INDEPENDENT_BCM(false),
		  POINT_DUPLICATION(false),
		  OctreeGPMap<pcl::PointNormal>(BLOCK_SIZE, NUM_CELLS_PER_AXIS, INDEPENDENT_BCM, POINT_DUPLICATION)
	{
		// some constants
		const size_t NUM_DATA = 4; 
		const std::string strInputDataFolder ("../../data/input/bunny/");
		const std::string strOutputDataFolder("../../data/output/bunny/");
		const std::string strFilenames_[] = {"bun000", "bun090", "bun180", "bun270"};
		StringList strFileNames(strFilenames_, strFilenames_ + NUM_DATA); 

		// [1] load/save hit points
		loadPointClouds<pcl::PointXYZ>(hitPointCloudList, strFileNames, strInputDataFolder, "_hit_points.pcd");		// original pcd files which are transformed in global coordinates

		// [2] load sensor positions
		loadSensorPositionList(sensorPositionList, strFileNames, strInputDataFolder, "_camera_position.txt");

		// [3] load/save surface normals
		loadPointClouds<pcl::PointNormal>(pPointNormalClouds, strFileNames, strInputDataFolder, "_normals.pcd");		// original pcd files which are transformed in global coordinates

		// [4] bounding box
		pcl::PointXYZ min_pt, max_pt;
		getMinMax3DFromPointClouds<pcl::PointNormal>(pPointNormalClouds, min_pt, max_pt);
		defineBoundingBox(min_pt, max_pt);
	}

protected:
		const double		BLOCK_SIZE;
		const size_t		NUM_CELLS_PER_AXIS;
		const bool			INDEPENDENT_BCM;
		const bool			POINT_DUPLICATION;

		PointXYZCloudPtrList			hitPointCloudList;
		PointXYZVector					sensorPositionList;
		PointNormalCloudPtrList		pPointNormalClouds;
};

/** @brief Update by mean vectors and covariance matrices */
TEST_F(TestOctreeGPMap, BoundingBoxTest)
{
	double minX, minY, minZ, maxX, maxY, maxZ;
	getBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
	std::cout << minX << ", " << minY << ", " << minZ << std::endl;
	EXPECT_EQ(minX, 0.0);
	EXPECT_EQ(minY, 0.0);
	EXPECT_EQ(minZ, 0.0);
}

#endif