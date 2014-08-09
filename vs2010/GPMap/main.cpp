#if 0

// GPMap
#include "io/io.hpp"								// loadPointClouds, savePointClouds, loadSensorPositionList
#include "visualization/cloud_viewer.hpp"	// show
#include "data/training_data.hpp"			// genEmptyPointList
#include "features/surface_normal.hpp"		// estimateSurfaceNormals
#include "common/common.hpp"					// getMinMaxPointXYZ
#include "octree/octree_gpmap.hpp"			// OctreeGPMap
#include "octree/octree_viewer.hpp"			// OctreeViewer
using namespace GPMap;

int main(int argc, char** argv)
{
	// files
	const size_t NUM_DATA = 4; 
	const std::string strInputDataFolder ("../../data/input/bunny/");
	const std::string strOutputDataFolder("../../data/output/bunny/");
	const std::string strFilenames_[] = {"bun000", "bun090", "bun180", "bun270"};
	StringList strFileNames(strFilenames_, strFilenames_ + NUM_DATA); 

	// [1] load/save hit points
	PointXYZCloudPtrList hitPointCloudList;
	//loadPointClouds<pcl::PointXYZ>(hitPointCloudList, strFileNames, strInputDataFolder, ".ply");					// original ply files which are transformed in global coordinates
	//savePointClouds<pcl::PointXYZ>(hitPointCloudList, strFileNames, strInputDataFolder, "_hit_points.pcd");		// original pcd files which are transformed in global coordinates
	loadPointClouds<pcl::PointXYZ>(hitPointCloudList, strFileNames, strInputDataFolder, "_hit_points.pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointXYZ>("Hit Points", hitPointCloudList);

	// [2] load sensor positions
	PointXYZVector sensorPositionList;
	loadSensorPositionList(sensorPositionList, strFileNames, strInputDataFolder, "_camera_position.txt");

	// [3] load/save empty points
	const float GAP = 0.0001;
	PointXYZCloudPtrList emptyPointCloudList;
	genEmptyPointCloudList<pcl::PointXYZ>(GAP, sensorPositionList, hitPointCloudList, emptyPointCloudList);
	savePointClouds<pcl::PointXYZ>(emptyPointCloudList, strFileNames, strInputDataFolder, "_empty_points.pcd");		// original pcd files which are transformed in global coordinates
	loadPointClouds<pcl::PointXYZ>(emptyPointCloudList, strFileNames, strInputDataFolder, "_empty_points.pcd");		// original pcd files which are transformed in global coordinates
	PointXYZCloudPtrList hitEmptyPointCloudList;
	combinePointCloudList(hitPointCloudList, emptyPointCloudList, hitEmptyPointCloudList);
	show<pcl::PointXYZ>("Hit/Empty Points", hitEmptyPointCloudList);

	// [3] load/save surface normals
	PointNormalCloudPtrList pointNormalCloudList;
	////estimateSurfaceNormals<ByNearestNeighbors>(hitPointCloudList, sensorPositionList, false, 0.01, pointNormalCloudList);
	//estimateSurfaceNormals<ByMovingLeastSquares>(hitPointCloudList, sensorPositionList, false, 0.01, pointNormalCloudList);
	//savePointClouds<pcl::PointNormal>(pointNormalCloudList, strFileNames, strInputDataFolder, "_normals.pcd");		// original pcd files which are transformed in global coordinates
	loadPointClouds<pcl::PointNormal>(pointNormalCloudList, strFileNames, strInputDataFolder, "_normals.pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointNormal>("Surface Normals", pointNormalCloudList, 0.005, true, 0.001);

	// gpmap
	pcl::PointXYZ min_pt, max_pt;
	getMinMaxPointXYZ<pcl::PointNormal>(pointNormalCloudList, min_pt, max_pt);
	const double BLOCK_SIZE = 0.01; // 0.001
	const size_t NUM_CELLS_PER_AXIS = 10;
	const bool INDEPENDENT_BCM = true;
	const bool POINT_DUPLICATION = false;
	//OctreeGPMap<pcl::PointNormal> gpmap(BLOCK_SIZE, nCellsPerBlock, min_pt, max_pt);
	//pcl::octree::OctreePointCloud<pcl::PointNormal> gpmap(BLOCK_SIZE);
	//gpmap.setInputCloud(pointNormalCloudList[0]);
	//gpmap.defineBoundingBox();			//update bounding box automatically    
	//gpmap.addPointsFromInputCloud();	//add points in the tree
	OctreeGPMap<pcl::PointNormal> gpmap(BLOCK_SIZE, NUM_CELLS_PER_AXIS, INDEPENDENT_BCM, POINT_DUPLICATION);
	gpmap.defineBoundingBox(min_pt, max_pt);
	gpmap.setInputCloud(pointNormalCloudList[0], GAP, sensorPositionList[0]);
	gpmap.addPointsFromInputCloud();
	gpmap.update();
	OctreeViewer<pcl::PointNormal> octree_viewer(pointNormalCloudList[0], gpmap);
	gpmap.setInputCloud(pointNormalCloudList[1], GAP, sensorPositionList[1]);
	gpmap.addPointsFromInputCloud();
	gpmap.update();
	OctreeViewer<pcl::PointNormal> octree_viewer2(pointNormalCloudList[1], gpmap);
	//gpmap.predict();


	//// GPMap
	//const float resolution = 0.1f;	// 10cm
	//const float BLOCK_SIZE = 1.f;		// 1m
	//GPMap gpmap(resolution, BLOCK_SIZE);
	//for(size_t i = 0; i < NUM_DATA; i++)
	//{
	//	gpmap.update(pPointClouds[i]);
	//	show gpmap
	//}

	system("pause");
}

#endif