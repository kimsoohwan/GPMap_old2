// GPMap
#include "io/io.hpp"								// loadPointClouds, savePointClouds, loadSensorPositionList
#include "visualization/display.hpp"		// show
#include "data/training_data.hpp"			// genEmptyPointList
#include "features/surface_normal.hpp"		// estimateSurfaceNormals
#include "common/common.hpp"					// getMinMax3DFromPointClouds
#include "octree/octree_gp.hpp"				// OctreePointCloud_GP
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
	const float gap = 0.0001;
	PointXYZCloudPtrList emptyPointCloudList;
	genEmptyPointCloudList<pcl::PointXYZ>(gap, sensorPositionList, hitPointCloudList, emptyPointCloudList);
	savePointClouds<pcl::PointXYZ>(emptyPointCloudList, strFileNames, strInputDataFolder, "_empty_points.pcd");		// original pcd files which are transformed in global coordinates
	loadPointClouds<pcl::PointXYZ>(emptyPointCloudList, strFileNames, strInputDataFolder, "_empty_points.pcd");		// original pcd files which are transformed in global coordinates
	PointXYZCloudPtrList hitEmptyPointCloudList;
	combinePointCloudList(hitPointCloudList, emptyPointCloudList, hitEmptyPointCloudList);
	show<pcl::PointXYZ>("Hit/Empty Points", hitEmptyPointCloudList);

	// [3] load/save surface normals
	PointNormalCloudPtrList pPointNormalClouds;
	////estimateSurfaceNormals<ByNearestNeighbors>(hitPointCloudList, sensorPositionList, false, 0.01, pPointNormalClouds);
	//estimateSurfaceNormals<ByMovingLeastSquares>(hitPointCloudList, sensorPositionList, false, 0.01, pPointNormalClouds);
	//savePointClouds<pcl::PointNormal>(pPointNormalClouds, strFileNames, strInputDataFolder, "_normals.pcd");		// original pcd files which are transformed in global coordinates
	loadPointClouds<pcl::PointNormal>(pPointNormalClouds, strFileNames, strInputDataFolder, "_normals.pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointNormal>("Surface Normals", pPointNormalClouds, 0.005, true, 0.001);

	// octree
	pcl::PointXYZ min_pt, max_pt;
	getMinMax3DFromPointClouds<pcl::PointNormal>(pPointNormalClouds, min_pt, max_pt);
	const double blockSize = 0.01; // 0.001
	const size_t nCellsPerBlock = 10; // 0.001
	//OctreePointCloud_GP<pcl::PointNormal> octree(blockSize, nCellsPerBlock, min_pt, max_pt);
	//pcl::octree::OctreePointCloud<pcl::PointNormal> octree(blockSize);
	//octree.setInputCloud(pPointNormalClouds[0]);
	//octree.defineBoundingBox();			//update bounding box automatically    
	//octree.addPointsFromInputCloud();	//add points in the tree
	OctreePointCloud_GP<pcl::PointNormal> octree(blockSize, nCellsPerBlock, min_pt, max_pt);
	octree.setInputCloud(pPointNormalClouds[0]);
	octree.addPointsFromInputCloud();
	octree.update();
	OctreeViewer<pcl::PointNormal> octree_viewer(pPointNormalClouds[0], octree);
	octree.setInputCloud(pPointNormalClouds[1]);
	octree.addPointsFromInputCloud();
	octree.update();
	OctreeViewer<pcl::PointNormal> octree_viewer2(pPointNormalClouds[1], octree);
	//octree.predict();


	//// GPMap
	//const float resolution = 0.1f;	// 10cm
	//const float blockSize = 1.f;		// 1m
	//GPMap gpmap(resolution, blockSize);
	//for(size_t i = 0; i < NUM_DATA; i++)
	//{
	//	gpmap.update(pPointClouds[i]);
	//	show gpmap
	//}

	system("pause");
}