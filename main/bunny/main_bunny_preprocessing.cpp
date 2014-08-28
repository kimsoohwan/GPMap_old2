#if 1
// Eigen
#include "serialization/eigen_serialization.hpp" // Eigen
// includes followings inside of it
//		- #define EIGEN_NO_DEBUG		// to speed up
//		- #define EIGEN_USE_MKL_ALL	// to use Intel Math Kernel Library
//		- #include <Eigen/Core>

// combinations
// ---------------------------------------------------------------------
// |         Observations                        |       Process       |
// |===================================================================|
// |   (FuncObs)  |    (DerObs)     |  (AllObs)  |       | Incremental |
// | hit points,  |   hit points,   |     all    | Batch |-------------|
// | empty points | surface normals |            |       | BCM  | iBCM |
// ---------------------------------------------------------------------

// GPMap
#include "io/io.hpp"								// loadPointCloud, savePointCloud, loadSensorPositionList
#include "visualization/cloud_viewer.hpp"	// show
#include "features/surface_normal.hpp"		// estimateSurfaceNormals
#include "../macro_gpmap.hpp"					// macro_gpmap
using namespace GPMap;

int main(int argc, char** argv)
{
	// [0] setting - directory
	const std::string strInputDataFolder			("../../data/bunny/input/");
	const std::string strIntermediateDataFolder	("../../data/bunny/intermediate/");
	const std::string strGPMapDataFolder			("../../data/bunny/output/gpmap/");
	const std::string strOutputDataFolder			("../../data/bunny/output/gpmap/meta_data/");
	const std::string strOutputLogFolder			(strOutputDataFolder + "log/");
	create_directory(strGPMapDataFolder);
	create_directory(strOutputDataFolder);
	create_directory(strOutputLogFolder);

	// [0] setting - observations
	const size_t NUM_OBSERVATIONS = 4; 
	const std::string strObsFileNames_[]	= {"bun000", "bun090", "bun180", "bun270"};
	const std::string strFileNameAll			=  "bunny_all";
	StringList strObsFileNames(strObsFileNames_, strObsFileNames_ + NUM_OBSERVATIONS); 

	// log file
	std::string strLogFilePath = strOutputLogFolder + "preprocessing.log";
	LogFile logFile;
	logFile.open(strLogFilePath);

	// [1-1] Hit Points - Sequential
	//PointXYZCloudPtrList hitPointCloudPtrList;
	//loadPointCloud<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strInputDataFolder, ".ply");				// original ply files which are transformed in global coordinates
	//savePointCloud<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strIntermediateDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//loadPointCloud<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strIntermediateDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointXYZ>("Hit Points", hitPointCloudPtrList);

	// [1-1] Hit Points - All
	//PointXYZCloudPtr pAllHitPointCloud(new PointXYZCloud());
	//for(size_t i = 0; i < hitPointCloudPtrList.size(); i++)	(*pAllHitPointCloud) += (*(hitPointCloudPtrList[i]));
	//savePointCloud<pcl::PointXYZ>(pAllHitPointCloud, strFileNameAll, strIntermediateDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//loadPointCloud<pcl::PointXYZ>(pAllHitPointCloud, strFileNameAll, strIntermediateDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointXYZ>("All Hit Points", pAllHitPointCloud);

	// [2] Sensor Positions
	//PointXYZVList sensorPositionList;
	//loadSensorPositionList(sensorPositionList, strObsFileNames, strInputDataFolder, "_camera_position.txt");
	//assert(NUM_OBSERVATIONS == hitPointCloudPtrList.size() && NUM_OBSERVATIONS == sensorPositionList.size());

	// [3-1] Function Observations (Hit Points + Unit Ray Back Vectors) - Sequential
	PointNormalCloudPtrList funcObsCloudPtrList;
	//unitRayBackVectors(hitPointCloudPtrList, sensorPositionList, funcObsCloudPtrList);
	//savePointCloud<pcl::PointNormal>(funcObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_func_obs.pcd");
	loadPointCloud<pcl::PointNormal>(funcObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_func_obs.pcd");
	//show<pcl::PointNormal>("Unit Ray Back Vectors", funcObsCloudPtrList, 0.005, 0.001);

	// [3-2] Function Observations (Hit Points + Unit Ray Back Vectors) - All
	PointNormalCloudPtr pAllFuncObs(new PointNormalCloud());
	//for(size_t i = 0; i < funcObsCloudPtrList.size(); i++)	*pAllFuncObs += *funcObsCloudPtrList[i];
	//savePointCloud<pcl::PointNormal>(pAllFuncObs, strFileNameAll, strIntermediateDataFolder, "_func_obs.pcd");
	loadPointCloud<pcl::PointNormal>(pAllFuncObs, strFileNameAll, strIntermediateDataFolder, "_func_obs.pcd");
	//show<pcl::PointNormal>("All Unit Ray Back Vectors", pAllFuncObs, 0.005, 0.001);

	// [4-1] Derivative Observations (Virtual Hit Points + Surface Normal Vectors) - Sequential
	PointNormalCloudPtrList derObsCloudPtrList;
	////estimateSurfaceNormals<ByNearestNeighbors>(hitPointCloudPtrList, sensorPositionList, false, 0.01, derObsCloudPtrList);
	//estimateSurfaceNormals<ByMovingLeastSquares>(hitPointCloudPtrList, sensorPositionList, false, 0.01, derObsCloudPtrList);
	//savePointCloud<pcl::PointNormal>(derObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_der_obs.pcd");
	loadPointCloud<pcl::PointNormal>(derObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_der_obs.pcd");
	//show<pcl::PointNormal>("Surface Normals", derObsCloudPtrList, 0.005, 0.001);

	// [4-2] Derivative Observations (Virtual Hit Points + Surface Normal Vectors) - All
	PointNormalCloudPtr pAllDerObs(new PointNormalCloud());
	//for(size_t i = 0; i < derObsCloudPtrList.size(); i++)	(*pAllDerObs) += (*(derObsCloudPtrList[i]));
	//savePointCloud<pcl::PointNormal>(pAllDerObs, strFileNameAll, strIntermediateDataFolder, "_der_obs.pcd");
	loadPointCloud<pcl::PointNormal>(pAllDerObs, strFileNameAll, strIntermediateDataFolder, "_der_obs.pcd");
	//show<pcl::PointNormal>("All Surface Normals", pAllDerObs, 0.005, 0.001);

	// [5-1] Partitioning Function Observations - Sequential
	// [5-2] Partitioning Function Observations - All
	// [5-3] Partitioning Derivative Observations - Sequential
	// [5-4] Partitioning Derivative Observations - All
	// density
	const size_t maxLimitOfNumObsInLeafNode = 100;
	typedef pcl::octree::OctreePointCloudDensity<pcl::PointNormal> PointNormalOctree;


	PointNormalOctree octree(0.03f);
	octree.setInputCloud(pAllDerObs);
	octree.addPointsFromInputCloud();
	PointNormalOctree::LeafNodeIterator iter(octree);
	size_t nLeafNodes(0), nPoint;
	size_t minNumPointsInLeafNode	= std::numeric_limits<size_t>::max();
	size_t maxNumPointsInLeafNode	= std::numeric_limits<size_t>::min();
	while(*++iter)
	{
		// leaf node counter
		nLeafNodes++;

		// leaf node
		PointNormalOctree::LeafNode *pLeafNode = static_cast<PointNormalOctree::LeafNode *>(iter.getCurrentOctreeNode());
		logFile << nLeafNodes << ": " << pLeafNode->getPointCounter() << std::endl;

		// min, max
		minNumPointsInLeafNode	= std::min<size_t>(minNumPointsInLeafNode, pLeafNode->getPointCounter());
		maxNumPointsInLeafNode	= std::max<size_t>(maxNumPointsInLeafNode, pLeafNode->getPointCounter());
	}

	// number of partitions
	const size_t numPartitions = ceil(static_cast<float>(maxNumPointsInLeafNode) / static_cast<float>(maxLimitOfNumObsInLeafNode));


	// [5] load/save all observations
	PointNormalCloudPtrList allObsCloudPtrList;
	combinePointCloud<pcl::PointNormal>(funcObsCloudPtrList, derObsCloudPtrList, allObsCloudPtrList);
	savePointCloud<pcl::PointNormal>(allObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_all_obs.pcd");
	loadPointCloud<pcl::PointNormal>(allObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_all_obs.pcd");
	show<pcl::PointNormal>("All Observation List", allObsCloudPtrList, 0.005, 0.001);

	PointNormalCloudPtr pAllObs(new PointNormalCloud());
	*pAllObs += *pAllFuncObs;
	*pAllObs += *pAllDerObs;
	savePointCloud<pcl::PointNormal>(pAllObs, strFileNameAll, strIntermediateDataFolder, "_all_obs.pcd");
	loadPointCloud<pcl::PointNormal>(pAllObs, strFileNameAll, strIntermediateDataFolder, "_all_obs.pcd");
	show<pcl::PointNormal>("All Observations", pAllDerObs, 0.005, 0.001);

	system("pause");
}

#endif