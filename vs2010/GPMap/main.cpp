#if 1

// GPMap
#include "io/io.hpp"								// loadPointClouds, savePointClouds, loadSensorPositionList
#include "visualization/cloud_viewer.hpp"	// show
#include "data/training_data.hpp"			// genEmptyPointList
#include "features/surface_normal.hpp"		// estimateSurfaceNormals
#include "common/common.hpp"					// getMinMaxPointXYZ
#include "octree/octree_gpmap.hpp"			// OctreeGPMap
#include "octree/octree_viewer.hpp"			// OctreeViewer
using namespace GPMap;

typedef OctreeGPMap<pcl::PointNormal, GP::MeanZeroDerObs, GP::CovSEisoDerObs, GP::LikGaussDerObs, GP::InfExactDerObs> OctreeGPMapType;

int main(int argc, char** argv)
{
	// files
	const size_t NUM_DATA = 4; 
	const std::string strInputDataFolder ("../../data/input/bunny/");
	const std::string strOutputDataFolder("../../data/output/bunny/");
	const std::string strFilenames_[] = {"bun000", "bun090", "bun180", "bun270"};
	StringList strFileNames(strFilenames_, strFilenames_ + NUM_DATA); 

	// [1] load/save hit points
	//PointXYZCloudPtrList hitPointCloudList;
	//loadPointClouds<pcl::PointXYZ>(hitPointCloudList, strFileNames, strInputDataFolder, ".ply");					// original ply files which are transformed in global coordinates
	//savePointClouds<pcl::PointXYZ>(hitPointCloudList, strFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//loadPointClouds<pcl::PointXYZ>(hitPointCloudList, strFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointXYZ>("Hit Points", hitPointCloudList);

	// [2] load sensor positions
	PointXYZVList sensorPositionList;
	loadSensorPositionList(sensorPositionList, strFileNames, strInputDataFolder, "_camera_position.txt");

	// [?] load/save empty points
	//const float GAP = 0.0001;
	//PointXYZCloudPtrList emptyPointCloudList;
	//genEmptyPointCloudList<pcl::PointXYZ>(GAP, sensorPositionList, hitPointCloudList, emptyPointCloudList);
	//savePointClouds<pcl::PointXYZ>(emptyPointCloudList, strFileNames, strInputDataFolder, "_empty_points.pcd");		// original pcd files which are transformed in global coordinates
	//loadPointClouds<pcl::PointXYZ>(emptyPointCloudList, strFileNames, strInputDataFolder, "_empty_points.pcd");		// original pcd files which are transformed in global coordinates
	//PointXYZCloudPtrList hitEmptyPointCloudList;
	//combinePointCloudList(hitPointCloudList, emptyPointCloudList, hitEmptyPointCloudList);
	//show<pcl::PointXYZ>("Hit/Empty Points", hitEmptyPointCloudList);

	// [4] load/save surface normals
	PointNormalCloudPtrList pointNormalCloudList;
	////estimateSurfaceNormals<ByNearestNeighbors>(hitPointCloudList, sensorPositionList, false, 0.01, pointNormalCloudList);
	//estimateSurfaceNormals<ByMovingLeastSquares>(hitPointCloudList, sensorPositionList, false, 0.01, pointNormalCloudList);
	//savePointClouds<pcl::PointNormal>(pointNormalCloudList, strFileNames, strInputDataFolder, "_normals.pcd");		// original pcd files which are transformed in global coordinates
	loadPointClouds<pcl::PointNormal>(pointNormalCloudList, strFileNames, strInputDataFolder, "_normals.pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointNormal>("Surface Normals", pointNormalCloudList, 0.005, true, 0.001);

	// [5] bounding box
	pcl::PointXYZ min_pt, max_pt;
	getMinMaxPointXYZ<pcl::PointNormal>(pointNormalCloudList, min_pt, max_pt);

	// [6] octree-based GPMap
	const double	BLOCK_SIZE = 0.01; // 0.001
	const size_t	NUM_CELLS_PER_AXIS = 10;
	const bool		INDEPENDENT_BCM = true;
	const bool		POINT_DUPLICATION = false;
	OctreeGPMapType gpmap(BLOCK_SIZE, NUM_CELLS_PER_AXIS, INDEPENDENT_BCM, POINT_DUPLICATION);
	gpmap.defineBoundingBox(min_pt, max_pt);

	// [7] observation 1
	const float GAP = 0.001; // 0.0001
	gpmap.setInputCloud(pointNormalCloudList[0], GAP, sensorPositionList[0]);
	gpmap.addPointsFromInputCloud();
	OctreeViewer<pcl::PointNormal, OctreeGPMapType> octree_viewer1(gpmap);

	// train
	const int MAX_ITER = 1000;
	const float ell(0.001f), sigma_f(1.f), sigma_n(0.01f), sigma_nd(0.1f);
	OctreeGPMapType::Hyp logHyp;
	logHyp.cov(0) = log(ell);
	logHyp.cov(1) = log(sigma_f);
	logHyp.lik(0) = log(sigma_n);
	logHyp.lik(1) = log(sigma_nd);
	gpmap.train(logHyp, MAX_ITER);
	std::cout << "hyp.mean = " << std::endl << logHyp.mean.array().exp().matrix() << std::endl << std::endl;
	std::cout << "hyp.cov = "  << std::endl << logHyp.cov.array().exp().matrix()  << std::endl << std::endl;
	std::cout << "hyp.lik = "  << std::endl << logHyp.lik.array().exp().matrix()  << std::endl << std::endl;

	system("pause");

	gpmap.update();
	OctreeViewer<pcl::PointNormal, OctreeGPMapType> octree_viewer2(gpmap);

	// [7] observations 2
	gpmap.setInputCloud(pointNormalCloudList[1], GAP, sensorPositionList[1]);
	gpmap.addPointsFromInputCloud();
	gpmap.update();
	OctreeViewer<pcl::PointNormal, OctreeGPMapType> octree_viewer3(gpmap);
	
	//// [7] observations 
	//for(size_t i = 0; i < NUM_DATA; i++)
	//{
	//	gpmap.setInputCloud(pointNormalCloudList[i], GAP, sensorPositionList[i]);
	//	gpmap.addPointsFromInputCloud();
	//	gpmap.update();
	//	OctreeViewer<pcl::PointNormal, OctreeGPMapType> octree_viewer(gpmap);
	//}

	system("pause");
}

#endif