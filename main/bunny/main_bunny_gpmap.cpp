#if 1
// Eigen
#include "serialization/eigen_serialization.hpp" // Eigen
//#define EIGEN_NO_DEBUG		// to speed up
//#define EIGEN_USE_MKL_ALL	// to use Intel Math Kernel Library
//#include <Eigen/Core>

// GPMap
#include "io/io.hpp"								// loadPointClouds, savePointClouds, loadSensorPositionList
#include "visualization/cloud_viewer.hpp"	// show
#include "features/surface_normal.hpp"		// estimateSurfaceNormals
#include "../macro_gpmap.hpp"					// macro_gpmap
#include "octomap/octomap.hpp"				// Octomap

using namespace GPMap;

typedef OctreeGPMap<pcl::PointNormal, GP::MeanZeroDerObs, GP::CovSEisoDerObs, GP::LikGaussDerObs, GP::InfExactDerObs> MyOctreeGPMapT;

int main(int argc, char** argv)
{
	// setting
	const size_t NUM_OBSERVATIONS = 4; 
	const std::string strInputDataFolder ("../../data/input/bunny/");
	const std::string strOutputDataFolder("../../data/output/bunny/");
	const std::string strObsFileNames_[]	= {"bun000", "bun090", "bun180", "bun270"};
	const std::string strFileNameAll		=  "bunny";
	StringList strObsFileNames(strObsFileNames_, strObsFileNames_ + NUM_OBSERVATIONS); 

	// log file
	std::string strLogFilePath = strOutputDataFolder + "gpmap.log";
	LogFile logFile;
	logFile.open(strLogFilePath);

	// [1] load/save hit points
	//PointXYZCloudPtrList hitPointCloudPtrList;
	//loadPointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strInputDataFolder, ".ply");			// original ply files which are transformed in global coordinates
	//savePointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//loadPointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointXYZ>("Hit Points", hitPointCloudPtrList);

	//PointXYZCloudPtr pAllHitPointCloud(new PointXYZCloud());
	//for(size_t i = 0; i < hitPointCloudPtrList.size(); i++)	(*pAllHitPointCloud) += (*(hitPointCloudPtrList[i]));
	//savePointCloud<pcl::PointXYZ>(pAllHitPointCloud, strFileNameAll, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//loadPointCloud<pcl::PointXYZ>(pAllHitPointCloud, strFileNameAll, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointXYZ>("All Hit Points", pAllHitPointCloud);

	// [2] load sensor positions
	//PointXYZVList sensorPositionList;
	//loadSensorPositionList(sensorPositionList, strObsFileNames, strInputDataFolder, "_camera_position.txt");
	//assert(NUM_OBSERVATIONS == hitPointCloudPtrList.size() && NUM_OBSERVATIONS == sensorPositionList.size());

	// [4] load/save surface normals
	PointNormalCloudPtrList pointNormalCloudList;
	////estimateSurfaceNormals<ByNearestNeighbors>(hitPointCloudPtrList, sensorPositionList, false, 0.01, pointNormalCloudList);
	//estimateSurfaceNormals<ByMovingLeastSquares>(hitPointCloudPtrList, sensorPositionList, false, 0.01, pointNormalCloudList);
	//savePointClouds<pcl::PointNormal>(pointNormalCloudList, strObsFileNames, strInputDataFolder, "_normals.pcd");
	loadPointClouds<pcl::PointNormal>(pointNormalCloudList, strObsFileNames, strInputDataFolder, "_normals.pcd");
	//show<pcl::PointNormal>("Surface Normals", pointNormalCloudList, 0.005, 0.001);

	PointNormalCloudPtr pAllPointNormalCloud(new PointNormalCloud());
	//for(size_t i = 0; i < pointNormalCloudList.size(); i++)	(*pAllPointNormalCloud) += (*(pointNormalCloudList[i]));
	//savePointCloud<pcl::PointNormal>(pAllPointNormalCloud, strFileNameAll, strInputDataFolder, "_normals.pcd");
	loadPointCloud<pcl::PointNormal>(pAllPointNormalCloud, strFileNameAll, strInputDataFolder, "_normals.pcd");
	//show<pcl::PointNormal>("All Surface Normals", pAllPointNormalCloud, 0.005, 0.001);

	// [5] GPMap - setting
	const double	BLOCK_SIZE						= 0.003;		// 0.01
	const size_t	NUM_CELLS_PER_AXIS			= 3;			// 10
	const bool		POINT_DUPLICATION				= false;
	const size_t	MIN_NUM_POINTS_TO_PREDICT	= 3;
	const bool		NO_DUPLICATED_POINTS			= false;
	const bool		INDEPENDENT_BCM				= true;
	const bool		BCM								= false;
	const float		GAP								= 0.001f;
	const float		NO_GAP							= 0.f;
	const int		maxIterBeforeUpdate			= 0;			// 100
	std::string		strFileName;

	// hyperparameters
	//const float ell(0.1f), sigma_f(1.f), sigma_n(0.01f), sigma_nd(0.1f);
	const float ell			= 0.107467f;		// 0.107363f;
	const float sigma_f		= 0.99968f;			// 0.99985f;
	const float sigma_n		= 0.00343017f;		// 0.0034282f;
	const float sigma_nd		= 0.0985929f;		// 0.0990157f;
	MyOctreeGPMapT::Hyp logHyp;
	logHyp.cov(0) = log(ell);
	logHyp.cov(1) = log(sigma_f);
	logHyp.lik(0) = log(sigma_n);
	logHyp.lik(1) = log(sigma_nd);

	// [5-0] traininig
	//strFileName = "gpmap_meanZero_covSEiso_training";
	//logFile.open(strOutputDataFolder + strFileName + ".log");
	//macro_gpmap_training<pcl::PointNormal, 
	//							GP::MeanZeroDerObs, GP::CovSEisoDerObs, GP::LikGaussDerObs, 
	//							GP::InfExactDerObs>(BLOCK_SIZE,						// block size
	//													  NUM_CELLS_PER_AXIS,			// number of cells per each axie
	//													  BCM,								// independent BCM
	//													  NO_DUPLICATED_POINTS,			// no duplicated points
	//													  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
	//													  logHyp,							// hyperparameters
	//													  pAllPointNormalCloud,			// observations
	//													  NO_GAP);							// gap for free points

	// [5-1] GPMap - batch - BCM - derivative obs.
	strFileName = "gpmap_meanZero_covSEiso_batch_BCM_der";
	logFile.open(strOutputDataFolder + strFileName + ".log");
	macro_gpmap<pcl::PointNormal, 
					GP::MeanZeroDerObs, GP::CovSEisoDerObs, GP::LikGaussDerObs, 
					GP::InfExactDerObs>(BLOCK_SIZE,						// block size
											  NUM_CELLS_PER_AXIS,			// number of cells per each axie
											  BCM,								// independent BCM
											  NO_DUPLICATED_POINTS,			// no duplicated points
											  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
											  logHyp,							// hyperparameters
											  pAllPointNormalCloud,			// observations
											  NO_GAP,							// gap for free points
											  maxIterBeforeUpdate,			// number of iterations for training before update
											  strOutputDataFolder + strFileName);	// save file path

	// [5-2] GPMap - batch - BCM - function obs.
	strFileName = "gpmap_meanZero_covSEiso_batch_BCM_func";
	logFile.open(strOutputDataFolder + strFileName + ".log");
	macro_gpmap<pcl::PointNormal, 
					GP::MeanZeroDerObs, GP::CovSEisoDerObs, GP::LikGaussDerObs, 
					GP::InfExactDerObs>(BLOCK_SIZE,						// block size
											  NUM_CELLS_PER_AXIS,			// number of cells per each axie
											  BCM,								// independent BCM
											  NO_DUPLICATED_POINTS,			// no duplicated points
											  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
											  logHyp,							// hyperparameters
											  pAllPointNormalCloud,			// observations
											  GAP,								// gap for free points
											  maxIterBeforeUpdate,			// number of iterations for training before update
											  strOutputDataFolder + strFileName);	// save file path

	// [5-3] GPMap - batch - independent BCM - derivative obs.
	strFileName = "gpmap_meanZero_covSEiso_batch_iBCM_der";
	logFile.open(strOutputDataFolder + strFileName + ".log");
	macro_gpmap<pcl::PointNormal, 
					GP::MeanZeroDerObs, GP::CovSEisoDerObs, GP::LikGaussDerObs, 
					GP::InfExactDerObs>(BLOCK_SIZE,						// block size
											  NUM_CELLS_PER_AXIS,			// number of cells per each axie
											  INDEPENDENT_BCM,				// independent BCM
											  NO_DUPLICATED_POINTS,			// no duplicated points
											  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
											  logHyp,							// hyperparameters
											  pAllPointNormalCloud,			// observations
											  NO_GAP,							// gap for free points
											  maxIterBeforeUpdate,			// number of iterations for training before update
											  strOutputDataFolder + strFileName);	// save file path

	// [5-4] GPMap - batch - independent BCM - function obs.
	strFileName = "gpmap_meanZero_covSEiso_batch_iBCM_func";
	logFile.open(strOutputDataFolder + strFileName + ".log");
	macro_gpmap<pcl::PointNormal, 
					GP::MeanZeroDerObs, GP::CovSEisoDerObs, GP::LikGaussDerObs, 
					GP::InfExactDerObs>(BLOCK_SIZE,						// block size
											  NUM_CELLS_PER_AXIS,			// number of cells per each axie
											  INDEPENDENT_BCM,				// independent BCM
											  NO_DUPLICATED_POINTS,			// no duplicated points
											  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
											  logHyp,							// hyperparameters
											  pAllPointNormalCloud,			// observations
											  GAP,								// gap for free points
											  maxIterBeforeUpdate,			// number of iterations for training before update
											  strOutputDataFolder + strFileName);	// save file path

	// [5-5] GPMap - incremental update - BCM - derivative obs.
	strFileName = "gpmap_meanZero_covSEiso_incremental_BCM_der";
	logFile.open(strOutputDataFolder + strFileName + ".log");
	macro_gpmap<pcl::PointNormal, 
					GP::MeanZeroDerObs, GP::CovSEisoDerObs, GP::LikGaussDerObs, 
					GP::InfExactDerObs>(BLOCK_SIZE,						// block size
											  NUM_CELLS_PER_AXIS,			// number of cells per each axie
											  BCM,								// independent BCM
											  NO_DUPLICATED_POINTS,			// no duplicated points
											  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
											  logHyp,							// hyperparameters
											  pointNormalCloudList,			// observations
											  NO_GAP,							// gap for free points
											  maxIterBeforeUpdate,			// number of iterations for training before update
											  strOutputDataFolder + strFileName);	// save file path


	// [5-6] GPMap - incremental update - BCM - function obs.
	strFileName = "gpmap_meanZero_covSEiso_incremental_BCM_func";
	logFile.open(strOutputDataFolder + strFileName + ".log");
	macro_gpmap<pcl::PointNormal, 
					GP::MeanZeroDerObs, GP::CovSEisoDerObs, GP::LikGaussDerObs, 
					GP::InfExactDerObs>(BLOCK_SIZE,						// block size
											  NUM_CELLS_PER_AXIS,			// number of cells per each axie
											  BCM,								// independent BCM
											  NO_DUPLICATED_POINTS,			// no duplicated points
											  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
											  logHyp,							// hyperparameters
											  pointNormalCloudList,			// observations
											  GAP,								// gap for free points
											  maxIterBeforeUpdate,			// number of iterations for training before update
											  strOutputDataFolder + strFileName);	// save file path

	// [5-7] GPMap - incremental update - independent BCM - derivative obs.
	strFileName = "gpmap_meanZero_covSEiso_incremental_iBCM_der";
	logFile.open(strOutputDataFolder + strFileName + ".log");
	macro_gpmap<pcl::PointNormal, 
					GP::MeanZeroDerObs, GP::CovSEisoDerObs, GP::LikGaussDerObs, 
					GP::InfExactDerObs>(BLOCK_SIZE,						// block size
											  NUM_CELLS_PER_AXIS,			// number of cells per each axie
											  INDEPENDENT_BCM,				// independent BCM
											  NO_DUPLICATED_POINTS,			// no duplicated points
											  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
											  logHyp,							// hyperparameters
											  pointNormalCloudList,			// observations
											  NO_GAP,							// gap for free points
											  maxIterBeforeUpdate,			// number of iterations for training before update
											  strOutputDataFolder + strFileName);	// save file path



	// [5-8] GPMap - incremental update - independent BCM - function obs.
	strFileName = "gpmap_meanZero_covSEiso_incremental_iBCM_func";
	logFile.open(strOutputDataFolder + strFileName + ".log");
	macro_gpmap<pcl::PointNormal, 
					GP::MeanZeroDerObs, GP::CovSEisoDerObs, GP::LikGaussDerObs, 
					GP::InfExactDerObs>(BLOCK_SIZE,						// block size
											  NUM_CELLS_PER_AXIS,			// number of cells per each axie
											  INDEPENDENT_BCM,				// independent BCM
											  NO_DUPLICATED_POINTS,			// no duplicated points
											  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
											  logHyp,							// hyperparameters
											  pointNormalCloudList,			// observations
											  GAP,								// gap for free points
											  maxIterBeforeUpdate,			// number of iterations for training before update
											  strOutputDataFolder + strFileName);	// save file path

/*
	// [6] Octomap for GPMap - setting
	const double	RESOLUTION = 0.001;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pPointXYZICloud;
	std::string strFileName;

	// [6-1] GPMap - batch - BCM - derivative obs.
	// Mean min: -0.0798497, max: 0.474393
	// Var  min: 9.99999e-007, max: 0.00999929
	strFileName = "gpmap_meanZero_covSEiso_batch_BCM_der";
	loadPointCloud<pcl::PointXYZI>(pPointXYZICloud, strOutputDataFolder + strFileName + ".pcd");
	Octomap<NO_COLOR> octomap1(RESOLUTION, *pPointXYZICloud, 0, 1);
	octomap1.save(strOutputDataFolder + strFileName);
	//Octomap<COLOR>		octomap1c(RESOLUTION, *pPointXYZICloud, 0, 1, 9.99999e-007, 0.00999929); 
	Octomap<COLOR>		octomap1c_default(RESOLUTION, *pPointXYZICloud, 0, 1); 
	octomap1c_default.save(strOutputDataFolder + strFileName + "_default_color");
	while(true)
	{
		float minVar; std::cout << "min var: "; std::cin >> minVar;
		float maxVar; std::cout << "max var: "; std::cin >> maxVar;
		if(minVar == maxVar) break;
		Octomap<COLOR>		octomap1c(RESOLUTION, *pPointXYZICloud, 0, 1, minVar, maxVar); 
		octomap1c.save(strOutputDataFolder + strFileName + "_color");
	}
*/
	// [6] evaluation
	//logFile << "============= Evaluation =============" << std::endl;
	//unsigned int num_points, num_voxels_correct, num_voxels_wrong, num_voxels_unknown;
	//octomap.evaluate<pcl::PointXYZ, pcl::PointXYZ>(hitPointCloudPtrList, sensorPositionList,
	//															  num_points, num_voxels_correct, num_voxels_wrong, num_voxels_unknown);
	//logFile << "Number of hit points: " << num_points << std::endl;
	//logFile << "Number of correct voxels: " << num_voxels_correct << std::endl;
	//logFile << "Number of wrong voxels: " << num_voxels_wrong << std::endl;
	//logFile << "Number of unknown voxels: " << num_voxels_unknown << std::endl;
	//logFile << "Correct rate (correct/(correct+wrong)): " << static_cast<float>(num_voxels_correct)/static_cast<float>(num_voxels_correct+num_voxels_wrong) << std::endl;

	//// save
	//while(true)
	//{
	//	float occupancyThreshold; std::cout << "Occupancy Threshold: "; std::cin >> occupancyThreshold; std::cout << occupancyThreshold << std::endl;
	//	std::stringstream ss;
	//	ss << strOutputDataFolder << "gpmap_bunny_upto_" << i << "_" << occupancyThreshold;
	//	gpmap.saveAsOctomap(ss.str(), occupancyThreshold, false);
	//}

	system("pause");
}

#endif