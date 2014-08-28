#if 1
// Eigen
#include "serialization/eigen_serialization.hpp" // Eigen
// includes followings inside of it
//		- #define EIGEN_NO_DEBUG		// to speed up
//		- #define EIGEN_USE_MKL_ALL	// to use Intel Math Kernel Library
//		- #include <Eigen/Core>

// GPMap
#include "io/io.hpp"								// loadPointCloud, savePointCloud, loadSensorPositionList
#include "visualization/cloud_viewer.hpp"	// show
#include "features/surface_normal.hpp"		// estimateSurfaceNormals
#include "octree/macro_gpmap.hpp"			// macro_gpmap
using namespace GPMap;

int main(int argc, char** argv)
{
	// [0] GPMap - setting
	// CELL_SIZE = 0.001
	//const double	BLOCK_SIZE	= 0.003;		const size_t	NUM_CELLS_PER_AXIS	= 3;		// NUM_CELLS_PER_BLOCK = 3*3*3 = 27						BCM/iBCM (too small BLOCK_SIZE to produce non-smooth signed distance function)
	//const double	BLOCK_SIZE	= 0.01;		const size_t	NUM_CELLS_PER_AXIS	= 10;		// NUM_CELLS_PER_BLOCK = 10*10*10 = 1,000				BCM/iBCM
	//const double	BLOCK_SIZE	= 0.03;		const size_t	NUM_CELLS_PER_AXIS	= 30;		// NUM_CELLS_PER_BLOCK = 30*30*30 = 27,000			iBCM (sparse_ell for Der Obs): good?
	//const double	BLOCK_SIZE	= 0.06;		const size_t	NUM_CELLS_PER_AXIS	= 60;		// NUM_CELLS_PER_BLOCK = 60*60*60 = 216,000			iBCM (2*sparse_ell for Der Obs): very good?
	//const double	BLOCK_SIZE	= 0.15;		const size_t	NUM_CELLS_PER_AXIS	= 150;	// NUM_CELLS_PER_BLOCK = 150*150*150 = 3,375,000	iBCM (spase_ell for Func Obs)	
	double	BLOCK_SIZE;				std::cout << "Block Size? ";						std::cin >> BLOCK_SIZE;
	size_t	NUM_CELLS_PER_AXIS;	std::cout << "Number of cells per axis? ";	std::cin >> NUM_CELLS_PER_AXIS;

	// [0] setting - directory
	const std::string strInputDataFolder			("../../data/bunny/input/");
	const std::string strIntermediateDataFolder	("../../data/bunny/intermediate/");

	std::stringstream ss; 
	ss << "../../data/bunny/output/gpmap_block_size_" << BLOCK_SIZE 
												<< "_cell_size_" << static_cast<float>(BLOCK_SIZE)/static_cast<float>(NUM_CELLS_PER_AXIS) << "/";
	const std::string strGPMapDataFolder			(ss.str());
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
	std::string strLogFilePath = strOutputLogFolder + "gpmap.log";
	LogFile logFile;
	logFile.open(strLogFilePath);

	bool fDataPreprocessing;
	logFile << "[Data Preprocessing] - Do you wish to run? (0/1)";
	std::cin >> fDataPreprocessing;
	logFile << fDataPreprocessing << std::endl;

	if(fDataPreprocessing)
	{
		// [1] load/save hit points
		PointXYZCloudPtrList hitPointCloudPtrList;
		loadPointCloud<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strInputDataFolder, ".ply");				// original ply files which are transformed in global coordinates
		savePointCloud<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strIntermediateDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
		loadPointCloud<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strIntermediateDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
		show<pcl::PointXYZ>("Hit Points", hitPointCloudPtrList);

		PointXYZCloudPtr pAllHitPointCloud(new PointXYZCloud());
		for(size_t i = 0; i < hitPointCloudPtrList.size(); i++)	(*pAllHitPointCloud) += (*(hitPointCloudPtrList[i]));
		savePointCloud<pcl::PointXYZ>(pAllHitPointCloud, strFileNameAll, strIntermediateDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
		loadPointCloud<pcl::PointXYZ>(pAllHitPointCloud, strFileNameAll, strIntermediateDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
		show<pcl::PointXYZ>("All Hit Points", pAllHitPointCloud);

		// [2] load sensor positions
		PointXYZVList sensorPositionList;
		loadSensorPositionList(sensorPositionList, strObsFileNames, strInputDataFolder, "_camera_position.txt");
		assert(NUM_OBSERVATIONS == hitPointCloudPtrList.size() && NUM_OBSERVATIONS == sensorPositionList.size());

		// [3] load/save function observations
		PointNormalCloudPtrList funcObsCloudPtrList;
		unitRayBackVectors(hitPointCloudPtrList, sensorPositionList, funcObsCloudPtrList);
		savePointCloud<pcl::PointNormal>(funcObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_func_obs.pcd");
		loadPointCloud<pcl::PointNormal>(funcObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_func_obs.pcd");
		show<pcl::PointNormal>("Unit Ray Back Vectors", funcObsCloudPtrList, 0.005, 0.001);

		PointNormalCloudPtr pAllFuncObs(new PointNormalCloud());
		for(size_t i = 0; i < funcObsCloudPtrList.size(); i++)	*pAllFuncObs += *funcObsCloudPtrList[i];
		savePointCloud<pcl::PointNormal>(pAllFuncObs, strFileNameAll, strIntermediateDataFolder, "_func_obs.pcd");
		loadPointCloud<pcl::PointNormal>(pAllFuncObs, strFileNameAll, strIntermediateDataFolder, "_func_obs.pcd");
		show<pcl::PointNormal>("All Unit Ray Back Vectors", pAllFuncObs, 0.005, 0.001);

		// [4] load/save derivative observations
		PointNormalCloudPtrList derObsCloudPtrList;
		//estimateSurfaceNormals<ByNearestNeighbors>(hitPointCloudPtrList, sensorPositionList, false, 0.01, derObsCloudPtrList);
		estimateSurfaceNormals<ByMovingLeastSquares>(hitPointCloudPtrList, sensorPositionList, false, 0.01, derObsCloudPtrList);
		savePointCloud<pcl::PointNormal>(derObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_der_obs.pcd");
		loadPointCloud<pcl::PointNormal>(derObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_der_obs.pcd");
		show<pcl::PointNormal>("Surface Normals", derObsCloudPtrList, 0.005, 0.001);

		PointNormalCloudPtr pAllDerObs(new PointNormalCloud());
		for(size_t i = 0; i < derObsCloudPtrList.size(); i++)	(*pAllDerObs) += (*(derObsCloudPtrList[i]));
		savePointCloud<pcl::PointNormal>(pAllDerObs, strFileNameAll, strIntermediateDataFolder, "_der_obs.pcd");
		loadPointCloud<pcl::PointNormal>(pAllDerObs, strFileNameAll, strIntermediateDataFolder, "_der_obs.pcd");
		show<pcl::PointNormal>("All Surface Normals", pAllDerObs, 0.005, 0.001);

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
	}

	// combinations
	// ---------------------------------------------------------------------
	// |         Observations                        |       Process       |
	// |===================================================================|
	// |   (FuncObs)  |    (DerObs)     |  (AllObs)  |       | Incremental |
	// | hit points,  |   hit points,   |     all    | Batch |-------------|
	// | empty points | surface normals |            |       | BCM  | iBCM |
	// ---------------------------------------------------------------------

	// [6] GPMap - setting
	// constants
	//const double	BLOCK_SIZE						= 0.003;		//
	//const size_t	NUM_CELLS_PER_AXIS			= 3;			// CELL_SIZE = 0.001, NUM_CELLS_PER_BLOCK = 27
	//const double	BLOCK_SIZE						= 0.1;		//
	//const size_t	NUM_CELLS_PER_AXIS			= 100;		// CELL_SIZE = 0.001, NUM_CELLS_PER_BLOCK = 1,000,000 wow!!!
	//const double	BLOCK_SIZE						= 0.01;		// too big = too many training points
	//const size_t	NUM_CELLS_PER_AXIS			= 10;			// CELL_SIZE = 0.001, NUM_CELLS_PER_BLOCK = 1,000
	//const double	BLOCK_SIZE						= 0.027;		// too big = too many training points
	//const size_t	NUM_CELLS_PER_AXIS			= 27;			// CELL_SIZE = 0.001, NUM_CELLS_PER_BLOCK = 19,683
	//const double	BLOCK_SIZE						= 0.03;		//
	//const size_t	NUM_CELLS_PER_AXIS			= 30;			// CELL_SIZE = 0.001, NUM_CELLS_PER_BLOCK = 27,000
	const size_t	MIN_NUM_POINTS_TO_PREDICT	= 3;
	const float		GAP								= 0.001f;
	const bool		FLAG_BCM							= false;
	const bool		FLAG_INDEPENDENT_BCM			= true;
	const int		maxIterBeforeUpdate			= 0;			// 100
	std::string		strFileName;
	typedef GP::InfExactDerObs<float, GP::MeanZeroDerObs, GP::CovSparseMaternisoDerObs, GP::LikGaussDerObs>::Hyp Hyp;
	float sparse_ell, matern_ell, sigma_f2, sigma_n, sigma_nd;
	int		maxIter;				// 100
	size_t	numRandomBlocks;	// 100, <=0 for all

	// [7] GPMap - Function Observations
	strFileName = "gpmap_function_observations";
	logFile.open(strOutputLogFolder + strFileName + ".log");
	bool fRunFuncObs = true;
	//logFile << "[Function Observations] - Do you wish to run? (0/1)";
	//std::cin >> fRunFuncObs;
	//logFile << fRunFuncObs << std::endl;

	if(fRunFuncObs)
	{
		// [7-0] GPMap - Function Observations - Loading
		PointNormalCloudPtrList funcObsCloudPtrList;
		loadPointCloud<pcl::PointNormal>(funcObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_func_obs.pcd");

		PointNormalCloudPtr pAllFuncObs(new PointNormalCloud());
		loadPointCloud<pcl::PointNormal>(pAllFuncObs, strFileNameAll, strIntermediateDataFolder, "_func_obs.pcd");

		// [7-1] GPMap - Function Observations - Training
		bool fRunTraining = false;
		//logFile << "Do you wish to train hyperparameters? (0/1)";
		//std::cin >> fRunTraining;
		//logFile << fRunTraining << std::endl;

		// max iterations and number of random blocks
		if(fRunTraining)
		{
			logFile << "Learn Function Observations - Max Iterations? (0 for no training): ";
			std::cin >> maxIter;	logFile << maxIter << std::endl;;
			logFile << "Train - Num Random Blocks? (0 for all) ";	
			std::cin >> numRandomBlocks; logFile << numRandomBlocks << std::endl;;
		}

		// hyperparameters
		bool fUsePredefinedHyperparameters = true;
		//logFile << "Use Predefined Hyperparameters? (0/1) ";
		//std::cin >> fUsePredefinedHyperparameters;
		//logFile << fUsePredefinedHyperparameters << std::endl;;
		if(fUsePredefinedHyperparameters)
		{
			// outside negative distance
			sparse_ell	= 0.141505f;		// 0.107363f;
			matern_ell	= 0.131221f;		// 0.107363f;
			sigma_f2		= 0.245837f;		// 0.99985f;
			sigma_n		= 0.000271421f;	// 0.0034282f;
			sigma_nd		= 1.00000f;			// 0.0990157f; // will not be used here

			// outside positive distance
			sparse_ell	= 0.121951f;	
			matern_ell	= 0.103329f;	
			sigma_f2		= 0.409635f;	
			sigma_n		= 0.000299012f;	
			sigma_nd		= 1.00122f;			// will not be used here
		}
		else
		{
			std::cout << "sparse_ell: ";			std::cin >> sparse_ell;		// 0.1f
			std::cout << "matern_ell: ";			std::cin >> matern_ell;		// 0.1f
			std::cout << "sigma_f2: ";				std::cin >> sigma_f2;			// 1.f
			std::cout << "sigma_n: ";				std::cin >> sigma_n;			// 0.1f
			std::cout << "sigma_nd: ";				std::cin >> sigma_nd;		// 0.1f
		}
		logFile << "sparse_ell: "	<< sparse_ell	<< std::endl;
		logFile << "matern_ell: "	<< matern_ell	<< std::endl;
		logFile << "sigma_f2: "		<< sigma_f2		<< std::endl;
		logFile << "sigma_n: "		<< sigma_n		<< std::endl;
		logFile << "sigma_nd: "		<< sigma_nd		<< std::endl;

		// set hyperparameters
		Hyp logHyp;
		logHyp.cov(0) = log(sparse_ell);
		logHyp.cov(1) = log(matern_ell);
		logHyp.cov(2) = log(sigma_f2);
		logHyp.lik(0) = log(sigma_n);
		logHyp.lik(0) = log(sigma_nd);

		// training
		if(fRunTraining && maxIter > 0)
		{
			strFileName = "gpmap_function_observations_training";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_training<GP::MeanZeroDerObs, 
								GP::CovSparseMaternisoDerObs, 
								GP::LikGaussDerObs, 
								GP::InfExactDerObs>(BLOCK_SIZE,						// block size
														  NUM_CELLS_PER_AXIS,			// number of cells per each axie
														  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
														  logHyp,							// hyperparameters
														  pAllFuncObs,						// observations
														  GAP,								// gap
														  maxIter,							// number of iterations for training before update
														  numRandomBlocks);				// number of randomly selected blocks
		}

		// [7-2] GPMap - Function Observations - Prediction
		bool fRunPrediction = true;
		//logFile << "Do you wish to build GPMaps? (0/1)";
		//std::cin >> fRunPrediction;
		//logFile << fRunPrediction << std::endl;

		if(fRunPrediction)
		{
			// [7-2-1] GPMap - Function Observations - Batch
			strFileName = "gpmap_function_observations_batch";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_batch<GP::MeanZeroDerObs, 
							GP::CovSparseMaternisoDerObs, 
							GP::LikGaussDerObs, 
							GP::InfExactDerObs>(BLOCK_SIZE,						// block size
													  NUM_CELLS_PER_AXIS,			// number of cells per each axie
													  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
													  logHyp,							// hyperparameters
													  pAllFuncObs,						// observations
													  GAP,								// gap
													  maxIterBeforeUpdate,			// number of iterations for training before update
													  strOutputDataFolder + strFileName);	// save file path

			// [7-2-2] GPMap - Function Observations - Incremental (iBCM)
			strFileName = "gpmap_function_observations_iBCM";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_incremental<GP::MeanZeroDerObs, 
									GP::CovSparseMaternisoDerObs, 
									GP::LikGaussDerObs, 
									GP::InfExactDerObs>(BLOCK_SIZE,						// block size
															  NUM_CELLS_PER_AXIS,			// number of cells per each axie
															  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
															  FLAG_INDEPENDENT_BCM,			// independent BCM or BCM
															  logHyp,							// hyperparameters
															  funcObsCloudPtrList,			// observations
															  GAP,								// gap for free points
															  maxIterBeforeUpdate,			// number of iterations for training before update
															  strOutputDataFolder + strFileName);	// save file path

			// [7-2-3] GPMap - Function Observations - Incremental (BCM)
			strFileName = "gpmap_function_observations_BCM";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_incremental<GP::MeanZeroDerObs, 
									GP::CovSparseMaternisoDerObs, 
									GP::LikGaussDerObs, 
									GP::InfExactDerObs>(BLOCK_SIZE,						// block size
															  NUM_CELLS_PER_AXIS,			// number of cells per each axie
															  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
															  FLAG_BCM,							// independent BCM or BCM
															  logHyp,							// hyperparameters
															  funcObsCloudPtrList,			// observations
															  GAP,								// gap
															  maxIterBeforeUpdate,			// number of iterations for training before update
															  strOutputDataFolder + strFileName);	// save file path
		}
	}

	// [8] Derivative Observations
	strFileName = "gpmap_derivative_observations";
	logFile.open(strOutputLogFolder + strFileName + ".log");
	bool fRunDerObs = true;
	//logFile << "[Derivative Observations] - Do you wish to run? (0/1)";
	//std::cin >> fRunDerObs;
	//logFile << fRunDerObs << std::endl;

	if(fRunDerObs)
	{
		// [8-0] GPMap - Derivative Observations - Loading
		PointNormalCloudPtrList derObsCloudPtrList;
		loadPointCloud<pcl::PointNormal>(derObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_der_obs.pcd");

		PointNormalCloudPtr pAllDerObs(new PointNormalCloud());
		loadPointCloud<pcl::PointNormal>(pAllDerObs, strFileNameAll, strIntermediateDataFolder, "_der_obs.pcd");

		// [8-1] GPMap - Derivative Observations - Traininig
		bool fRunTraining = false;
		//logFile << "Do you wish to train hyperparameters? (0/1)";
		//std::cin >> fRunTraining;
		//logFile << fRunTraining << std::endl;

		// max iterations and number of random blocks
		if(fRunTraining)
		{
			logFile << "Learn Function Observations - Max Iterations? (0 for no training): ";
			std::cin >> maxIter;	logFile << maxIter << std::endl;;
			logFile << "Train - Num Random Blocks? (0 for all) ";	
			std::cin >> numRandomBlocks; logFile << numRandomBlocks << std::endl;;
		}

		// hyperparameters
		bool fUsePredefinedHyperparameters = true;
		//logFile << "Use Predefined Hyperparameters? (0/1) ";
		//std::cin >> fUsePredefinedHyperparameters;
		//logFile << fUsePredefinedHyperparameters << std::endl;;
		if(fUsePredefinedHyperparameters)
		{
			sparse_ell	= 0.0268108f;		// 0.107363f;
			matern_ell	= 0.0907317f;		// 0.107363f;
			sigma_f2		= 0.0144318f;		// 0.99985f;
			sigma_n		= 4.31269e-007f;	// 0.0034282f; will not be used
			sigma_nd		= 0.017184f;		// 0.0990157f;
		}
		else
		{
			std::cout << "sparse_ell: ";			std::cin >> sparse_ell;		// 0.1f
			std::cout << "matern_ell: ";			std::cin >> matern_ell;		// 0.1f
			std::cout << "sigma_f2: ";				std::cin >> sigma_f2;			// 1.f
			std::cout << "sigma_n: ";				std::cin >> sigma_n;			// 0.1f
			std::cout << "sigma_nd: ";				std::cin >> sigma_nd;		// 0.1f
		}
		logFile << "sparse_ell: "	<< sparse_ell	<< std::endl;
		logFile << "matern_ell: "	<< matern_ell	<< std::endl;
		logFile << "sigma_f2: "		<< sigma_f2		<< std::endl;
		logFile << "sigma_n: "		<< sigma_n		<< std::endl;
		logFile << "sigma_nd: "		<< sigma_nd		<< std::endl;

		// set hyperparameters
		Hyp logHyp;
		logHyp.cov(0) = log(sparse_ell);
		logHyp.cov(1) = log(matern_ell);
		logHyp.cov(2) = log(sigma_f2);
		logHyp.lik(0) = log(sigma_n);
		logHyp.lik(1) = log(sigma_nd);

		// training
		if(fRunTraining && maxIter > 0)
		{
			strFileName = "gpmap_derivative_observations_training";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_training<GP::MeanZeroDerObs, 
								GP::CovSparseMaternisoDerObs, 
								GP::LikGaussDerObs, 
								GP::InfExactDerObs>(BLOCK_SIZE,						// block size
														  NUM_CELLS_PER_AXIS,			// number of cells per each axie
														  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
														  logHyp,							// hyperparameters
														  pAllDerObs,						// observations
														  GAP,								// gap
														  maxIter,							// number of iterations for training before update
														  numRandomBlocks);				// number of randomly selected blocks
		}

		// [8-2] GPMap - Derivative Observations - Prediction
		bool fRunPrediction = true;
		//logFile << "Do you wish to build GPMaps? (0/1)";
		//std::cin >> fRunPrediction;
		//logFile << fRunPrediction << std::endl;

		if(fRunPrediction)
		{
			// [8-2-1] GPMap - Derivative Observations - Batch
			strFileName = "gpmap_derivative_observations_batch";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_batch<GP::MeanZeroDerObs, 
							GP::CovSparseMaternisoDerObs, 
							GP::LikGaussDerObs, 
							GP::InfExactDerObs>(BLOCK_SIZE,						// block size
													  NUM_CELLS_PER_AXIS,			// number of cells per each axie
													  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
													  logHyp,							// hyperparameters
													  pAllDerObs,						// observations
													  GAP,								// gap
													  maxIterBeforeUpdate,			// number of iterations for training before update
													  strOutputDataFolder + strFileName);	// save file path

			// [8-2-2] GPMap - Derivative Observations - Incremental (iBCM)
			strFileName = "gpmap_derivative_observations_iBCM";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_incremental<GP::MeanZeroDerObs, 
									GP::CovSparseMaternisoDerObs, 
									GP::LikGaussDerObs, 
									GP::InfExactDerObs>(BLOCK_SIZE,						// block size
															  NUM_CELLS_PER_AXIS,			// number of cells per each axie
															  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
															  FLAG_INDEPENDENT_BCM,			// independent BCM or BCM
															  logHyp,							// hyperparameters
															  derObsCloudPtrList,			// observations
															  GAP,								// gap
															  maxIterBeforeUpdate,			// number of iterations for training before update
															  strOutputDataFolder + strFileName);	// save file path

			// [8-2-3] GPMap - Derivative Observations - Incremental (BCM)
			strFileName = "gpmap_derivative_observations_BCM";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_incremental<GP::MeanZeroDerObs, 
									GP::CovSparseMaternisoDerObs, 
									GP::LikGaussDerObs, 
									GP::InfExactDerObs>(BLOCK_SIZE,						// block size
															  NUM_CELLS_PER_AXIS,			// number of cells per each axie
															  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
															  FLAG_BCM,							// independent BCM or BCM
															  logHyp,							// hyperparameters
															  derObsCloudPtrList,			// observations
															  GAP,								// gap
															  maxIterBeforeUpdate,			// number of iterations for training before update
															  strOutputDataFolder + strFileName);	// save file path
		}
	}

	// [9] All Observations
	strFileName = "gpmap_all_observations";
	logFile.open(strOutputLogFolder + strFileName + ".log");
	bool fRunAllObs;
	logFile << "[All Observations] - Do you wish to run? (0/1)";
	std::cin >> fRunAllObs;
	logFile << fRunAllObs << std::endl;

	if(fRunAllObs)
	{
		// [9-0] GPMap - All Observations - Loading
		PointNormalCloudPtrList allObsCloudPtrList;
		loadPointCloud<pcl::PointNormal>(allObsCloudPtrList, strObsFileNames, strIntermediateDataFolder, "_all_obs.pcd");

		PointNormalCloudPtr pAllObs(new PointNormalCloud());
		loadPointCloud<pcl::PointNormal>(pAllObs, strFileNameAll, strIntermediateDataFolder, "_all_obs.pcd");

		// [9-1] GPMap - All Observations - Traininig
		bool fRunTraining;
		logFile << "Do you wish to train hyperparameters? (0/1)";
		std::cin >> fRunTraining;
		logFile << fRunTraining << std::endl;

		// max iterations and number of random blocks
		if(fRunTraining)
		{
			logFile << "Learn Function Observations - Max Iterations? (0 for no training): ";
			std::cin >> maxIter;	logFile << maxIter << std::endl;;
			logFile << "Train - Num Random Blocks? (0 for all) ";	
			std::cin >> numRandomBlocks; logFile << numRandomBlocks << std::endl;;
		}

		// hyperparameters
		bool fUsePredefinedHyperparameters;
		logFile << "Use Predefined Hyperparameters? (0/1) ";
		std::cin >> fUsePredefinedHyperparameters;
		logFile << fUsePredefinedHyperparameters << std::endl;;
		if(fUsePredefinedHyperparameters)
		{
			sparse_ell	= 0.145733f;		// 0.107363f;
			matern_ell	= 0.134403f;		// 0.107363f;
			sigma_f2		= 0.370553f;		// 0.99985f;
			sigma_n		= 0.0200443f;		// 0.0034282f;
			sigma_nd		= 0.200443f;		// 0.0990157f;
		}
		else
		{
			std::cout << "sparse_ell: ";			std::cin >> sparse_ell;		// 0.1f
			std::cout << "matern_ell: ";			std::cin >> matern_ell;		// 0.1f
			std::cout << "sigma_f2: ";				std::cin >> sigma_f2;			// 1.f
			std::cout << "sigma_n: ";				std::cin >> sigma_n;			// 0.1f
			std::cout << "sigma_nd: ";				std::cin >> sigma_nd;		// 0.1f
		}
		logFile << "sparse_ell: "	<< sparse_ell	<< std::endl;
		logFile << "matern_ell: "	<< matern_ell	<< std::endl;
		logFile << "sigma_f2: "		<< sigma_f2		<< std::endl;
		logFile << "sigma_n: "		<< sigma_n		<< std::endl;
		logFile << "sigma_nd: "		<< sigma_nd		<< std::endl;

		// set hyperparameters
		Hyp logHyp;
		logHyp.cov(0) = log(sparse_ell);
		logHyp.cov(1) = log(matern_ell);
		logHyp.cov(2) = log(sigma_f2);
		logHyp.lik(0) = log(sigma_n);
		logHyp.lik(1) = log(sigma_nd);

		// training
		if(fRunTraining && maxIter > 0)
		{
			strFileName = "gpmap_all_observations_training";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_training<GP::MeanZeroDerObs, 
								GP::CovSparseMaternisoDerObs, 
								GP::LikGaussDerObs, 
								GP::InfExactDerObs>(BLOCK_SIZE,						// block size
														  NUM_CELLS_PER_AXIS,			// number of cells per each axie
														  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
														  logHyp,							// hyperparameters
														  pAllObs,							// observations
														  GAP,								// gap
														  maxIter,							// number of iterations for training before update
														  numRandomBlocks);				// number of randomly selected blocks
		}

		// [9-2] GPMap - All Observations - Prediction
		bool fRunPrediction;
		logFile << "Do you wish to build GPMaps? (0/1)";
		std::cin >> fRunPrediction;
		logFile << fRunPrediction << std::endl;

		if(fRunPrediction)
		{
			// [9-2-1] GPMap - All Observations - Batch
			strFileName = "gpmap_all_observations_batch";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_batch<GP::MeanZeroDerObs, 
							GP::CovSparseMaternisoDerObs, 
							GP::LikGaussDerObs, 
							GP::InfExactDerObs>(BLOCK_SIZE,						// block size
													  NUM_CELLS_PER_AXIS,			// number of cells per each axie
													  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
													  logHyp,							// hyperparameters
													  pAllObs,							// observations
													  GAP,								// gap for free points
													  maxIterBeforeUpdate,			// number of iterations for training before update
													  strOutputDataFolder + strFileName);	// save file path

			// [9-2-2] GPMap - All Observations - Incremental (iBCM)
			strFileName = "gpmap_all_observations_iBCM";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_incremental<GP::MeanZeroDerObs, 
									GP::CovSparseMaternisoDerObs, 
									GP::LikGaussDerObs, 
									GP::InfExactDerObs>(BLOCK_SIZE,						// block size
															  NUM_CELLS_PER_AXIS,			// number of cells per each axie
															  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
															  FLAG_INDEPENDENT_BCM,			// independent BCM or BCM
															  logHyp,							// hyperparameters
															  allObsCloudPtrList,			// observations
															  GAP	,								// gap for free points
															  maxIterBeforeUpdate,			// number of iterations for training before update
															  strOutputDataFolder + strFileName);	// save file path

			// [9-2-3] GPMap - All Observations - Incremental (BCM)
			strFileName = "gpmap_all_observations_BCM";
			logFile.open(strOutputLogFolder + strFileName + ".log");
			gpmap_incremental<GP::MeanZeroDerObs, 
									GP::CovSparseMaternisoDerObs, 
									GP::LikGaussDerObs, 
									GP::InfExactDerObs>(BLOCK_SIZE,						// block size
															  NUM_CELLS_PER_AXIS,			// number of cells per each axie
															  MIN_NUM_POINTS_TO_PREDICT,	// min number of points to predict
															  FLAG_BCM,							// independent BCM or BCM
															  logHyp,							// hyperparameters
															  allObsCloudPtrList,			// observations
															  GAP,								// gap for free points
															  maxIterBeforeUpdate,			// number of iterations for training before update
															  strOutputDataFolder + strFileName);	// save file path
		}
	}

	system("pause");
}

#endif