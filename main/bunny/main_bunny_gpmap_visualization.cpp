#if 1

// STL
#include <string>
#include <vector>
#include <sstream>

// GPMap
#include "io/io.hpp"								// loadPointClouds, savePointClouds, loadSensorPositionList
#include "octomap/octomap.hpp"				// Octomap

using namespace GPMap;

int main(int argc, char** argv)
{
	// [0] setting - directory
	const std::string strInputDataFolder ("../../data/input/bunny/");
	const std::string strOutputDataFolder("../../data/output/bunny/");

	// [0] setting - observations
	const size_t NUM_OBSERVATIONS = 4; 
	const std::string strObsFileNames_[]	= {"bun000", "bun090", "bun180", "bun270"};
	StringList strObsFileNames(strObsFileNames_, strObsFileNames_ + NUM_OBSERVATIONS); 

	// [0] setting - GPMaps
	const size_t NUM_GPMAPS = 8; 
	const std::string strGPMapFileNames_[]	= {"gpmap_meanZero_covSEiso_batch_BCM_der", 
															"gpmap_meanZero_covSEiso_batch_BCM_func",
															"gpmap_meanZero_covSEiso_batch_iBCM_der", 
															"gpmap_meanZero_covSEiso_batch_iBCM_func",
															"gpmap_meanZero_covSEiso_incremental_BCM_der", 
															"gpmap_meanZero_covSEiso_incremental_BCM_func",
															"gpmap_meanZero_covSEiso_incremental_iBCM_der",
															"gpmap_meanZero_covSEiso_incremental_iBCM_func"};
	StringList strGPMapFileNames(strGPMapFileNames_, strGPMapFileNames_ + NUM_GPMAPS); 

	// [1] load/save hit points
	PointXYZCloudPtrList hitPointCloudPtrList;
	//loadPointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strInputDataFolder, ".ply");			// original ply files which are transformed in global coordinates
	//savePointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	loadPointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strObsFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointXYZ>("Hit Points", hitPointCloudPtrList);

	// [2] load sensor positions
	PointXYZVList sensorPositionList;
	loadSensorPositionList(sensorPositionList, strObsFileNames, strInputDataFolder, "_camera_position.txt");
	//assert(NUM_OBSERVATIONS == hitPointCloudPtrList.size() && NUM_OBSERVATIONS == sensorPositionList.size());

	// [3] Octomaps

	// log file
	LogFile logFile(strOutputDataFolder + "octomap.log");

	// setting
	const double	RESOLUTION = 0.001;
	Octomap<NO_COLOR> octomap(RESOLUTION);

	// update
	CPU_Times octomap_elapsed, octomap_total_elapsed;
	octomap_total_elapsed.clear();
	for(size_t i = 0; i < hitPointCloudPtrList.size(); i++)
	{
		logFile << "==== Updating the Octomap with the point cloud #" << i << " ====" << std::endl;

		// update
		octomap_elapsed = octomap.update<pcl::PointXYZ, pcl::PointXYZ>(*(hitPointCloudPtrList[i]), sensorPositionList[i]);

		// save
		std::stringstream ss;
		ss << strOutputDataFolder << "octomap_bunny_upto_" << i;
		octomap.save(ss.str());

		// accumulate cpu times
		octomap_total_elapsed += octomap_elapsed;
		logFile << octomap_elapsed << std::endl << std::endl;
	}

	// total time
	logFile << "============= Total Time =============" << std::endl;
	logFile << octomap_total_elapsed << std::endl << std::endl;

	// [4] evaluation
	logFile << "============= Evaluation =============" << std::endl;
	unsigned int num_points, num_voxels_correct, num_voxels_wrong, num_voxels_unknown;
	octomap.evaluate<pcl::PointXYZ, pcl::PointXYZ>(hitPointCloudPtrList, sensorPositionList,
																  num_points, num_voxels_correct, num_voxels_wrong, num_voxels_unknown);
	logFile << "Number of hit points: " << num_points << std::endl;
	logFile << "Number of correct voxels: " << num_voxels_correct << std::endl;
	logFile << "Number of wrong voxels: " << num_voxels_wrong << std::endl;
	logFile << "Number of unknown voxels: " << num_voxels_unknown << std::endl;
	logFile << "Correct rate (correct/(correct+wrong)): " << static_cast<float>(num_voxels_correct)/static_cast<float>(num_voxels_correct+num_voxels_wrong) << std::endl;

	// [3] Convert GPMaps to Octomaps
	for(size_t i = 0; i < strGPMapFileNames.size(); i++)
	{
		std::cout << "================[ " << strGPMapFileNames[i] << " ]================" << std::endl;

		// log file
		std::string strLogFilePath = strOutputDataFolder + strGPMapFileNames[i] + "_visualization.log";
		LogFile logFile;
		logFile.open(strLogFilePath);

		// setting - resolution and GPMap
		pcl::PointCloud<pcl::PointNormal>::Ptr pPointCloudGPMap;
		loadPointCloud<pcl::PointNormal>(pPointCloudGPMap, strOutputDataFolder + strGPMapFileNames[i] + ".pcd");

		// [3-1] train PLSC parameters
		//Octomap<NO_COLOR> octomap_train(RESOLUTION);
		//octomap_train.GPMap2Octomap(*pPointCloudGPMap);
		//float PLSC_mean = 0.05f;
		//float PLSC_var  = 0.0001f;
		//const float sumNegLOO = octomap_train.train(pPointCloudGPMap, PLSC_mean, PLSC_var, true, 1000);
		//std::cout << "PLSC_mean = " << expf(PLSC_mean) << std::endl;
		//std::cout << "PLSC_var = "  << expf(PLSC_var)  << std::endl;
		//std::cout << "sumNegLOO = "  << sumNegLOO  << std::endl;
		// sum_neg_log_occupied only:						PLSC_mean = 1.292437, PLSC_var = 1.000087, sumNegLOO = 2.286207
		// sum_neg_log_occupied + sum_neg_log_empty: PLSC_mean = 2.64143,  PLSC_var = 1,			 sumNegLOO = 1.19209e-7
		PLSC::mean = 2.64143f;
		PLSC::var = 1.f;

		// [3-1] OcTree
		Octomap<NO_COLOR> octomap(RESOLUTION);
		octomap.GPMap2Octomap(*pPointCloudGPMap);
		octomap.save(strOutputDataFolder + strGPMapFileNames[i]);

		// [3-2] OcTree - evaluation
		unsigned int num_points, num_voxels_correct, num_voxels_wrong, num_voxels_unknown;
		octomap.evaluate<pcl::PointXYZ, pcl::PointXYZ>(hitPointCloudPtrList, sensorPositionList,
																	  num_points, num_voxels_correct, num_voxels_wrong, num_voxels_unknown);
		logFile << "Number of hit points: "			<< num_points				<< std::endl;
		logFile << "Number of correct voxels: "	<< num_voxels_correct	<< std::endl;
		logFile << "Number of wrong voxels: "		<< num_voxels_wrong		<< std::endl;
		logFile << "Number of unknown voxels: "	<< num_voxels_unknown	<< std::endl;
		logFile << "Correct rate (correct/(correct+wrong)): " << static_cast<float>(num_voxels_correct)/static_cast<float>(num_voxels_correct+num_voxels_wrong) << std::endl;

		// [3-3] ColorOcTree
		Octomap<COLOR>		color_octomap(RESOLUTION);
		color_octomap.GPMap2Octomap(*pPointCloudGPMap);
		color_octomap.save(strOutputDataFolder + strGPMapFileNames[i] + "_color");

		std::cout << std::endl << std::endl;
	}

	// [3-3] ColorOcTree - coloring with specific min/max ranges of variance
	while(true)
	{
		// thresholds for color
		float minVar; std::cout << "min var thld: "; std::cin >> minVar;
		float maxVar; std::cout << "max var thld: "; std::cin >> maxVar;
		if(minVar == maxVar) break;

		// for each GPMap
		for(size_t i = 0; i < strGPMapFileNames.size(); i++)
		{
			std::cout << "================[ " << strGPMapFileNames[i] << " ]================" << std::endl;

			// log file
			std::string strLogFilePath = strOutputDataFolder + strGPMapFileNames[i] + "_visualization_specific.log";
			LogFile logFile;
			logFile.open(strLogFilePath);

			// setting - resolution and GPMap
			const double	RESOLUTION = 0.001;
			pcl::PointCloud<pcl::PointNormal>::Ptr pPointCloudGPMap;
			loadPointCloud<pcl::PointNormal>(pPointCloudGPMap, strOutputDataFolder + strGPMapFileNames[i] + ".pcd");

			// file name
			std::stringstream ss;
			ss.precision(std::numeric_limits<float>::digits10);
			ss << strOutputDataFolder << strGPMapFileNames[i] << "_color_"
																			  << "_min_var_" << std::scientific << minVar
																			  << "_max_var_" << std::scientific << maxVar;

			// color octree
			Octomap<COLOR>		color_octomap(RESOLUTION);
			color_octomap.GPMap2Octomap(*pPointCloudGPMap, minVar, maxVar); 
			color_octomap.save(ss.str());

			std::cout << std::endl << std::endl;
		}
	}

	system("pause");
}

#endif