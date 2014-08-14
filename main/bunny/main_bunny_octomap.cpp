#if 1

// STL
#include <sstream>

// GPMap
#include "serialization/eigen_serialization.hpp"	// Eigen
#include "io/io.hpp"											// loadPointClouds, savePointClouds, loadSensorPositionList
#include "octomap/octomap.hpp"							// Octomap
#include "util/log.hpp"										// Log
using namespace GPMap;

int main(int argc, char** argv)
{
	// files
	const size_t NUM_DATA = 4; 
	const std::string strInputDataFolder ("../../data/input/bunny/");
	const std::string strOutputDataFolder("../../data/output/bunny/");
	const std::string strFilenames_[] = {"bun000", "bun090", "bun180", "bun270"};
	StringList strFileNames(strFilenames_, strFilenames_ + NUM_DATA); 

	// log
	std::string strLogFileName = strOutputDataFolder + "octomap.log";
	Log log(strLogFileName);

	// [1] load/save hit points
	PointXYZCloudPtrList hitPointCloudPtrList;
	//loadPointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strFileNames, strInputDataFolder, ".ply");					// original ply files which are transformed in global coordinates
	//savePointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	loadPointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointXYZ>("Hit Points", hitPointCloudPtrList);

	// [2] load sensor positions
	PointXYZVList sensorPositionList;
	loadSensorPositionList(sensorPositionList, strFileNames, strInputDataFolder, "_camera_position.txt");
	assert(NUM_DATA == hitPointCloudPtrList.size() && NUM_DATA == sensorPositionList.size());

	// [3] octomap
	const double OCTOMAP_RESOLUTION = 0.001;
	Octomap octomap(OCTOMAP_RESOLUTION);

	// update
	boost::timer::cpu_times octomap_elapsed, octomap_total_elapsed;
	octomap_total_elapsed.clear();
	for(size_t i = 0; i < hitPointCloudPtrList.size(); i++)
	{
		log << "==== Updating the octomap with the point cloud #" << i << " ====" << std::endl;

		// update
		octomap_elapsed = octomap.update<pcl::PointXYZ, pcl::PointXYZ>(*(hitPointCloudPtrList[i]), sensorPositionList[i]);

		// save
		std::stringstream ss;
		ss << strOutputDataFolder << "bunny_upto_" << i;
		octomap.save(ss.str());

		// accumulate cpu times
		octomap_total_elapsed += octomap_elapsed;
		log << octomap_elapsed << std::endl << std::endl;
	}

	// save
	log << "============= Total Time =============" << std::endl;
	log << octomap_total_elapsed << std::endl << std::endl;

	// [4] evaluation
	log << "============= Evaluation =============" << std::endl;
	unsigned int num_points, num_voxels_correct, num_voxels_wrong, num_voxels_unknown;
	octomap.evaluate<pcl::PointXYZ, pcl::PointXYZ>(hitPointCloudPtrList, sensorPositionList,
																  num_points, num_voxels_correct, num_voxels_wrong, num_voxels_unknown);
	log << "Number of hit points: " << num_points << std::endl;
	log << "Number of correct voxels: " << num_voxels_correct << std::endl;
	log << "Number of wrong voxels: " << num_voxels_wrong << std::endl;
	log << "Number of unknown voxels: " << num_voxels_unknown << std::endl;
	log << "Correct rate (correct/(correct+wrong)): " << static_cast<float>(num_voxels_correct)/static_cast<float>(num_voxels_correct+num_voxels_wrong) << std::endl;

	system("pause");

	return 0;
}

#endif