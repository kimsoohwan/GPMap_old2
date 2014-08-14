#if 1

// STL
#include <sstream>

// GPMap
#include "serialization/eigen_serialization.hpp"	// Eigen
#include "io/io.hpp"											// loadPointClouds, savePointClouds, loadSensorPositionList
#include "octomap/octomap.hpp"							// Octomap
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
	PointXYZCloudPtrList hitPointCloudPtrList;
	//loadPointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strFileNames, strInputDataFolder, ".ply");					// original ply files which are transformed in global coordinates
	//savePointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	loadPointClouds<pcl::PointXYZ>(hitPointCloudPtrList, strFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointXYZ>("Hit Points", hitPointCloudPtrList);

	// [2] load sensor positions
	PointXYZVList sensorPositionList;
	loadSensorPositionList(sensorPositionList, strFileNames, strInputDataFolder, "_camera_position.txt");
	assert(NUM_DATA == hitPointCloudPtrList.size() && NUM_DATA == sensorPositionList.size());

	// [3-1] octomap
	const double OCTOMAP_RESOLUTION = 0.001;
	Octomap octomap(OCTOMAP_RESOLUTION);

	// [3-2] update
	boost::timer::cpu_times octomap_elapsed, octomap_total_elapsed;
	octomap_total_elapsed.clear();
	for(size_t i = 0; i < hitPointCloudPtrList.size(); i++)
	{
		std::cout << "==== Updating the octomap with the point cloud #" << i << " ====" << std::endl;

		// update
		octomap_elapsed = octomap.update<pcl::PointXYZ, pcl::PointXYZ>(*(hitPointCloudPtrList[i]), sensorPositionList[i]);

		// save
		std::stringstream ss;
		ss << strOutputDataFolder << "bunny_upto_" << i;
		octomap.save(ss.str());

		// accumulate cpu times
		octomap_total_elapsed += octomap_elapsed;
		std::cout << std::endl << octomap_elapsed << std::endl << std::endl;
	}

	// save
	std::cout << "============= Total =============" << std::endl;
	std::cout << octomap_total_elapsed << std::endl << std::endl;

	system("pause");

	return 0;
}

#endif