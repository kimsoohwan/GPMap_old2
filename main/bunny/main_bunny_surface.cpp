#if 1

// STL
#include <string>
#include <vector>
#include <sstream>

// GPMap
#include "io/io.hpp"								// loadPointCloud, savePointCloud, loadSensorPositionList
#include "iso_surface/iso_surface.hpp"		// IsoSurfaceExtraction
using namespace GPMap;

int main(int argc, char** argv)
{
	// [0] setting - directory
	//const std::string strMetaDataFolder		("../../data/bunny/output/gpmap/meta_data/");
	//const std::string strOutputDataFolder	("../../data/bunny/output/gpmap/surface/");
	const std::string strMetaDataFolder		("../../data/bunny/output/gpmap_outside_neg_distance_block_size_0.003_cell_size_0.001/meta_data/");
	const std::string strOutputDataFolder	("../../data/bunny/output/gpmap_outside_neg_distance_block_size_0.003_cell_size_0.001/surface/");
	const std::string strOutputLogFolder	(strOutputDataFolder + "log/");
	create_directory(strOutputDataFolder);
	create_directory(strOutputLogFolder);

	// [0] setting - GPMaps
	const size_t NUM_GPMAPS = 9; 
	const std::string strGPMapFileNames_[]	= {
															//"gpmap_function_observations_batch", 
															//"gpmap_function_observations_iBCM_upto_0",
															//"gpmap_function_observations_iBCM_upto_1",
															//"gpmap_function_observations_iBCM_upto_2",
															//"gpmap_function_observations_iBCM_upto_3",
															//"gpmap_function_observations_BCM_upto_0",
															//"gpmap_function_observations_BCM_upto_1",
															//"gpmap_function_observations_BCM_upto_2",
															//"gpmap_function_observations_BCM_upto_3",
															"gpmap_derivative_observations_batch", 
															"gpmap_derivative_observations_iBCM_upto_0",
															"gpmap_derivative_observations_iBCM_upto_1",
															"gpmap_derivative_observations_iBCM_upto_2",
															"gpmap_derivative_observations_iBCM_upto_3",
															"gpmap_derivative_observations_BCM_upto_0",
															"gpmap_derivative_observations_BCM_upto_1",
															"gpmap_derivative_observations_BCM_upto_2",
															"gpmap_derivative_observations_BCM_upto_3",
															//"gpmap_all_observations_batch", 
															//"gpmap_all_observations_iBCM_upto_0",
															//"gpmap_all_observations_iBCM_upto_1",
															//"gpmap_all_observations_iBCM_upto_2",
															//"gpmap_all_observations_iBCM_upto_3",
															//"gpmap_all_observations_BCM_upto_0",
															//"gpmap_all_observations_BCM_upto_1",
															//"gpmap_all_observations_BCM_upto_2",
															//"gpmap_all_observations_BCM_upto_3",
															};
	StringList strGPMapFileNames(strGPMapFileNames_, strGPMapFileNames_ + NUM_GPMAPS); 

	// surface reconstruction
	const float	RESOLUTION = 0.001f;

	bool fMarchingCubes;
	std::cout << "Marching Cubes(0) or Marching Tetrahedron(1)? ";
	std::cin >> fMarchingCubes;
	std::cout << fMarchingCubes << std::endl;
	fMarchingCubes = !fMarchingCubes;

	std::string strMethodName;
	if(fMarchingCubes)	strMethodName = "_marching_cubes_";
	else						strMethodName = "_marching_etrahedrons_";

	// [1] Convert GPMaps to Surface
	for(size_t i = 0; i < strGPMapFileNames.size(); i++)
	{
		std::cout << "================[ " << strGPMapFileNames[i] << " ]================" << std::endl;

		// log file
		std::string	strLogFilePath = strOutputLogFolder + strGPMapFileNames[i] + "_surface.log";
		LogFile logFile(strLogFilePath);

		// load GPMap
		pcl::PointCloud<pcl::PointNormal>::Ptr pPointCloudGPMap;
		loadPointCloud<pcl::PointNormal>(pPointCloudGPMap, strMetaDataFolder + strGPMapFileNames[i] + ".pcd");

		// iso-surface
		IsoSurfaceExtraction isoSurface(RESOLUTION);

		// convert GPMap to surface
		isoSurface.insertMeanVarFromGPMap(*pPointCloudGPMap);

		// zero-value iso-surface
		if(fMarchingCubes)		isoSurface.marchingcubes();			// marching cubes
		else							isoSurface.marchingTetrahedron();	// marching tetrahedron

		// min max
		const float minX = isoSurface.getMinVertexValueForColor(IsoSurfaceExtraction::COLOR_BY_X);
		const float maxX = isoSurface.getMaxVertexValueForColor(IsoSurfaceExtraction::COLOR_BY_X);
		const float minY = isoSurface.getMinVertexValueForColor(IsoSurfaceExtraction::COLOR_BY_Y);
		const float maxY = isoSurface.getMaxVertexValueForColor(IsoSurfaceExtraction::COLOR_BY_Y);
		const float minZ = isoSurface.getMinVertexValueForColor(IsoSurfaceExtraction::COLOR_BY_Z);
		const float maxZ = isoSurface.getMaxVertexValueForColor(IsoSurfaceExtraction::COLOR_BY_Z);
		const float minV = isoSurface.getMinVertexValueForColor(IsoSurfaceExtraction::COLOR_BY_V);
		const float maxV = isoSurface.getMaxVertexValueForColor(IsoSurfaceExtraction::COLOR_BY_V);

		logFile << "X: min = " << minX << ", max = " << maxX << std::endl;
		logFile << "Y: min = " << minY << ", max = " << maxY << std::endl;
		logFile << "Z: min = " << minZ << ", max = " << maxZ << std::endl;
		logFile << "V: min = " << minV << ", max = " << maxV << std::endl;
		logFile << std::endl;

		// color by Z
		{
			std::stringstream ss;
			ss << strOutputDataFolder << strGPMapFileNames[i] << strMethodName << "_min_z_" << minZ << "_max_z_" << maxZ << ".ply";
			isoSurface.setVertexColors(IsoSurfaceExtraction::COLOR_BY_Z, minZ, maxZ);
			isoSurface.saveAsPLY(ss.str());
		}

		// color by V
		{
			std::stringstream ss;
			ss << strOutputDataFolder << strGPMapFileNames[i] << strMethodName << "_min_v_" << minV << "_max_v_" << maxV << ".ply";
			isoSurface.setVertexColors(IsoSurfaceExtraction::COLOR_BY_V, minV, maxV);
			isoSurface.saveAsPLY(ss.str());
		}
	}

	// particular range
	while(true)
	{
		// color mode
		char  cColorMode;			std::cout << "color mode (q for quit): "; std::cin >> cColorMode;
		if(cColorMode == 'q' || cColorMode == 'Q') break;

		// color map
		float minValueForColor; std::cout << "min value for color map: "; std::cin >> minValueForColor;
		float maxValueForColor; std::cout << "max value for color map: "; std::cin >> maxValueForColor;

		for(size_t i = 0; i < strGPMapFileNames.size(); i++)
		{
			std::cout << "================[ " << strGPMapFileNames[i] << " ]================" << std::endl;

			// log file
			std::string	strLogFilePath = strOutputLogFolder + strGPMapFileNames[i] + "_surface.log";
			LogFile logFile(strLogFilePath);

			// load GPMap
			pcl::PointCloud<pcl::PointNormal>::Ptr pPointCloudGPMap;
			loadPointCloud<pcl::PointNormal>(pPointCloudGPMap, strMetaDataFolder + strGPMapFileNames[i] + ".pcd");

			// iso-surface
			IsoSurfaceExtraction isoSurface(RESOLUTION);

			// convert GPMap to surface
			isoSurface.insertMeanVarFromGPMap(*pPointCloudGPMap);

			// zero-value iso-surface
			if(fMarchingCubes)		isoSurface.marchingcubes();			// marching cubes
			else							isoSurface.marchingTetrahedron();	// marching tetrahedron

			// set color
			std::stringstream ss;
			switch(cColorMode)
			{
			case 'x':
			case 'X': 
				ss << strOutputDataFolder << strGPMapFileNames[i] << strMethodName << "_min_x_" << minValueForColor << "_max_x_" << maxValueForColor << ".ply";
				isoSurface.setVertexColors(IsoSurfaceExtraction::COLOR_BY_X);
				break;
			case 'y':
			case 'Y': 
				ss << strOutputDataFolder << strGPMapFileNames[i] << strMethodName << "_min_y_" << minValueForColor << "_max_y_" << maxValueForColor << ".ply";
				isoSurface.setVertexColors(IsoSurfaceExtraction::COLOR_BY_Y);
				break;
			case 'z':
			case 'Z': 
				ss << strOutputDataFolder << strGPMapFileNames[i] << strMethodName << "_min_z_" << minValueForColor << "_max_z_" << maxValueForColor << ".ply";
				isoSurface.setVertexColors(IsoSurfaceExtraction::COLOR_BY_Z);
				break;
			case 'v':
			case 'V': 
				ss << strOutputDataFolder << strGPMapFileNames[i] << strMethodName << "_min_v_" << minValueForColor << "_max_v_" << maxValueForColor << ".ply";
				isoSurface.setVertexColors(IsoSurfaceExtraction::COLOR_BY_V);
				break;
			}

			// save
			isoSurface.saveAsPLY(ss.str());
		}
	}

	system("pause");
}

#endif