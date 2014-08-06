// GPMap
#include "io/io.hpp"
#include "features/surface_normal.hpp"
#include "visualization/display.hpp"
#include "octree/octree_viewer.hpp"
#include "octree/octree_gp.hpp"
using namespace GPMap;

int main(int argc, char** argv)
{
	// files
	const size_t NUM_DATA = 4; 
	const std::string strInputDataFolder ("../../data/input/bunny/");
	const std::string strOutputDataFolder("../../data/output/bunny/");
	const std::string strFilenames_[] = {"bun000", "bun090", "bun180", "bun270"};
	std::vector<std::string> strFileNames(strFilenames_, strFilenames_ + NUM_DATA); 

	// [1] load/save point clouds
	//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pPointClouds;
	////loadPointClouds<pcl::PointXYZ>(pPointClouds, strFileNames, strInputDataFolder, ".ply");		// original ply files which are transformed in global coordinates
	////savePointClouds<pcl::PointXYZ>(pPointClouds, strFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//loadPointClouds<pcl::PointXYZ>(pPointClouds, strFileNames, strInputDataFolder, ".pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointXYZ>(pPointClouds);

	// [2] load sensor positions
	//std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > sensorPositions;
	//loadSensorPositions(sensorPositions, strFileNames, strInputDataFolder, "_camera_position.txt");

	// [3] load/save surface normals
	std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> pPointNormalClouds;
	////estimateSurfaceNormals<ByNearestNeighbors>(pPointClouds, sensorPositions, false, 0.01, pPointNormalClouds);
	//estimateSurfaceNormals<ByMovingLeastSquares>(pPointClouds, sensorPositions, false, 0.01, pPointNormalClouds);
	//savePointClouds<pcl::PointNormal>(pPointNormalClouds, strFileNames, strInputDataFolder, "_normals.pcd");		// original pcd files which are transformed in global coordinates
	loadPointClouds<pcl::PointNormal>(pPointNormalClouds, strFileNames, strInputDataFolder, "_normals.pcd");		// original pcd files which are transformed in global coordinates
	//show<pcl::PointNormal>(pPointNormalClouds, 0.005, true, 0.001);

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
	octree.update(pPointNormalClouds[0]);
	OctreeViewer<pcl::PointNormal> octree_viewer(pPointNormalClouds[0], octree);
	octree.update(pPointNormalClouds[1]);
	OctreeViewer<pcl::PointNormal> octree_viewer2(pPointNormalClouds[1], octree);
	//octree.predict();

	//// visualizer
	//PCLVisualizer viewer("3D Viewer");
	////viewer.setBackgroundColor(1, 1, 1);
	//viewer.setBackgroundColor(0, 0, 0);

	////for(int i = 0; i < NUM_DATA; i++)
	////{
	////	PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr pColorHandle(new PointCloudColorHandlerCustom<pcl::PointXYZ>(pPointClouds[i], colors[i](0), colors[i](1), colors[i](2)));
	////	viewer.addPointCloud<pcl::PointXYZ>(pPointClouds[i], *pColorHandle, strFileNames[i].c_str());
	////	viewer.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, strFileNames[i].c_str());
	////}

	//// setting
	//viewer.addCoordinateSystem(0.1);
	//viewer.resetCameraViewpoint(strFileNames[0].c_str());
	//viewer.spin();

	////// main loop
	////while(!viewer.wasStopped())
	////{
	////	viewer.spinOnce(100);
	////	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	////}

	//// surface normals
	//pcl::PointXYZ sensorPositions2[NUM_DATA];
	//sensorPositions2[0] = pcl::PointXYZ(0, -0.1, 1);
	//sensorPositions2[1] = pcl::PointXYZ(-1, -0.1, 0);
	//sensorPositions2[2] = pcl::PointXYZ(0, -0.1, -1);
	//sensorPositions2[3] = pcl::PointXYZ(1, -0.1, 0);

	//pcl::PointCloud<pcl::PointNormal>::Ptr pPointNormals[NUM_DATA];
	//pcl::PointCloud<Normal>::Ptr pNormals[NUM_DATA];
	//for(size_t i = 0; i < NUM_DATA; i++)
	//{
	//	std::cout << "Estimating surface normals: " << strFileNames[i] << " ... ";
	//	std::vector<int> index;
	//	//pPointNormals[i] = estimateSurfaceNormals(pPointClouds[i], 
	//	//													   sensorPositions2[i],
	//	//														false,
	//	//														0.01, 
	//	//														index);
	//	//pPointNormals[i] = estimateSurfaceNormals(pPointClouds[i], 
	//	//													   sensorPositions2[i],
	//	//														true,
	//	//														20, 
	//	//														index);
	//	pPointNormals[i] = smoothAndNormalEstimation(pPointClouds[i],
	//																0.01);
	//	flipSurfaceNormals(sensorPositions2[i], *(pPointNormals[i]));

	//	std::cout << pPointNormals[i]->size() << std::endl;

	//	pNormals[i] = estimateSurfaceNormals(pPointClouds[i], 
	//													 sensorPositions2[i],
	//													 true,
	//													 20);

	//	if(pPointNormals[i]->size())
	//	{
	//		savePLYFileASCII<pcl::PointNormal>(strFileNames[i], *(pPointNormals[i]));
	//		//savePLYFileASCII<Normal>(strFileNames[i], *(pNormals[i]));
	//	}
	//}

	//for(int i = 0; i < NUM_DATA; i++)
	//{
	//	std::cout << "Adding surface normals: " << strFileNames[i] << std::endl;;
	//	PointCloudColorHandlerGenericField<pcl::PointNormal> color(pPointNormals[i], "z");
	//	viewer.addPointCloud<pcl::PointNormal> (pPointNormals[i], color, strFileNames[i].c_str());
	//	viewer.addPointCloudNormals<pcl::PointNormal>(pPointNormals[i], 1, 0.005, (strFileNames[i] + "normals").c_str());	// normals
	//	//viewer.addPointCloudNormals<pcl::PointXYZ,Normal>(pPointClouds[i], pNormals[i], 100, 100, (strFileNames[i] + "normals").c_str());
	//	viewer.setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, strFileNames[i].c_str());
	//	viewer.setPointCloudRenderingProperties(PCL_VISUALIZER_LINE_WIDTH, 2, (strFileNames[i] + "normals").c_str());
	//}
	//viewer.spin();

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