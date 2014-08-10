#if 0

#define _TEST_OCTREE_GPMAP

// GPMap
#include "io/io.hpp"								// loadPointClouds, savePointClouds, loadSensorPositionList
#include "visualization/cloud_viewer.hpp"	// show
#include "data/training_data.hpp"			// genEmptyPointList
#include "features/surface_normal.hpp"		// estimateSurfaceNormals
#include "common/common.hpp"					// getMinMaxPointXYZ
#include "octree/octree_gpmap.hpp"			// OctreeGPMap
#include "octree/octree_viewer.hpp"			// OctreeViewer
using namespace GPMap;

int main(int argc, char**argv)
{
		// [1] load sensor positions
		pcl::PointXYZ sensorPosition(7.2201273e-01, 2.5926464e-02, 1.6074278e-01);

		// [2] load surface normals
		PointNormalCloudPtr pPointNormalCloud(new PointNormalCloud());
		pcl::io::loadPCDFile<pcl::PointNormal>("../../data/input/bunny/bun000_normals.pcd", *pPointNormalCloud);

		// [3] bounding box
		pcl::PointXYZ min_pt, max_pt;
		getMinMaxPointXYZ<pcl::PointNormal>(*pPointNormalCloud, min_pt, max_pt);

		// [4] cropped point cloud list of four parts
		const size_t NUM_DATA = 4; 
		PointNormalCloudPtrList pointNormalCloudList(NUM_DATA);
		pcl::PointXYZ mid_pt((min_pt.x+max_pt.x)/2.f, 
									(min_pt.y+max_pt.y)/2.f,
									(min_pt.z+max_pt.z)/2.f);

		Eigen::Vector4f min_pt_temp[NUM_DATA], max_pt_temp[NUM_DATA];
		min_pt_temp[0].x() = min_pt.x;	min_pt_temp[0].y() = mid_pt.y;	min_pt_temp[0].z() = mid_pt.z;
		max_pt_temp[0].x() = max_pt.x;	max_pt_temp[0].y() = max_pt.y;	max_pt_temp[0].z() = max_pt.z;

		min_pt_temp[1].x() = min_pt.x;	min_pt_temp[1].y() = min_pt.y;	min_pt_temp[1].z() = mid_pt.z;
		max_pt_temp[1].x() = max_pt.x;	max_pt_temp[1].y() = mid_pt.y;	max_pt_temp[1].z() = max_pt.z;

		min_pt_temp[2].x() = min_pt.x;	min_pt_temp[2].y() = min_pt.y;	min_pt_temp[2].z() = min_pt.z;
		max_pt_temp[2].x() = max_pt.x;	max_pt_temp[2].y() = mid_pt.y;	max_pt_temp[2].z() = mid_pt.z;

		min_pt_temp[3].x() = min_pt.x;	min_pt_temp[3].y() = mid_pt.y;	min_pt_temp[3].z() = min_pt.z;
		max_pt_temp[3].x() = max_pt.x;	max_pt_temp[3].y() = max_pt.y;	max_pt_temp[3].z() = mid_pt.z;


		// crop
		size_t nTotalPoints(0);
		for(size_t i = 0; i < NUM_DATA; i++)
		{
			pointNormalCloudList[i] = cropBox<pcl::PointNormal>(pPointNormalCloud, min_pt_temp[i], max_pt_temp[i]);
			nTotalPoints += pointNormalCloudList[i]->size();
		}
		assert(pPointNormalCloud->size() == nTotalPoints);
		//show<pcl::PointNormal>("Cropped Bunny000", pointNormalCloudList, 0.01);		
		
		// [5] octree-based GPMap
		const double		BLOCK_SIZE(0.01);
		const size_t		NUM_CELLS_PER_AXIS(10);
		const bool			INDEPENDENT_BCM(true); //INDEPENDENT_BCM(false)
		const bool			POINT_DUPLICATION(false);
		const float			GAP(0.001);
		OctreeGPMap<pcl::PointNormal> gpmap(BLOCK_SIZE, NUM_CELLS_PER_AXIS, INDEPENDENT_BCM, POINT_DUPLICATION);
		gpmap.defineBoundingBox(min_pt, max_pt);

		// [7] observations
		for(size_t i = 0; i < NUM_DATA; i++)
		{
			std::cout << "observation: " << i << std::endl;
			gpmap.setInputCloud(pointNormalCloudList[i], GAP, sensorPosition);
			gpmap.addPointsFromInputCloud();
			gpmap.update();
		}

		system("pause");

	return 0;
}

#endif