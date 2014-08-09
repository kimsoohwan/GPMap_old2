#if 0

// GPMap
#include "io/io.hpp"								// loadPointClouds, savePointClouds, loadSensorPositionList
#include "visualization/cloud_viewer.hpp"	// show
#include "data/training_data.hpp"			// genEmptyPointList
#include "features/surface_normal.hpp"		// estimateSurfaceNormals
#include "common/common.hpp"					// getMinMax3DFromPointClouds
#include "octree/octree_gpmap.hpp"			// OctreeGPMap
#include "octree/octree_viewer.hpp"			// OctreeViewer
using namespace GPMap;

class TestOctreeGPMap : public OctreeGPMap<pcl::PointNormal>
{
}


int main(int argc, char**argv)
{


	return 0;
}

#endif