#include "data/test_test_data.hpp"
#include "bcm/test_bcm.hpp"
#include "octree/test_octree_gpmap.hpp"

int main(int argc, char** argv) 
{ 
	// Initialize test environment
	::testing::InitGoogleTest(&argc, argv);
		
	// Test
	int ret = RUN_ALL_TESTS(); 

	system("pause");
	return ret;
}