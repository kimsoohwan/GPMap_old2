#include "serialization/test_eigen_serialization.hpp"
#include "data/test_test_data.hpp"
#include "bcm/test_bcm.hpp"
#include "bcm/test_bcm_serialization.hpp"
//#include "octree/test_octree_gpmap.hpp"
#include "serialization/test_eigen_serialization.hpp"

int main(int argc, char** argv) 
{ 
	// Initialize test environment
	::testing::InitGoogleTest(&argc, argv);
		
	// Test
	int ret = RUN_ALL_TESTS(); 

	system("pause");
	return ret;
}