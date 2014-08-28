#ifndef _TEST_DATA_PARTITIONING_HPP_
#define _TEST_DATA_PARTITIONING_HPP_

// Google Test
#include "gtest/gtest.h"

// GPMap
#include "octree/data_partitioning.hpp"
using namespace GPMap;

TEST(Octree, DataPartitioning)
{
	// indices 1
	const int N1 = 10;
	std::vector<int> indices1(N1);
	std::generate(indices1.begin(), indices1.end(), UniqueNonZeroInteger());
	std::cout << std::endl;
	for(int i = 0; i < N1; i++)
	{
		EXPECT_EQ(i, indices1[i]);
	}

	// indices 2
	const int N2 = 20;
	std::vector<int> indices2(N2);
	std::generate(indices2.begin(), indices2.end(), UniqueNonZeroInteger());
	std::cout << std::endl;
	for(int i = 0; i < N2; i++)
	{
		EXPECT_EQ(i, indices2[i]);
	}

	// indices
	const int N = 30;
	std::vector<int> indices(N);
	std::generate(indices.begin(), indices.end(), UniqueNonZeroInteger());

	// data partition
	const int M = 8;
	std::vector<std::vector<int> > partitionedIndices;

	// with out suffling
	if(random_data_partition(indices, M, partitionedIndices, false))
	{
		for(size_t i = 0; i < partitionedIndices.size(); i++)
		{
			std::cout << "Partition " << i << ": ";
			for(size_t j = 0; j < partitionedIndices[i].size(); j++)
			{
				std::cout << partitionedIndices[i][j] << ", ";
			}
			std::cout << std::endl;
		}
	}

	// with suffling
	if(random_data_partition(indices, M, partitionedIndices, true))
	{
		for(size_t i = 0; i < partitionedIndices.size(); i++)
		{
			std::cout << "Partition " << i << ": ";
			for(size_t j = 0; j < partitionedIndices[i].size(); j++)
			{
				std::cout << partitionedIndices[i][j] << ", ";
			}
			std::cout << std::endl;
		}
	}

}

#endif