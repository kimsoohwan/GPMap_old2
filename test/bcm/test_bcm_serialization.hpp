#ifndef _TEST_BCM_SERIALIZATION_HPP_
#define _TEST_BCM_SERIALIZATION_HPP_

// Google Test
#include "gtest/gtest.h"

#include "test_bcm.hpp"

class TestBCMSerialization : public TestBCM
{
};

/** @brief Update by mean vectors and covariance matrices */
TEST_F(TestBCMSerialization, Serialization)
{
	BCM bcm1(pMean1, pCov1);
	bcm1.save("bcm.txt");
	bcm1.load();

	BCM bcm2(pMean1, pCov1);
	EXPECT_TRUE(bcm1.getSumOfWeightedMeans()->isApprox(*bcm2.getSumOfWeightedMeans()));
	EXPECT_TRUE(bcm1.getSumOfInvCovs()->isApprox(*bcm2.getSumOfInvCovs()));
}

#endif