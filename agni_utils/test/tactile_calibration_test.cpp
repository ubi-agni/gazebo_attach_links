
// We are testing this
#include "agni_utils/tactile_calibration.hpp"

// Gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TactileCalibrationRange, calibration_range_default)
{
	int range=4096;
	// test output range with inputs outside and inside the range
	for( int raw=-1;raw < range+1; ++raw)
	{
		double val = agni::tactileCalibration(raw, range-1);
		EXPECT_TRUE(val >= 0.0 && val <= 1.0);
	}
}

TEST(TactileCalibrationMinMax, calibration_minmax_default)
{
	int range=4096;
	// test that minimum raw gives 0 output and maximum raw gives 1.0
	EXPECT_DOUBLE_EQ(agni::tactileCalibration(0, range-1),0.0);
	EXPECT_DOUBLE_EQ(agni::tactileCalibration(range-1, range-1),1.0);
}


TEST(TactileCalibrationRange, calibration_range_ubi0)
{
	int range=1024;
	// test output range with inputs outside and inside the range
	for( int raw=-1;raw < range+1; ++raw)
	{
		double val = agni::tactileCalibration(raw, range-1,agni::UBI0);
		EXPECT_TRUE(val >= 0.0 && val <= 1.0);
	}
}

TEST(TactileCalibrationMinMax, calibration_minmax_ubi0)
{
	int range=1024;
	// test that minimum raw gives 1.0 output and maximum raw gives 0.0 (INVERTED)
	EXPECT_DOUBLE_EQ(agni::tactileCalibration(0, range-1,agni::UBI0),1.0);
	EXPECT_DOUBLE_EQ(agni::tactileCalibration(range-1, range-1,agni::UBI0),0.0);
}

TEST(TactileCalibrationRange, calibration_range_palm)
{
	int range=1024;
	// test output range with inputs outside and inside the range
	for( int raw=-1;raw < range+1; ++raw)
	{
		double val = agni::tactileCalibration(raw, range-1,agni::Palm);
		EXPECT_TRUE(val >= 0.0 && val <= 1.0);
	}
}

TEST(TactileCalibrationMinMax, calibration_minmax_palm)
{
	int range=4096;
	// test that minimum raw gives 0 output and maximum raw gives 1.0
	EXPECT_DOUBLE_EQ(agni::tactileCalibration(0, range-1,agni::Palm),0.0);
	EXPECT_DOUBLE_EQ(agni::tactileCalibration(range-1, range-1,agni::Palm),1.0);
}

TEST(TactileCalibrationRange, calibration_range_middle)
{
	int range=4096;
	// test output range with inputs outside and inside the range
	for( int raw=-1;raw < range+1; ++raw)
	{
		double val = agni::tactileCalibration(raw, range-1,agni::Middle);
		EXPECT_TRUE(val >= 0.0 && val <= 1.0);
	}
}

TEST(TactileCalibrationMinMax, calibration_minmax_middle)
{
	int range=4096;
	// test that minimum raw gives 0 output and maximum raw gives 1.0
	EXPECT_DOUBLE_EQ(agni::tactileCalibration(0, range-1,agni::Middle),0.0);
	EXPECT_DOUBLE_EQ(agni::tactileCalibration(range-1, range-1,agni::Middle),1.0);
}

TEST(TactileCalibrationRange, calibration_range_prox)
{
	int range=4096;
	// test output range with inputs outside and inside the range
	for( int raw=-1;raw < range+1; ++raw)
	{
		double val = agni::tactileCalibration(raw, range-1,agni::Prox);
		EXPECT_TRUE(val >= 0.0 && val <= 1.0);
	}
}

TEST(TactileCalibrationMinMax, calibration_minmax_prox)
{
	int range=4096;
	// test that minimum raw gives 0 output and maximum raw gives 1.0
	EXPECT_DOUBLE_EQ(agni::tactileCalibration(0, range-1,agni::Prox),0.0);
	EXPECT_DOUBLE_EQ(agni::tactileCalibration(range-1, range-1,agni::Prox),1.0);
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
	
	testing::InitGoogleTest(&argc, argv);
  
  return RUN_ALL_TESTS();
}
