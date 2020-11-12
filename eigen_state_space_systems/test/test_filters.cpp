#include <iostream>
#include <cstdlib>
#include <ctime>

#include <ros/ros.h>

#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_state_space_systems/eigen_common_filters.h>
#include <gtest/gtest.h>

ros::NodeHandle* nh;

double natural_frequency =   500; // [rad/s]
double sampling_period   = 0.001; // s

// Declare a test
TEST(TestSuite, FirstOrderLowPassX)
{
  EXPECT_NO_FATAL_FAILURE( eigen_control_toolbox::FirstOrderLowPassX lpf );
  EXPECT_NO_FATAL_FAILURE( eigen_control_toolbox::FirstOrderLowPassX lpf(natural_frequency,sampling_period, 3) );

  eigen_control_toolbox::FirstOrderLowPassX lpf;
  EXPECT_TRUE( lpf.init(natural_frequency, sampling_period, 3) );

  EXPECT_NO_FATAL_FAILURE( int order = lpf.getOrder()           );
  EXPECT_NO_FATAL_FAILURE( int nin   = lpf.getNumberOfInputs()  );
  EXPECT_NO_FATAL_FAILURE( int nout  = lpf.getNumberOfOutputs() );

  int ch;
  EXPECT_NO_FATAL_FAILURE( ch = lpf.getChannels()        );
  Eigen::VectorXd u(ch); u.setZero();
  Eigen::VectorXd y(ch); y.setZero();

  EXPECT_TRUE( lpf.setStateFromLastIO(u,  y) );
  EXPECT_TRUE( eigen_utils::norm(lpf.getState()) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.getState()));  
  EXPECT_TRUE( eigen_utils::norm(lpf.getInput()) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.getInput()));  
  EXPECT_TRUE( eigen_utils::norm(lpf.getOutput()) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.getOutput()));  
  EXPECT_TRUE( eigen_utils::norm(lpf.getOutput() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.getOutput() - u));

  u(0) = 1.0;
  for (unsigned int i=0;i<100000;i++)
  {
    EXPECT_NO_FATAL_FAILURE( y=lpf.update(u) );
  }
  EXPECT_TRUE( eigen_utils::norm(lpf.getOutput() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.getOutput() - u));
}


TEST(TestSuite, FirstOrderLowPass3)
{
  EXPECT_NO_FATAL_FAILURE( eigen_control_toolbox::FirstOrderLowPass<3> lpf(natural_frequency,sampling_period) );

  eigen_control_toolbox::FirstOrderLowPass<3> lpf(natural_frequency,sampling_period);
  EXPECT_NO_FATAL_FAILURE( int order = lpf.getOrder()           );
  EXPECT_NO_FATAL_FAILURE( int nin   = lpf.getNumberOfInputs()  );
  EXPECT_NO_FATAL_FAILURE( int nout  = lpf.getNumberOfOutputs() );

  int ch;
  EXPECT_NO_FATAL_FAILURE( ch = lpf.getChannels()        );
  Eigen::VectorXd u(ch); u.setZero();
  Eigen::VectorXd y(ch); y.setZero();

  EXPECT_TRUE( lpf.setStateFromLastIO(u,  y) );
  EXPECT_TRUE( eigen_utils::norm(lpf.getOutput() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.getOutput() - u));
  u(0) = 1.0;
  for (unsigned int i=0;i<100000;i++)
  {
    EXPECT_NO_FATAL_FAILURE( y=lpf.update(u) );
  }
  EXPECT_TRUE( eigen_utils::norm(lpf.getOutput() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.getOutput() - u));
}

TEST(TestSuite, FirstOrderLowPass1)
{
  EXPECT_NO_FATAL_FAILURE( eigen_control_toolbox::FirstOrderLowPass<1> lpf(natural_frequency,sampling_period) );

  eigen_control_toolbox::FirstOrderLowPass<1> lpf(natural_frequency,sampling_period);
  EXPECT_NO_FATAL_FAILURE( int order = lpf.getOrder()           );
  EXPECT_NO_FATAL_FAILURE( int nin   = lpf.getNumberOfInputs()  );
  EXPECT_NO_FATAL_FAILURE( int nout  = lpf.getNumberOfOutputs() );

  int ch;
  EXPECT_NO_FATAL_FAILURE( ch = lpf.getChannels()        );
  double u = 0;
  double y = 0;

  EXPECT_TRUE( lpf.setStateFromLastIO(u,  y) );
  EXPECT_TRUE( eigen_utils::norm(lpf.getOutput() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.getOutput() - u));
  u = 1.0;
  for (unsigned int i=0;i<100000;i++)
  {
    EXPECT_NO_FATAL_FAILURE( y=lpf.update(u) );
  }
  EXPECT_TRUE( eigen_utils::norm(lpf.getOutput() - u) < 1e-12 ) << "get: " + std::to_string(eigen_utils::norm(lpf.getOutput() - u));
}


int main(int argc,char** argv)
{
  // ------ Init ROS ------
  ros::init(argc,&*argv,"test_filters");
  
  nh = new ros::NodeHandle();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 

  return 0; 
}
