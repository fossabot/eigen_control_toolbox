#include <cstdlib>
#include <ctime>
#include <iostream>

#include <gtest/gtest.h>
#include <gtest/gtest-death-test.h>

#include <ros/ros.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_state_space_systems/eigen_state_space_systems.h>

ros::NodeHandle* nh;
constexpr unsigned int order=10; // system order
constexpr unsigned int nin=1;    // number of inputs
constexpr unsigned int nout=1;   // number of outputs

using DiscreteStateSpaceNXX = eigen_control_toolbox::DiscreteStateSpace<order,  -1,  -1>;
using DiscreteStateSpaceNNN = eigen_control_toolbox::DiscreteStateSpace<order,nin, nout>;
using DiscreteStateSpaceXXX = eigen_control_toolbox::DiscreteStateSpace<   -1,  -1,  -1>;

TEST(TestSuite, TestDiscreteStateSpaceNXX)
{

  Eigen::MatrixXd A(order,order);
  Eigen::MatrixXd B(order,nin);
  Eigen::MatrixXd C(nout,order);
  Eigen::MatrixXd D(nout,nin);
  
  A.setRandom();
  B.setRandom();
  C.setRandom();
  D.setRandom();
  
  // FIRST WAY TO DEFINE: consider the state with fixed-order, 
  // but the input and the output are defined online, 
  // according to the dimension of A,B,C and D.
  // NOTE: since it is in runtime, if the Input or the Output are of
  //       dimension 1, you have to use the "Eigen::VectorXd" as input
  //       to the function, or Eigen::VectorXd with the proper dimension

  EXPECT_NO_FATAL_FAILURE(DiscreteStateSpaceNXX ss(A,B,C,D));

  DiscreteStateSpaceNXX ss(A,B,C,D);
  Eigen::VectorXd u(nin);   //input vector
  Eigen::VectorXd y(nout);  //output vector
  
  u.setRandom();
  y.setRandom();
    
  EXPECT_TRUE(ss.setStateFromLastIO(u,y)); // initialize initial state value for dumpless startup
  
  EXPECT_NO_FATAL_FAILURE( y=ss.update(u) ); // computing one step, updating state and output
}

TEST(TestSuite, TestDiscreteStateSpaceNNN)
{
  Eigen::MatrixXd A(order,order);
  Eigen::MatrixXd B(order,nin);
  Eigen::MatrixXd C(nout,order);
  Eigen::MatrixXd D(nout,nin);
  
  A.setRandom();
  B.setRandom();
  C.setRandom();
  D.setRandom();
  
// SECOND WAY TO DEFINE: consider the state, the input and the output defined fixed 
  // NOTE: since it is templated at compile time, if the Input or the Output are of
  //       dimension 1, you have to use the "double" as input
  EXPECT_NO_FATAL_FAILURE(DiscreteStateSpaceNNN ss(A,B,C,D));

  DiscreteStateSpaceNNN ss(A,B,C,D);
  double u;   //input vector
  double y;  //output vector

  EXPECT_TRUE(ss.setStateFromLastIO(u,y)); // initialize initial state value for dumpless startup
  EXPECT_NO_FATAL_FAILURE(y=ss.update(u)); // computing one step, updating state and output
}

TEST(TestSuite, TestDiscreteStateSpaceXXX)  
{
  EXPECT_NO_FATAL_FAILURE(DiscreteStateSpaceXXX ss);
  
  DiscreteStateSpaceXXX dss;
  EXPECT_FALSE(dss.importMatricesFromParam(*nh,"/non_existent_namespace"));
  EXPECT_FALSE(dss.importMatricesFromParam(*nh,"/dss"));
  EXPECT_TRUE(dss.importMatricesFromParam(*nh,"dss"));

  EXPECT_TRUE( 2 == dss.getOrder() );
  EXPECT_TRUE( 3 == dss.getNumberOfInputs() );
  EXPECT_TRUE( 1 == dss.getNumberOfOutputs() );
}



int main(int argc,char** argv)
{
  // ------ Init ROS ------
  ros::init(argc,&*argv,"test_filters");
  
  nh = new ros::NodeHandle("~");

  srand((unsigned int) time(0));

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 

  return 0; 
}
