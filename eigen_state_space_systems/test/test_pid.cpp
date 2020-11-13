#include <iostream>
#include <cstdlib>
#include <ctime>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_state_space_systems/filtered_values.h>
#include <eigen_state_space_systems/eigen_controllers.h>


ros::NodeHandle* nh;

// Declare a test
TEST(TestSuite, FilteredScalar)
{
  eigen_control_toolbox::Controller<-1,-1,-1> proportional;
  if (!proportional.importMatricesFromParam(*nh,"/ctrl1"))
  {
    ROS_ERROR("Failing initializing controller ctrl1");
  }
  ROS_INFO("ctrl1:");
  proportional.print();

  eigen_control_toolbox::Controller<-1,-1,-1> pi;
  if (!pi.importMatricesFromParam(*nh,"/ctrl2"))
  {
    ROS_ERROR("Failing initializing controller ctrl1");
  }

  ROS_INFO("ctrl2:");
  pi.print();

  for (unsigned int idx=0;idx<200;idx++)
  {
    double controller_input=1;
    double p_output=proportional.update(controller_input);
    ROS_INFO("controller_input=%f, proportional output=%f",controller_input,p_output);
  }

  for (unsigned int idx=0;idx<200;idx++)
  {
    double controller_input=1;
    double pi_output=pi.update(controller_input);
    double pi_sat=std::max(std::min(pi_output,2.0),-2.0);
    pi.antiwindup(pi_sat,pi_output);
    ROS_INFO("controller_input=%f, PI output=%f, PI saturated output=%f",controller_input,pi_output,pi_sat);
   }
}


int main(int argc,char** argv)
{
  // ------ Init ROS ------
  ros::init(argc,&*argv,"test_pid");
  nh = new ros::NodeHandle();

  srand((unsigned int) time(0));

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS(); 

  return 0; 
}