#include <ros/ros.h>
#include <Eigen/Dense>
#include <eigen_state_space_systems/eigen_state_space_systems.h>
#include <ros/console.h>

int main(int argc,char** argv)
{
  // ------ Init ROS ------
  ros::init(argc,&*argv,"test_matrix");
  ros::NodeHandle nh;
  srand((unsigned int) time(0));
  
  const unsigned int order=10; // system order
  const unsigned int nin=1;    // number of inputs
  const unsigned int nout=1;   // number of outputs
  
  Eigen::MatrixXd A(order,order);
  Eigen::MatrixXd B(order,nin);
  Eigen::MatrixXd C(nout,order);
  Eigen::MatrixXd D(nout,nin);
  
  A.setRandom();
  B.setRandom();
  C.setRandom();
  D.setRandom();
  
  { // FIRST WAY TO DEFINE: consider the state with fixed-order, 
    // but the input and the output are defined online, 
    // according to the dimension of A,B,C and D.
    // NOTE: since it is in runtime, if the Input or the Output are of
    //       dimension 1, you have to use the "Eigen::VectorXd" as input
    //       to the function, or Eigen::VectorXd with the proper dimension
    eigen_control_toolbox::DiscreteStateSpace<order,-1,-1> ss(A,B,C,D);

    Eigen::VectorXd u(nin);   //input vector
    Eigen::VectorXd y(nout);  //output vector
    
    u.setRandom();
    y.setRandom();
    
    ss.setStateFromLastIO(u,y); // initialize initial state value for dumpless startup
    ROS_INFO_STREAM("state:\n"<<ss.getState());
    ROS_INFO_STREAM("output:\n"<<ss.getOutput() << "\ndesired:\n"<<y);
  
    y=ss.update(u); // computing one step, updating state and output
  
  }

  { // SECOND WAY TO DEFINE: consider the state, the input and the output defined fixed 
    // NOTE: since it is templated at compile time, if the Input or the Output are of
    //       dimension 1, you have to use the "double" as input
    eigen_control_toolbox::DiscreteStateSpace<order,nin, nout, 20,30,40> ss(A,B,C,D);

    double u;   //input vector
    double y;  //output vector

    ss.setStateFromLastIO(u,y); // initialize initial state value for dumpless startup
    ROS_INFO_STREAM("state:\n"<<ss.getState());
    ROS_INFO_STREAM("output:\n"<<ss.getOutput() << "\ndesired:\n"<<y);
  
    y=ss.update(u); // computing one step, updating state and output
  }
  
  eigen_control_toolbox::DiscreteStateSpaceX ss2;
  if(!ss2.importMatricesFromParam(nh,"ss"))
  {
    ROS_ERROR("error");
    return -1;
  }
  return 0; 
}
