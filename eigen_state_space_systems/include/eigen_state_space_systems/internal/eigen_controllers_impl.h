#ifndef eigen_controller_impl_201811280951
#define eigen_controller_impl_201811280951

#include <ros/console.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <eigen_state_space_systems/eigen_controllers.h>

namespace eigen_control_toolbox
{

template< int S, int I, int O, int MS, int MI, int MO >
inline Controller<S,I,O,MS,MI,MO>::Controller(const Controller<S,I,O,MS,MI,MO>::MatrixA& A, 
                                              const Controller<S,I,O,MS,MI,MO>::MatrixB& B, 
                                              const Controller<S,I,O,MS,MI,MO>::MatrixC& C, 
                                              const Controller<S,I,O,MS,MI,MO>::MatrixD& D)
{
  Controller<S,I,O,MS,MI,MO>::setMatrices(A,B,C,D);
}

template< int S, int I, int O, int MS, int MI, int MO >
inline void Controller<S,I,O,MS,MI,MO>::setMatrices(const Controller<S,I,O,MS,MI,MO>::MatrixA& A, 
                                                    const Controller<S,I,O,MS,MI,MO>::MatrixB& B, 
                                                    const Controller<S,I,O,MS,MI,MO>::MatrixC& C, 
                                                    const Controller<S,I,O,MS,MI,MO>::MatrixD& D)
{
  eigen_control_toolbox::Controller<S,I,O,MS,MI,MO>::setMatrices(A, B, C, D);
  
  m_Baw.resize(this->getOrder(),this->getNumberOfOutputs());
  m_Baw.setZero();
}

template< int S, int I, int O, int MS, int MI, int MO >
inline Controller<S,I,O,MS,MI,MO>::Controller()
{

}

template< int S, int I, int O, int MS, int MI, int MO >
inline void Controller<S,I,O,MS,MI,MO>::setAntiWindupMatrix(const Eigen::Ref< Eigen::MatrixXd > Baw)
{
  assert(Baw.rows()==this->getOrder());
  assert(Baw.cols()==this->getNumberOfOutputs());
  
  m_Baw=Baw;
}

template< int S, int I, int O, int MS, int MI, int MO >
inline bool Controller<S,I,O,MS,MI,MO>::importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name)
{
  std::string type;
  if (nh.hasParam(name+"/type"))
  {
    if (!nh.getParam(name+"/type",type))
    {
      ROS_ERROR("%s/type is not a string",name.c_str());
      return false;
    }
  }
  else
  {
    ROS_DEBUG("%s/type is not defined, using state-space",name.c_str());
    type="state-space";
  }

  ROS_DEBUG("loading controller %s of type %s",name.c_str(),type.c_str());
  if (!type.compare("proportional"))
  {
    return importProportionalFromParam(nh,name);
  }
  else if (!type.compare("PI"))
  {
    return importPIFromParam(nh,name);
  }
  else if (!type.compare("state-space"))
  {

    Controller<S,I,O,MS,MI,MO>::importMatricesFromParam(nh,name);
    Eigen::MatrixXd aw_gain;   //antiwindup_gain
    std::vector<double> aw_states; //antiwindup_gain

    if (!eigen_utils::getParam(nh, name+"/antiwindup_gain", aw_gain))
    {
      ROS_DEBUG("[Controller] cannot find '%s/antiwindup_gain' parameter!. SET NULL!!!!!",name.c_str());
      aw_gain.resize(this->getNumberOfInputs(),this->getNumberOfOutputs());
      aw_gain.setZero();
    }
    if ( (aw_gain.cols()!=this->getNumberOfOutputs()) || (aw_gain.rows()!=this->getNumberOfInputs()))
    {
      ROS_WARN("antiwindup_gain size is wrong ([%zu,%zu] instead of [%u,%u]",aw_gain.rows(),aw_gain.cols(),this->getNumberOfInputs(),this->getNumberOfOutputs());
      return false;
    }

    if (!rosparam_utilities::getParamVector(nh, name+"/antiwindup_states", aw_states))
    {
      ROS_DEBUG("[Controller] cannot find '%s/antiwindup_states' parameter!",name.c_str());
      aw_states.resize(this->getOrder(),0);
    }
    if (aw_states.size()!=this->getOrder())
    {
      ROS_WARN("antiwindup_states size is wrong (%zu instead of %u",aw_states.size(),this->getOrder());
      return false;
    }

    Eigen::MatrixXd Baw=this->m_B*aw_gain;
    // Baw norder x nout
    // B  norder x nin
    // aw_gain nin x nout
    // Baw=m_B*aw_gain  ===>  (norder x nout) = (norder x nin) x ( nin x nout)


    for (unsigned int iord=0;iord<this->getOrder();iord++)
    {
      if (!aw_states.at(iord))
        Baw.row(iord).setZero();
    }

    setAntiWindupMatrix(Baw);
  }
  else if (!type.compare("none"))
  {
    setPI(0,0,0);
  }
  else
  {
    ROS_ERROR("controller type %s does not exist. \nAvailable ones are:\n - proportional,\n - PI,\n - state-space,\n - none",type.c_str());
	return false;
  }
  return true;
}


template< int S, int I, int O, int MS, int MI, int MO >
inline void Controller<S,I,O,MS,MI,MO>::antiwindup(const Controller<S,I,O,MS,MI,MO>::Output& saturated_output, 
                                                   const Controller<S,I,O,MS,MI,MO>::Output& unsaturated_output)
{
  Eigen::VectorXd aw=saturated_output-unsaturated_output;
  this->m_state+=this->m_Baw*aw;
}

template< int S, int I, int O, int MS, int MI, int MO >
inline void Controller<S,I,O,MS,MI,MO>::antiwindup(const double& saturated_output, const double& unsaturated_output)
{
  assert(this->getNumberOfOutputs()==1);
  Eigen::VectorXd aw(1);
  aw(0)=saturated_output-unsaturated_output;
  this->m_state+=this->m_Baw*aw;
}

template< int S, int I, int O, int MS, int MI, int MO >
inline double Controller<S,I,O,MS,MI,MO>::update(const double& input)
{
  assert(this->getNumberOfInputs()==1);
  Eigen::VectorXd u(1);
  u(0)=input;
  assert(this->getNumberOfOutputs()==1);
  this->m_output=DiscreteStateSpace<S,I,O,MS,MI,MO>::update(u);
  return this->m_output(0);
  
}

template< int S, int I, int O, int MS, int MI, int MO >
inline void Controller<S,I,O,MS,MI,MO>::setPI(const double &Kp, const double &Ki, const double& sampling_period)
{
  this->m_A.resize(1,1);
  this->m_B.resize(1,1);
  this->m_C.resize(1,1);
  this->m_D.resize(1,1);
  this->m_Baw.resize(1,1);

  if (Ki==0)
  {
    this->m_A(0,0)=0.0;
    this->m_B(0,0)=0;
    this->m_Baw(0,0)=0;
    this->m_C(0,0)=0;
    this->m_D(0,0)=Kp;
  }
  else
  {
    double Ti_inv=0;
    if (Kp==0)
      Ti_inv=1;
    else
      Ti_inv=Ki/Kp; //Ki=Kp/Ti -> 1/Ti=Ki/kp

    this->m_A(0,0)=1.0;
    this->m_B(0,0)=sampling_period*Ki;
    this->m_Baw(0,0)=sampling_period*Ti_inv;
    this->m_C(0,0)=1;
    this->m_D(0,0)=Kp;
  }
  
  std::string what;
  if(!DiscreteStateSpace<S,I,O,MS,MI,MO>::setMatrices(this->m_A,this->m_B,this->m_C,this->m_D,what))
  {
    std::cerr <<__PRETTY_FUNCTION__<<":"<<__LINE__<<":" << what << std::endl;
  }
}

template< int S, int I, int O, int MS, int MI, int MO >
inline bool Controller<S,I,O,MS,MI,MO>::importProportionalFromParam(const ros::NodeHandle &nh, const std::string &name)
{
  double Kp=0;
  if (nh.hasParam(name+"/proportional_gain"))
  {
    if (!nh.getParam(name+"/proportional_gain",Kp))
    {
      ROS_ERROR("%s/proportional_gain is not a double",name.c_str());
      return false;
    }
    if (Kp<0)
    {
      ROS_INFO("%s/proportional_gain is negative, are you sure?",name.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("%s/proportional_gain is not defined",name.c_str());
    return false;
  }
  setPI(Kp,0,0);
  return true;
}

template< int S, int I, int O, int MS, int MI, int MO >
inline bool Controller<S,I,O,MS,MI,MO>::importPIFromParam(const ros::NodeHandle &nh, const std::string &name)
{
  double Kp=0;
  if (nh.hasParam(name+"/proportional_gain"))
  {
    if (!nh.getParam(name+"/proportional_gain",Kp))
    {
      ROS_ERROR("%s/proportional_gain is not a double",name.c_str());
      return false;
    }
    if (Kp<0)
    {
      ROS_INFO("%s/proportional_gain is negative, are you sure?",name.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("%s/proportional_gain is not defined",name.c_str());
    return false;
  }

  double Ki=0;
  if (nh.hasParam(name+"/integral_gain"))
  {
    if (!nh.getParam(name+"/integral_gain",Ki))
    {
      ROS_ERROR("%s/integral_gain is not a double",name.c_str());
      return false;
    }
    if (Ki<0)
    {
      ROS_INFO("%s/integral_gain is negative, are you sure?",name.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("%s/integral_gain is not defined",name.c_str());
    return false;
  }

  double sample_period;

  if (nh.hasParam(name+"/sample_period"))
  {
    if (!nh.getParam(name+"/sample_period",sample_period))
    {
      ROS_ERROR("%s/sample_period is not a double",name.c_str());
      return false;
    }
    if (sample_period<0)
    {
      ROS_ERROR("%s/sample_period should be positive",name.c_str());
      return false;
    }
  }
  else
  {
    ROS_ERROR("%s/sample_period is not defined",name.c_str());
    return false;
  }

  setPI(Kp,Ki,sample_period);
  return true;
}


}



#endif
