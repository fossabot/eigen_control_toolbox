#ifndef   eigen_iir_filters_201805060814
#define   eigen_iir_filters_201805060814

#include <eigen_state_space_systems/eigen_state_space_systems.h>

namespace eigen_control_toolbox
{
  /*
  *     FirstOrderLowPass( const double& natural_frequency,
  *                        const double& sample_period);
  */
  class FirstOrderLowPass: public DiscreteStateSpace<1,1,1,1,1,1>
  {
  protected:
    void computeMatrices(const double& natural_frequency,
                    const double& sample_period);
    double m_natural_frequency;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FirstOrderLowPass();
    ~FirstOrderLowPass() = default;
    FirstOrderLowPass( const double& natural_frequency,
                       const double& sample_period);
    
    virtual bool importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name);
    double getNaturalFrequency(){return m_natural_frequency;};
    
  };
  
  class FirstOrderHighPass: public DiscreteStateSpace<1,1,1,1,1,1>
  {
  protected:
    void computeMatrices(const double& natural_frequency,
                         const double& sample_period);
    double m_natural_frequency;
  
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FirstOrderHighPass();
    ~FirstOrderHighPass() = default;
    
    FirstOrderHighPass( const double& natural_frequency,
                       const double& sample_period);
    
    virtual bool importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name);
    double getNaturalFrequency(){return m_natural_frequency;};
  };
  
}

#include <eigen_state_space_systems/internal/eigen_iir_filters_impl.h>
#endif
