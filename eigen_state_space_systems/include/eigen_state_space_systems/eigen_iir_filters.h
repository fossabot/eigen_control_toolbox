#ifndef   eigen_iir_filters_201805060814
#define   eigen_iir_filters_201805060814

#include <memory>
#include <eigen_state_space_systems/eigen_state_space_systems.h>

namespace eigen_control_toolbox
{


bool importMatricesFromParam( const ros::NodeHandle& nh, 
                              const std::string& name, 
                              double& natural_frequency,
                              double& sampling_period,
                              int&    channels);
/*
 *     FirstOrderLowPass( const double& natural_frequency,
 *                        const double& sampling_period);
 */
template<int N, int MaxN = N>
class FirstOrderLowPass: public DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN>
{
protected:
  bool   computeMatrices();
  double m_natural_frequency;
  int    m_channels;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef std::shared_ptr<FirstOrderLowPass<N,MaxN>> Ptr;
  typedef std::shared_ptr<FirstOrderLowPass<N,MaxN> const> ConstPtr;

  FirstOrderLowPass();
  virtual ~FirstOrderLowPass() = default;
  FirstOrderLowPass(const double& natural_frequency, const double& sampling_period, const int& channels = N);

  virtual bool init(const double& natural_frequency, const double& sampling_period, const int& channels = N);

  [[deprecated("Use the Ctor, o the function 'init'. The dependency from ROS will be removed in the future")]]
  virtual bool importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name);

  double getNaturalFrequency(){return m_natural_frequency;};
};

//!
template<int N, int MaxN> using FirstOrderLowPassPtr = typename FirstOrderLowPass<N, MaxN>::Ptr;
template<int N, int MaxN> using FirstOrderLowPassConstPtr = typename FirstOrderLowPass<N, MaxN>::ConstPtr;

//! 
typedef FirstOrderLowPass<-1> FirstOrderLowPassX;
typedef FirstOrderLowPass<-1>::Ptr FirstOrderLowPassXPtr;
typedef FirstOrderLowPass<-1>::ConstPtr FirstOrderLowPassXConstPtr;



/**
 * template
 */
template<int N, int MaxN = N>
class FirstOrderHighPass: public DiscreteStateSpace<N,N,N,MaxN,MaxN,MaxN>
{
protected:
  bool   computeMatrices();
  double m_natural_frequency;
  int    m_channels;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FirstOrderHighPass();
  ~FirstOrderHighPass() = default;
  
  FirstOrderHighPass(const double& natural_frequency, const double& sampling_period, const int& channels = N);
  
  virtual bool init(const double& natural_frequency, const double& sampling_period, const int& channels = N);

  [[deprecated("Use the Ctor, o the function 'init'. The dependency from ROS will be removed in the future")]]
  virtual bool importMatricesFromParam(const ros::NodeHandle& nh, const std::string& name);

  double getNaturalFrequency(){return m_natural_frequency;};
}; 


//!
template<int N, int MaxN> using FirstOrderHighPassPtr = typename FirstOrderHighPass<N, MaxN>::Ptr;
template<int N, int MaxN> using FirstOrderHighPassConstPtr = typename FirstOrderHighPass<N, MaxN>::ConstPtr;

//! 
typedef FirstOrderLowPass<-1> FirstOrderHighPassX;
typedef FirstOrderLowPass<-1>::Ptr FirstOrderHighPassXPtr;
typedef FirstOrderLowPass<-1>::ConstPtr FirstOrderHighPassXConstPtr;

}


#include <eigen_state_space_systems/internal/eigen_iir_filters_impl.h>
#endif
