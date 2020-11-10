#ifndef   eigen_state_space_systems_20180506011741
#define   eigen_state_space_systems_20180506011741

#include <memory>
#include <Eigen/Core>
#include <ros/node_handle.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>
  

namespace eigen_control_toolbox
{


bool importMatricesFromParam( const ros::NodeHandle&  nh, 
                              const std::string&      name,
                              Eigen::MatrixXd&        A,
                              Eigen::MatrixXd&        B,
                              Eigen::MatrixXd&        C,
                              Eigen::MatrixXd&        D,
                              std::string&            what);

/**
 * 
 * 
 * 
 * 
 */
class BaseDiscreteStateSpace : std::enable_shared_from_this<BaseDiscreteStateSpace>
{
private:
  BaseDiscreteStateSpace(const BaseDiscreteStateSpace&) = delete;
  BaseDiscreteStateSpace& operator=(const BaseDiscreteStateSpace&)= delete;
  BaseDiscreteStateSpace(BaseDiscreteStateSpace&&) = delete;
  BaseDiscreteStateSpace& operator=(BaseDiscreteStateSpace&&) = delete;

protected:
  double m_sampling_period;
  
public:
  typedef std::shared_ptr<BaseDiscreteStateSpace> Ptr;
  typedef std::shared_ptr<BaseDiscreteStateSpace const> ConstPtr;
  
  BaseDiscreteStateSpace()=default;
  BaseDiscreteStateSpace(const Eigen::MatrixXd& A, 
                         const Eigen::MatrixXd& B, 
                         const Eigen::MatrixXd& C, 
                         const Eigen::MatrixXd& D)
  {
  }
  virtual ~BaseDiscreteStateSpace() = default;

  bool importMatricesFromParam( const ros::NodeHandle&  nh, 
                                const std::string&      name)
  {
    Eigen::MatrixXd A,B,C,D;
    std::string what;
    if(!eigen_control_toolbox::importMatricesFromParam(nh, name, A,B,C,D,what))
    {
      std::cerr << __PRETTY_FUNCTION__ <<":"<<__LINE__<<":" << what << std::endl;
      return false;
    }

    if(!this->setMatrices(A,B,C,D,what))
    {
      std::cerr << __PRETTY_FUNCTION__ <<":"<<__LINE__<<":" << what << std::endl;
      return false;
    }
    return true;
  }

  virtual bool setMatrices( const Eigen::MatrixXd& A, 
                            const Eigen::MatrixXd& B, 
                            const Eigen::MatrixXd& C, 
                            const Eigen::MatrixXd& D,
                            std::string&           error) = 0;

  void setSamplingPeriod(const double& sampling_period)
  {
    m_sampling_period=sampling_period;
  }
  double getSamplingPeriod() const 
  {
    return m_sampling_period;
  }
};

typedef BaseDiscreteStateSpace::Ptr BaseDiscreteStateSpacePtr;
typedef BaseDiscreteStateSpace::ConstPtr BaseDiscreteStateSpaceConstPtr;

/**
 * 
 * 
 */
inline 
bool createDiscreteStateSpace(const ros::NodeHandle&  nh, const std::string& name, BaseDiscreteStateSpacePtr dss)
{
  Eigen::MatrixXd A,B,C,D;
  std::string error;
  if(!importMatricesFromParam(nh, name, A,B,C,D, error))
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" <<__LINE__ <<": " << error << std::endl;
    return false;
  }
  if(!(dss->setMatrices(A,B,C,D,error)))
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" <<__LINE__ <<": " << error << std::endl;
    return false;
  }
  return true;
}


/*
  m_output = m_C*m_state + m_D*input;
  m_state  = m_A*m_state + m_B*input;
  */
template< int S,          // State Dimension
          int I,          // Input Dimension  
          int O,          // Output Dimension
          int MaxS = S,   // g.t S if specified, it pre-allocates the max usable memory in the stack 
          int MaxI = I,   // g.t I if specified, it pre-allocates the max usable memory in the stack 
          int MaxO = O >  // g.t O if specified, it pre-allocates the max usable memory in the stack 
class DiscreteStateSpace : public BaseDiscreteStateSpace
{
private:
  enum { IW = (I > 0 && S > 0 ? I*S : Eigen::Dynamic), MaxIW = (MaxI>0 && MaxS>0 ? MaxI>0 && MaxS>0 : Eigen::Dynamic),
         OW = (O > 0 && S > 0 ? O*S : Eigen::Dynamic), MaxOW = (MaxO>0 && MaxS>0 ? MaxO>0 && MaxS>0 : Eigen::Dynamic),
         S_RC  = 0, // it is always Eigen::ColMajor
         I_RC  = 0, // it is always Eigen::ColMajor
         O_RC  = 0, // it is always Eigen::ColMajor
         A_RC  = 0, // it is always Eigen::ColMajor
         B_RC  = S==1 && (I>1||I==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         C_RC  = O==1 && (S>1||S==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         D_RC  = O==1 && (I>1||I==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         IW_RC = 0, // it is always Eigen::ColMajor
         OW_RC = 0, // it is always Eigen::ColMajor
         OBS_RC = OW==1 && (S >1||S ==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         I2O_RC = OW==1 && (IW>1||IW==Eigen::Dynamic) ? Eigen::RowMajor : Eigen::ColMajor,
         S_MAX  = S ==Eigen::Dynamic ? MaxS : S,
         I_MAX  = I ==Eigen::Dynamic ? MaxI : I,
         O_MAX  = O ==Eigen::Dynamic ? MaxO : O,
         IW_MAX = IW==Eigen::Dynamic ? MaxIW : IW,
         OW_MAX = OW==Eigen::Dynamic ? MaxOW : OW,
        };
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using State = typename std::conditional<S==1, 
                    double, Eigen::Matrix<double,S,1,S_RC,S_MAX>>::type;
  using Input   = typename std::conditional<I==1, 
                    double, Eigen::Matrix<double,I,1,I_RC,I_MAX>>::type;
  using Output  = typename std::conditional<O==1, 
                    double, Eigen::Matrix<double,O,1,O_RC,O_MAX,1>>::type;
  using MatrixA = typename std::conditional<S==1, 
                    double, Eigen::Matrix<double,S,S,A_RC,S_MAX,S_MAX>>::type;
  using MatrixB = typename std::conditional<S==1 && I==1, 
                    double, Eigen::Matrix<double,S,I,B_RC,S_MAX,I_MAX>>::type;
  using MatrixC = typename std::conditional<O==1 && S==1, 
                    double, Eigen::Matrix<double,O,S,C_RC,O_MAX,S_MAX>>::type;
  using MatrixD = typename std::conditional<O==1 && I==1,
                    double, Eigen::Matrix<double,O,I,D_RC,O_MAX, I_MAX>>::type;
  using InputWindow = typename std::conditional<IW==1, 
                    double, Eigen::Matrix<double,IW,1,IW_RC,IW_MAX>>::type;
  using OutputWindow = typename std::conditional<OW==1, 
                    double, Eigen::Matrix<double,OW,1,OW_RC,OW_MAX>>::type;
  using MatrixObs = typename std::conditional<OW==1 && S==1, 
                    double, Eigen::Matrix<double,OW,S,OBS_RC,OW_MAX, S_MAX>>::type;
  using MatrixI2O = typename std::conditional<OW==1 && IW==1, 
                    double, Eigen::Matrix<double,OW,IW,I2O_RC,OW_MAX, IW_MAX>>::type;

  DiscreteStateSpace()=default;
  virtual ~DiscreteStateSpace() = default;
  DiscreteStateSpace(const DiscreteStateSpace&) = delete;
  DiscreteStateSpace& operator=(const DiscreteStateSpace&)= delete;
  DiscreteStateSpace(DiscreteStateSpace&&) = delete;
  DiscreteStateSpace& operator=(DiscreteStateSpace&&) = delete;

  DiscreteStateSpace( const Eigen::MatrixXd& A, 
                      const Eigen::MatrixXd& B, 
                      const Eigen::MatrixXd& C, 
                      const Eigen::MatrixXd& D) 
    : BaseDiscreteStateSpace(A,B,C,D) 
  {
    std::string error;
    if(!this->setMatrices(A,B,C,D,error))
    {
      throw std::invalid_argument(("Error in memory management: "+error).c_str());
    }
  }

  virtual bool setMatrices(const Eigen::MatrixXd& A, 
                           const Eigen::MatrixXd& B, 
                           const Eigen::MatrixXd& C, 
                           const Eigen::MatrixXd& D,
                           std::string&           error);
   
  const MatrixA& getAMatrix() const {return m_A;};
  const MatrixB& getBMatrix() const {return m_B;};
  const MatrixC& getCMatrix() const {return m_C;};
  const MatrixD& getDMatrix() const {return m_D;};
  const Output&  getOutput()  const {return m_output;};
  const State&   getState()   const {return m_state;};
  int   getOrder()            const {return eigen_utils::rows(m_state);};
  int   getNumberOfInputs()   const {return eigen_utils::rows(m_input);};
  int   getNumberOfOutputs()  const {return eigen_utils::rows(m_output);};
  
  void  setState(const State& state);
  
  void print();
  
  /*
    * 
    * void setStateFromIO(const Eigen::Ref<Eigen::Matrix<double, N,1>> past_inputs,
    *                     const Eigen::Ref<Eigen::Matrix<double, N,1>> past_outputs);
    * 
    * Compute state from past inputs and outputs during a time window large as the state
    */
  virtual void setStateFromIO(const InputWindow& past_inputs, const OutputWindow& past_outputs);
  
  /*
    * 
    * void setStateFromLastIO( const Input& inputs,
    *                          const Output& outputs);
    * 
    * Compute state from input and the output
    */
  void setStateFromLastIO(const Input& inputs, const Output& outputs);
  
  virtual Output& update(const Input& input);
  
  const MatrixObs& computeObservatibilityMatrix( );
  
  const MatrixI2O& computeInputToOutputMatrix( );


protected:

  State  m_state;
  Input  m_input;
  Output m_output;
  
  MatrixA m_A;
  MatrixB m_B;
  MatrixC m_C;
  MatrixD m_D;
  MatrixObs m_Obs;
  MatrixI2O m_i2o;
  InputWindow m_past_input;
  OutputWindow m_past_output;
};

template<int S,int I,int O,int MS=S,int MI=I,int MO=O>
using DiscreteStateSpacePtr = std::shared_ptr<DiscreteStateSpace<S,I,O,MS,MI,MO> >;

typedef DiscreteStateSpace<-1,-1,-1> DiscreteStateSpaceX;


}


#include <eigen_state_space_systems/internal/eigen_state_space_systems_impl.h>

#endif
