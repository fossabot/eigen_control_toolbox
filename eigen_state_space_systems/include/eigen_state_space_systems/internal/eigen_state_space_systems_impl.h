#ifndef   eigen_state_space_systems_impl_201811280956
#define   eigen_state_space_systems_impl_201811280956

#include <eigen_matrix_utils/eigen_matrix_utils.h>
#include <eigen_state_space_systems/eigen_control_utils.h>
#include <eigen_state_space_systems/eigen_state_space_systems.h>

namespace eigen_utils
{
  inline bool resize(const double& m, int rows, int cols)
  {
    return rows==1 && cols ==1;
  }

  /**
   * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
   */
  template<typename Derived,
          std::enable_if_t< (Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic) 
                          ||(Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic) 
                          , int> = 0>
  inline bool checkInputDim(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols, std::string& error)
  {
    if((m.rows()!= rows)||(m.cols()!= cols))
    {
      error += std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) + ":\n";
      error +=    id + " dimension mismatch."
          " Object size: " + std::to_string(m.rows()) + "x" + std::to_string(m.cols()) + ","
          " Expected size: " + std::to_string(rows) + "x" + std::to_string(cols) + "";
      return false;
    }
    return true;
  }


  /**
   * CHECK - SKIP, SINCE THE DIMENSION IS CHECKED AT COMPILE TIME
   */
  template<typename Derived,
          std::enable_if_t< (Eigen::MatrixBase<Derived>::RowsAtCompileTime != Eigen::Dynamic) 
                          && (Eigen::MatrixBase<Derived>::ColsAtCompileTime != Eigen::Dynamic) 
                          , int> = 0> 
  inline bool checkInputDim(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols, std::string& error)
  {
    return Eigen::MatrixBase<Derived>::RowsAtCompileTime == rows 
        && Eigen::MatrixBase<Derived>::ColsAtCompileTime == cols;
  }

  inline bool checkInputDim(const std::string& id, const double& m, int rows, int cols, std::string& error)
  {
    return rows == 1 && cols == 1;
  }

  template<typename Derived>
  inline void checkInputDimAndThrowEx(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols)
  {
    std::string error = std::string(__PRETTY_FUNCTION__) + ":" + std::to_string(__LINE__) + ":\n";
    if(!eigen_utils::checkInputDim(id,m,rows,cols,error))
    {
      throw std::runtime_error(error.c_str());
    }
  }

  inline void checkInputDimAndThrowEx(const std::string& id, const double& m, int rows, int cols)
  {
    if(rows !=1 && cols !=1)
    {
      throw std::runtime_error((id + ": expected a 1x1 (double) while a matrix has been assigned").c_str());
    }
  }

  // double, double
  inline bool copy(double& lhs, const double& rhs)
  {
    lhs = rhs;
    return true;
  }

  // double, matrix
  template<typename Derived> 
  inline bool copy(double& lhs, const Eigen::MatrixBase<Derived>& rhs)
  {
    if(rhs.rows()!=1 || rhs.cols()!=1)
    {
      return false;
    }
    lhs = rhs(0,0);
    return true;
  }

  // matrix, matrix
  template<typename Derived, typename OtherDerived>
  inline bool copy(Eigen::MatrixBase<Derived> & lhs, 
            const Eigen::MatrixBase<OtherDerived>& rhs)
  {
    if(!eigen_utils::resize(lhs, rhs.rows(),  rhs.cols()))
      return false;
    lhs = rhs;
    return true;
  }

  // matrix, double
  template<typename Derived> 
  inline bool copy(Eigen::MatrixBase<Derived>& lhs, const double& rhs)
  {
    lhs.setConstant(rhs);
    return true;
  }


  // A least-squares solution of m*x = rhs
  //     x   = S x 1
  //     rhs = (OxS) x 1
  //     m   = (OxS) x S
  template<typename Derived, typename InputDerived, typename OutputDerived>
  inline bool svd( const Eigen::MatrixBase<Derived>& m, // OW x S
            const Eigen::MatrixBase<InputDerived>& rhs,
            Eigen::MatrixBase<OutputDerived>& x)
  {
    Eigen::JacobiSVD< Eigen::MatrixXd > svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if(svd.rank()<m.cols())
    {
      return false;
    }
    x = svd.solve(rhs); // state at the begin of initialization interval
    return true;
  }

  inline bool svd(const double& m, 
                  const double& rhs,
                  double&       x)
  {
    x = rhs; // state at the begin of initialization interval
    return true;
  }
}  // namesapce eigen_utils

/**
 * 
 *
 * 
 * 
 * 
 */
namespace eigen_control_toolbox 
{


/**
 * The function is deprecated, in the future, the ROS dependecy from the library will be removed.
 */
inline 
bool BaseDiscreteStateSpace::importMatricesFromParam(const ros::NodeHandle&  nh, 
                                                     const std::string&      name)
{
  Eigen::MatrixXd A,B,C,D;
  std::string what;
  if(!eigen_control_toolbox::importMatricesFromParam(nh, name, A,B,C,D,what))
  {
    std::cerr << __PRETTY_FUNCTION__ <<":"<<__LINE__<<": what():\n" << what << std::endl;
    return false;
  }

  if(!this->setMatrices(A,B,C,D,what))
  {
    std::cerr << __PRETTY_FUNCTION__ <<":"<<__LINE__<<":" << what << std::endl;
    return false;
  }
  return true;
}

inline 
void BaseDiscreteStateSpace::setSamplingPeriod(const double& sampling_period)
{
  m_sampling_period=sampling_period;
}

inline 
double BaseDiscreteStateSpace::getSamplingPeriod() const 
{
  return m_sampling_period;
}



inline bool importMatricesFromParam(const ros::NodeHandle&  nh, 
                                    const std::string&      name,
                                    Eigen::MatrixXd&        A,
                                    Eigen::MatrixXd&        B,
                                    Eigen::MatrixXd&        C,
                                    Eigen::MatrixXd&        D,
                                    std::string&            what)
{
  std::string type;
  if(!nh.hasParam(name))
  {
      if(name.rfind("/") == 0)
        what = "The param '" +name + "' is not in ros param server";
      else
        what = "The param '" + nh.getNamespace() + "/" + name + "' is not in ros param server";
      return false;
  }

  if (nh.hasParam(name+"/type"))
  {
    if (!nh.getParam(name+"/type",type))
    {
      what = name +"/type is not a string";
      return false;
    }
  }
  else
  {
    what = name + "/type is not defined, using state-space";
    type  = "state-space";
  }

  if (!type.compare("unity"))
  {
    A.resize(1,1);
    B.resize(1,1);
    C.resize(1,1);
    D.resize(1,1);
    A.setZero();
    B.setZero();
    C.setZero();
    D(0,0)=1;
    return true;
  }

  if (!eigen_utils::getParam(nh, name+"/A", A))
  {
    what = "cannot find '" + name + "/A' parameter!";
    return false;
  }
  if (!eigen_utils::getParam(nh, name+"/B", B))
  {
    what = "cannot find '" + name + "/B' parameter!";
    return false;
  }
  if (!eigen_utils::getParam(nh, name+"/C", C))
  {
    what = "cannot find '" + name + "/C' parameter!";
    return false;
  }
  if (!eigen_utils::getParam(nh, name+"/D", D))
  {
    what = "cannot find '" + name + "/D' parameter!";
    return false;
  }
  return true;
}


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

/**
 * 
 * 
 * 
 * 
 */
template< int S, int I, int O, int MS, int MI, int MO > 
inline DiscreteStateSpace<S,I,O,MS,MI,MO>::DiscreteStateSpace( 
      const Eigen::MatrixXd& A, 
      const Eigen::MatrixXd& B, 
      const Eigen::MatrixXd& C, 
      const Eigen::MatrixXd& D) 
{
  std::string error;
  if(!this->setMatrices(A,B,C,D,error))
  {
    throw std::invalid_argument(("Error in memory management: "+error).c_str());
  }
}


template< int S, int I, int O, int MS, int MI, int MO > 
inline bool DiscreteStateSpace<S,I,O,MS,MI,MO>::setMatrices(
  const Eigen::MatrixXd& A,
  const Eigen::MatrixXd& B,
  const Eigen::MatrixXd& C,
  const Eigen::MatrixXd& D,
  std::string&           error )
{
  if(S==Eigen::Dynamic)
  {
    eigen_utils::resize(m_state, eigen_utils::rows(A), 1);    
  }
  if(I==Eigen::Dynamic)
  {
    eigen_utils::resize(m_input, eigen_utils::cols(B), 1);
  }
  if(O==Eigen::Dynamic)
  {
    eigen_utils::resize(m_output, eigen_utils::rows(C), 1);
  }
  error += std::string(__PRETTY_FUNCTION__) + ":\n\t";

  if(!eigen_utils::checkInputDim("Matrix A", A, getOrder(),           getOrder()          , error)) return false;
  if(!eigen_utils::checkInputDim("Matrix B", B, getOrder(),           getNumberOfInputs() , error)) return false;
  if(!eigen_utils::checkInputDim("Matrix C", C, getNumberOfOutputs(), getOrder()          , error)) return false;
  if(!eigen_utils::checkInputDim("Matrix D", D, getNumberOfOutputs(), getNumberOfInputs() , error)) return false;

  // it may change the dimension of the problem if the matrixes are dynamically allocated
  eigen_utils::copy(m_A, A); //A is S x S 
  eigen_utils::copy(m_B, B); //B is S x I
  eigen_utils::copy(m_C, C); //C is O x S
  eigen_utils::copy(m_D, D); //D is O x I

  eigen_utils::resize(m_Obs        , getNumberOfOutputs()*getOrder(), getOrder()); // Obs is (OxS) x S
  eigen_utils::resize(m_i2o        , getNumberOfOutputs()*getOrder(), getNumberOfInputs()*getOrder() );  // i2o is (OxS) x (IxS))
  eigen_utils::resize(m_past_input , getNumberOfInputs ()*getOrder(), 1); //  is IxS
  eigen_utils::resize(m_past_output, getNumberOfOutputs()*getOrder(), 1); //  is IxS

  return true;
}


template< int S, int I, int O, int MS, int MI, int MO > 
inline typename DiscreteStateSpace<S,I,O,MS,MI,MO>::Output& 
  DiscreteStateSpace<S,I,O,MS,MI,MO>::update(const DiscreteStateSpace<S,I,O,MS,MI,MO>::Input& input, bool skip_dimension_check)
{
  if(!DiscreteStateSpace<S,I,O,MS,MI,MO>::initialized())
  {
    throw std::runtime_error("The Discrete State Space has not been yet initialized (i.e., the matrices are undefined). Abort.");  
  }

  if(!skip_dimension_check)
    eigen_utils::checkInputDimAndThrowEx("Input", m_input, eigen_utils::rows(input), 1);
  m_input  = input;  
  m_output = m_C*m_state + m_D*m_input;
  m_state  = m_A*m_state + m_B*m_input;
  return m_output;
}

template< int S, int I, int O, int MS, int MI, int MO > 
inline void DiscreteStateSpace<S,I,O,MS,MI,MO>::setState(const DiscreteStateSpace<S,I,O,MS,MI,MO>::State& state)
{
  if(!DiscreteStateSpace<S,I,O,MS,MI,MO>::initialized())
  {
    throw std::runtime_error("The Discrete State Space has not been yet initialized (i.e., the matrices are undefined). Abort.");  
  }
  
  eigen_utils::checkInputDimAndThrowEx("State", m_state, eigen_utils::rows(state), 1);
  
  m_state = state;
}

template< int S, int I, int O, int MS, int MI, int MO > 
inline bool DiscreteStateSpace<S,I,O,MS,MI,MO>::setStateFromIO(
    const DiscreteStateSpace<S,I,O,MS,MI,MO>::InputWindow& past_inputs,
    const DiscreteStateSpace<S,I,O,MS,MI,MO>::OutputWindow& past_outputs)
{
  /*
   * 
   * y(0) = C*x(0)         + D u(0)
   * y(1) = C*A*x(0)       + D u(1) + C*B*u(0)
   * y(2) = C*A^2*x(0)     + D u(2) + C*B*u(1)+C*A*B*u(1)
   * ......
   * y(n) = C*A^(n-1)*x(0) + D u(n) + sum(j=0:n-1) C*A^j*B*u(n-1-j)
   *
   * Y=[y(0) ... y(n)]
   * U=[u(0) ... u(n)]
   * Y=obsv(A,C)*x(0)+ioMatrix(A,B,C,D)*U 
   * x(0)= obsv(A,C) \ ( Y-ioMatrix(A,B,C,D)*U )
   *
   * x(1) = A*x(0)+B*u(0)
   * ......
   * x(n) = A*x(n-1)+B*u(n-1)
   */
  std::string error;
  if(!eigen_utils::checkInputDim("Input Window", m_past_input, 
                    eigen_utils::rows(past_inputs), eigen_utils::cols(past_inputs), error))
  {
    std::cerr<<__PRETTY_FUNCTION__<<":"<<__LINE__<<":" << "Error in checking the input window dimension." << std::endl;
    std::cerr<< "\twhat():" << error << std::endl;
    return false;
  }
  if(!eigen_utils::checkInputDim("Output Window", m_past_output, 
    eigen_utils::rows(past_inputs), eigen_utils::cols(past_inputs), error))
  {
    std::cerr<<__PRETTY_FUNCTION__<<":"<<__LINE__<<":" << "Error in checking the input window dimension." << std::endl;
    std::cerr<< "\twhat():" << error << std::endl;
    return false;
  }
  
  m_past_input = past_inputs;
  m_past_output = past_outputs;

  computeObservatibilityMatrix();
  computeInputToOutputMatrix();

  if(!eigen_utils::svd(m_Obs, m_past_output - m_i2o * m_past_input, m_state) )
  {
    ROS_DEBUG("Matrix is rank deficient");
  }
  
  for(int istep=0;istep<getOrder();istep++)  
  {
    if(!eigen_utils::copy_from_block(m_input, m_past_input, istep*getNumberOfInputs(),0,getNumberOfInputs(),1))
    {
      std::cerr<<__PRETTY_FUNCTION__<<":"<<__LINE__<<":" << "Error in copying the past input in the input." << std::endl;
      return false;
    }
    update( m_input );
  }
  return true;
}

template< int S, int I, int O, int MS, int MI, int MO > 
inline bool DiscreteStateSpace<S,I,O,MS,MI,MO>::setStateFromLastIO(const Input& inputs, const Output& outputs)
{
  eigen_utils::checkInputDimAndThrowEx("Inputs", m_input, eigen_utils::rows(inputs), eigen_utils::cols(inputs));
  eigen_utils::checkInputDimAndThrowEx("Outputs", m_output, eigen_utils::rows(outputs), eigen_utils::cols(outputs));
  
  for (unsigned int idx=0;idx<getOrder();idx++)
  {
    if(!eigen_utils::copy_to_block(m_past_input, inputs, idx*getNumberOfInputs(), 0,getNumberOfInputs(), 1))
    {
      std::cerr << __PRETTY_FUNCTION__<<":"<<__LINE__<<":" << "Error in copy inputs in the input window list (" 
                << "idx=" <<idx <<" out of "<< getOrder()<<")" << std::endl;
      return false;
    }
    if(!eigen_utils::copy_to_block(m_past_output, outputs, idx*getNumberOfOutputs(),0,getNumberOfOutputs(),1))
    {
      std::cerr << __PRETTY_FUNCTION__<<":"<<__LINE__<<":" << "Error in copy outputs in the output window list (" 
                << "idx=" <<idx <<" out of "<< getOrder()<<")" << std::endl;
      return false;
    }
  }
  return setStateFromIO(m_past_input,m_past_output);
}
template< int S, int I, int O, int MS, int MI, int MO > inline 
const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixI2O& DiscreteStateSpace<S,I,O,MS,MI,MO>::computeInputToOutputMatrix()
{
  eigen_utils::setZero(m_i2o);
  for (unsigned int idx=0;idx<getOrder();idx++)
  {
    eigen_utils::copy_to_block(m_i2o, m_D, 
      idx*getNumberOfOutputs(),idx*getNumberOfInputs(), getNumberOfOutputs(),getNumberOfInputs());
  }
  
  MatrixA powA;
  eigen_utils::resize(powA,getOrder(), getOrder());
  eigen_utils::setIdentity(powA);
  
  for (unsigned int idx=1;idx<getOrder();idx++)
  {
    for (unsigned int idx2=0;idx2<(getOrder()-idx);idx2++)
    {
      eigen_utils::copy_to_block(m_i2o, m_C*powA*m_B,
        (idx+idx2)*getNumberOfOutputs(),(idx2)*getNumberOfInputs(), getNumberOfOutputs(),getNumberOfInputs());
    }
    powA*=m_A;
  }
  return m_i2o;
}

template< int S, int I, int O, int MS, int MI, int MO > 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixObs&
  DiscreteStateSpace<S,I,O,MS,MI,MO>::computeObservatibilityMatrix()
{
  eigen_utils::setZero(m_Obs);
  
  MatrixA pow_a;
  eigen_utils::resize(pow_a, getOrder(), getOrder());
  eigen_utils::setIdentity(pow_a);
  
  for (unsigned int idx=0;idx<getOrder();idx++)
  {
    eigen_utils::copy_to_block(m_Obs, m_C*pow_a, 
      idx*getNumberOfOutputs(),0,getNumberOfOutputs(), getOrder());
    pow_a=pow_a*m_A;
  }
  
  return m_Obs;
}

template< int S, int I, int O, int MS, int MI, int MO > 
inline bool DiscreteStateSpace<S,I,O,MS,MI,MO>::initialized()
{
  return getOrder() > 0 && getNumberOfInputs() > 0 && getNumberOfOutputs() > 0;
}

template< int S, int I, int O, int MS, int MI, int MO > inline void DiscreteStateSpace<S,I,O,MS,MI,MO>::print()
{
  ROS_INFO("system with %d inputs, %d states, %d outputs", getNumberOfInputs(), getOrder(), getNumberOfOutputs() );
  ROS_INFO_STREAM("A:\n"<< m_A);
  ROS_INFO_STREAM("B:\n"<< m_B);
  ROS_INFO_STREAM("C:\n"<< m_C);
  ROS_INFO_STREAM("D:\n"<< m_D);
  ROS_INFO_STREAM("output:\n"<< m_output);
  ROS_INFO_STREAM("state:\n" << m_state);
}

}  // namespace eigen_control_toolbox

#endif  // eigen_state_space_systems_impl_201811280956
