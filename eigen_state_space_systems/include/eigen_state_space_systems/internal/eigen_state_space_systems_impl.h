#ifndef   eigen_state_space_systems_impl_201811280956
#define   eigen_state_space_systems_impl_201811280956

#include <eigen_state_space_systems/eigen_control_utils.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>

#include <eigen_state_space_systems/eigen_state_space_systems.h>

/**
 * RESIZE - SAFE FUNCTION CALLED ONLY IF THE MATRIX IS DYNAMICALLY CREATED AT RUNTIME
 */
template<typename Derived,
         std::enable_if_t< (Eigen::MatrixBase<Derived>::RowsAtCompileTime == Eigen::Dynamic) 
                        || (Eigen::MatrixBase<Derived>::ColsAtCompileTime == Eigen::Dynamic) 
                        , int> = 0> 
bool checkInputDim(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols, std::string& error)
{
  if((m.rows() !=  rows)||(m.cols()!= cols))
  {
    error =    id + " dimension mismatch."
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
bool checkInputDim(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols, std::string& error)
{
  return true;
}

template<typename Derived>
void checkInputDimAndThrowEx(const std::string& id, const Eigen::MatrixBase<Derived>& m, int rows, int cols)
{
  std::string error;
  if(!checkInputDim(id,m,rows,cols,error))
  {
    throw std::runtime_error(error.c_str());
  }
}


namespace eigen_control_toolbox 
{

inline bool importMatricesFromParam(const ros::NodeHandle&  nh, 
                                    const std::string&      name,
                                    Eigen::MatrixXd&        A,
                                    Eigen::MatrixXd&        B,
                                    Eigen::MatrixXd&        C,
                                    Eigen::MatrixXd&        D,
                                    std::string&            what)
{
  std::string type;
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


/**
 * 
 * 
 * 
 * 
 */
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
    eigen_utils::resize(m_state, A.rows(), 1);    
  }
  if(I==Eigen::Dynamic)
  {
    eigen_utils::resize(m_input, m_B.cols(), 1);
  }
  if(O==Eigen::Dynamic)
  {
    eigen_utils::resize(m_output, m_C.rows(), 1);
  }

  if(!checkInputDim("A", A, getOrder(),           getOrder()          , error)) return false;
  if(!checkInputDim("B", B, getOrder(),           getNumberOfInputs() , error)) return false;
  if(!checkInputDim("C", C, getNumberOfOutputs(), getOrder()          , error)) return false;
  if(!checkInputDim("D", D, getNumberOfOutputs(), getNumberOfInputs() , error)) return false;

  // it may change the dimension of the problem if the matrixes are dynamically allocated
  m_A = A; //A is S x S 
  m_B = B; //B is S x I
  m_C = C; //C is O x S
  m_D = D; //D is O x I

  eigen_utils::resize(m_Obs        , getNumberOfOutputs()*getOrder(), getOrder()); // Obs is (OxS) x S
  eigen_utils::resize(m_i2o        , getNumberOfOutputs()*getOrder(), getNumberOfInputs()*getOrder() );  // i2o is (OxS) x (IxS))
  eigen_utils::resize(m_past_input , getNumberOfInputs ()*getOrder(), 1); //  is IxS
  eigen_utils::resize(m_past_output, getNumberOfOutputs()*getOrder(), 1); //  is IxS
  return true;
}


template< int S, int I, int O, int MS, int MI, int MO > 
inline typename DiscreteStateSpace<S,I,O,MS,MI,MO>::Output& 
  DiscreteStateSpace<S,I,O,MS,MI,MO>::update(const DiscreteStateSpace<S,I,O,MS,MI,MO>::Input& input)
{
  checkInputDimAndThrowEx("Input", m_input, input.rows(), 1);
  m_input  = input; 
  m_output = m_C*m_state + m_D*m_input;
  m_state  = m_A*m_state + m_B*m_input;
  return  m_output;
}

template< int S, int I, int O, int MS, int MI, int MO > 
inline void DiscreteStateSpace<S,I,O,MS,MI,MO>::setState(const DiscreteStateSpace<S,I,O,MS,MI,MO>::State& state)
{
  checkInputDimAndThrowEx("State", m_state, state.rows(), 1);
  m_state = state;
}

template< int S, int I, int O, int MS, int MI, int MO > 
inline void DiscreteStateSpace<S,I,O,MS,MI,MO>::setStateFromIO(
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
  checkInputDimAndThrowEx("Input Window", m_past_input, past_inputs.rows(), past_inputs.cols());
  checkInputDimAndThrowEx("Output Window", m_past_output, past_outputs.rows(), past_outputs.cols());
  
  m_past_input = past_inputs;
  m_past_output = past_outputs;

  computeObservatibilityMatrix();
  computeInputToOutputMatrix();
  
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(m_Obs, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.rank()<  getOrder())
  {
    ROS_DEBUG("Matrix is rank deficient");
    m_state.resize(  getOrder());
    m_state.setZero();
  }
  else 
  {
    m_state = svd.solve(m_past_output-m_i2o * m_past_input); // state at the begin of initialization interval
  }

  Input u;
  for (int istep=0;istep<getOrder();istep++)
  {
    u = m_past_input.block(istep*m_input.rows(),0,m_input.rows(),1);
    update(u);
  }
}

template< int S, int I, int O, int MS, int MI, int MO > 
inline void DiscreteStateSpace<S,I,O,MS,MI,MO>::setStateFromLastIO(const Input& inputs, const Output& outputs)
{
  checkInputDimAndThrowEx("Inputs", m_input, inputs.rows(), inputs.rows());
  checkInputDimAndThrowEx("Outputs", m_output, outputs.rows(), outputs.rows());
  
  for (unsigned int idx=0;idx<getOrder();idx++)
  {
    m_past_input.block(idx*m_input.rows(),0,m_input.rows(),1)=inputs;
    m_past_output.block(idx*m_input.rows(),0,m_input.rows(),1)=outputs;
  }
  setStateFromIO(m_past_input,m_past_output);
}
template< int S, int I, int O, int MS, int MI, int MO > inline 
const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixI2O& DiscreteStateSpace<S,I,O,MS,MI,MO>::computeInputToOutputMatrix()
{
  m_i2o.setZero();
  for (unsigned int idx=0;idx<  getOrder();idx++)
    m_i2o.block(idx*m_output.rows(),idx*m_input.rows(),m_output.rows(),m_input.rows())=m_D;
  
  Eigen::MatrixXd powA(getOrder(),  getOrder());
  powA.setIdentity();
  
  for (unsigned int idx=1;idx<  getOrder();idx++)
  {
    for (unsigned int idx2=0;idx2<(  getOrder()-idx);idx2++)
    {
      m_i2o.block((idx+idx2)*m_output.rows(),(idx2)*m_input.rows(),m_output.rows(),m_input.rows())=m_C*powA*m_B;
    }
    powA*=m_A;
  }
  return m_i2o;
}

template< int S, int I, int O, int MS, int MI, int MO > 
inline const typename DiscreteStateSpace<S,I,O,MS,MI,MO>::MatrixObs&
  DiscreteStateSpace<S,I,O,MS,MI,MO>::computeObservatibilityMatrix()
{
  m_Obs.setZero();
  
  Eigen::MatrixXd pow_a;
  pow_a.resize(  getOrder(),  getOrder());
  pow_a.setIdentity();
  
  for (unsigned int idx=0;idx<  getOrder();idx++)
  {
    m_Obs.block(idx*m_output.rows(),0,m_output.rows(),  getOrder())=m_C*pow_a;
    pow_a=pow_a*m_A;
  }
  
  return m_Obs;
}

template< int S, int I, int O, int MS, int MI, int MO > inline void DiscreteStateSpace<S,I,O,MS,MI,MO>::print()
{
  ROS_INFO("system with %ld inputs, %ld states, %ld outputs", m_input.rows(),m_state.rows(), m_output.rows() );
  ROS_INFO_STREAM("A:\n"<< m_A);
  ROS_INFO_STREAM("B:\n"<< m_B);
  ROS_INFO_STREAM("C:\n"<< m_C);
  ROS_INFO_STREAM("D:\n"<< m_D);
  ROS_INFO_STREAM("output:\n"<<  m_output);
  ROS_INFO_STREAM("state:\n"<< m_state);
}

// ========================================================================
//
//
// ========================================================================
// ========================================================================
// SPEC
// ========================================================================
// ========================================================================
template< int S, int MS > 
inline bool DiscreteStateSpace<S,1,1,MS,1,1>::setMatrices(const Eigen::MatrixXd& A,
                                                          const Eigen::MatrixXd& B,
                                                          const Eigen::MatrixXd& C,
                                                          const Eigen::MatrixXd& D,
                                                          std::string&           error )
{
  if(S==Eigen::Dynamic)
  {
    eigen_utils::resize(m_state, A.rows(), 1);    
  }
  
  if(!checkInputDim("A", A, getOrder(),           getOrder()          , error)) return false;
  if(!checkInputDim("B", B, getOrder(),           getNumberOfInputs() , error)) return false;
  if(!checkInputDim("C", C, getNumberOfOutputs(), getOrder()          , error)) return false;
  if(!checkInputDim("D", D, getNumberOfOutputs(), getNumberOfInputs() , error)) return false;;

  // it may change the dimension of the problem if the matrixes are dynamically allocated
  m_A = A; //A is S x S 
  m_B = B; //B is S x 1
  m_C = C; //C is 1 x S
  m_D = D(0,0); //D is 1 x 1

  eigen_utils::resize(m_Obs        , getNumberOfOutputs()*getOrder(), getOrder()); // Obs is (OxS) x S
  eigen_utils::resize(m_i2o        , getNumberOfOutputs()*getOrder(), getNumberOfInputs()*getOrder() );  // i2o is (OxS) x (IxS))
  eigen_utils::resize(m_past_input , getNumberOfInputs ()*getOrder(), 1); //  is IxS
  eigen_utils::resize(m_past_output, getNumberOfOutputs()*getOrder(), 1); //  is IxS
  return true;
}


template< int S, int MS > 
inline typename DiscreteStateSpace<S,1,1,MS,1,1>::Output& 
  DiscreteStateSpace<S,1,1,MS,1,1>::update(const DiscreteStateSpace<S,1,1,MS,1,1>::Input& input)
{
  m_input  = input; 
  m_output = m_C*m_state + m_D*m_input;
  m_state  = m_A*m_state + m_B*m_input;
  return  m_output;
}

template< int S, int MS > 
inline void DiscreteStateSpace<S,1,1,MS,1,1>::setState(const DiscreteStateSpace<S,1,1,MS,1,1>::State& state)
{
  checkInputDimAndThrowEx("State", m_state, state.rows(), 1);
  m_state = state;
}

template< int S, int MS > 
inline void DiscreteStateSpace<S,1,1,MS,1,1>::setStateFromIO(
    const DiscreteStateSpace<S,1,1,MS,1,1>::InputWindow& past_inputs,
    const DiscreteStateSpace<S,1,1,MS,1,1>::OutputWindow& past_outputs)
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
  checkInputDimAndThrowEx("Input Window", m_past_input, past_inputs.rows(), past_inputs.cols());
  checkInputDimAndThrowEx("Output Window", m_past_output, past_outputs.rows(), past_outputs.cols());
  
  m_past_input = past_inputs;
  m_past_output = past_outputs;

  computeObservatibilityMatrix();
  computeInputToOutputMatrix();
  
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(m_Obs, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.rank()<  getOrder())
  {
    ROS_DEBUG("Matrix is rank deficient");
    m_state.resize(  getOrder());
    m_state.setZero();
  }
  else 
  {
    m_state = svd.solve(m_past_output-m_i2o * m_past_input); // state at the begin of initialization interval
  }

  Input u;
  for (int istep=0;istep<getOrder();istep++)
  {
    u = m_past_input(istep,0);
    update(u);
  }
}

template< int S, int MS > 
inline void DiscreteStateSpace<S,1,1,MS,1,1>::setStateFromLastIO(const Input& inputs, const Output& outputs)
{
  for (unsigned int idx=0;idx<getOrder();idx++)
  {
    m_past_input(idx*1,0)=inputs;
    m_past_output(idx*1,0)=outputs;
  }
  setStateFromIO(m_past_input,m_past_output);
}



template< int S, int MS > inline 
const typename DiscreteStateSpace<S,1,1,MS,1,1>::MatrixI2O& DiscreteStateSpace<S,1,1,MS,1,1>::computeInputToOutputMatrix()
{
  m_i2o.setZero();
  for (unsigned int idx=0;idx<  getOrder();idx++)
    m_i2o(idx*1,idx*1)=m_D;
  
  Eigen::MatrixXd powA(getOrder(),  getOrder());
  powA.setIdentity();
  
  for (unsigned int idx=1;idx<  getOrder();idx++)
  {
    for (unsigned int idx2=0;idx2<(  getOrder()-idx);idx2++)
    {
      m_i2o.block((idx+idx2)*1,(idx2)*1,1,1)=m_C*powA*m_B;
    }
    powA*=m_A;
  }
  return m_i2o;
}

template< int S, int MS > 
inline const typename DiscreteStateSpace<S,1,1,MS,1,1>::MatrixObs&
  DiscreteStateSpace<S,1,1,MS,1,1>::computeObservatibilityMatrix()
{
  m_Obs.setZero();
  
  Eigen::MatrixXd pow_a;
  pow_a.resize(  getOrder(),  getOrder());
  pow_a.setIdentity();
  
  for (unsigned int idx=0;idx<  getOrder();idx++)
  {
    m_Obs.block(idx*1,0,1,  getOrder())=m_C*pow_a;
    pow_a=pow_a*m_A;
  }
  
  return m_Obs;
}

template< int S, int MS > inline void DiscreteStateSpace<S,1,1,MS,1,1>::print()
{
  ROS_INFO("system with %ld inputs, %ld states, %ld outputs", 1,m_state.rows(), 1 );
  ROS_INFO_STREAM("A:\n"<< m_A);
  ROS_INFO_STREAM("B:\n"<< m_B);
  ROS_INFO_STREAM("C:\n"<< m_C);
  ROS_INFO_STREAM("D:\n"<< m_D);
  ROS_INFO_STREAM("output:\n"<<  m_output);
  ROS_INFO_STREAM("state:\n"<< m_state);
}

}

#endif
