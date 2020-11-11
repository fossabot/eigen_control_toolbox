#ifndef   eigen_fir_filters_201805060814
#define   eigen_fir_filters_201805060814

#include <eigen_state_space_systems/eigen_state_space_systems.h>

namespace eigen_control_toolbox
{

  /*

  m_order=coeffs.cols();
  m_nout=coeffs.rows();
  m_nin=1;
  m_A.resize(m_order,m_order);
  m_B.resize(m_order,m_nin);
  m_C.resize(m_nout,m_order);
  m_D.resize(m_nout,m_nin);
  
 */

  template< int N, int MaxN = N>
  class FirFilter: public DiscreteStateSpace<N,1,N,MaxN,1,MaxN>
  {
  protected:
  public: 

    typedef Eigen::Matrix<double, N, N, 0, MaxN, MaxN> MatrixCoeff;
    
    typedef typename DiscreteStateSpace<N,1,N,MaxN,1,MaxN>::Input Input;
    typedef typename DiscreteStateSpace<N,1,N,MaxN,1,MaxN>::Output Output;
    typedef typename DiscreteStateSpace<N,1,N,MaxN,1,MaxN>::InputWindow InputWindow;
    typedef typename DiscreteStateSpace<N,1,N,MaxN,1,MaxN>::OutputWindow OutputWindow;
    
    FirFilter();
    FirFilter(const MatrixCoeff& coeffs);
    void computeMatrices(const MatrixCoeff& coeffs);
    
    virtual void setStateFromIO(const InputWindow& past_inputs, const OutputWindow& OutputWindow);
    virtual const Output& update(const Input& input);    
  };
  
  
  
}

#include <eigen_state_space_systems/internal/eigen_fir_filters_impl.h>
#endif
