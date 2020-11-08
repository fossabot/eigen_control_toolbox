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

  template< int DIM, int MaxDIM = DIM>
  class FirFilter: public DiscreteStateSpace<DIM,1,DIM,MaxDIM,1,MaxDIM>
  {
  protected:
  public: 

    typedef Eigen::Matrix<double, DIM, DIM, 0, MaxDIM, MaxDIM> MatrixCoeff;
    
    typedef typename DiscreteStateSpace<DIM,1,DIM,MaxDIM,1,MaxDIM>::Input Input;
    typedef typename DiscreteStateSpace<DIM,1,DIM,MaxDIM,1,MaxDIM>::Output Output;
    typedef typename DiscreteStateSpace<DIM,1,DIM,MaxDIM,1,MaxDIM>::InputWindow InputWindow;
    typedef typename DiscreteStateSpace<DIM,1,DIM,MaxDIM,1,MaxDIM>::OutputWindow OutputWindow;
    
    FirFilter();
    FirFilter(const MatrixCoeff& coeffs);
    void computeMatrices(const MatrixCoeff& coeffs);
    
    /*
     * 
     * void setStateFromIO(const Eigen::Ref<Eigen::VectorXd> past_inputs,
     *                     const Eigen::Ref<Eigen::VectorXd> past_outputs);
     * 
     * Compute state from input output, past inputs and outputs during a time window large as the state
     */
    virtual void setStateFromIO(const InputWindow& past_inputs, const OutputWindow& OutputWindow);
    virtual const Output& update(const Input& input);
    
  };
  
  
  
}

#include <eigen_state_space_systems/internal/eigen_fir_filters_impl.h>
#endif
