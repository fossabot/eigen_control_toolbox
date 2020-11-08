#ifndef   eigen_fir_filters_impl_201811280952
#define   eigen_fir_filters_impl_201811280952

#include <eigen_state_space_systems/eigen_fir_filters.h>

namespace eigen_control_toolbox
{

template<int DIM, int MaxDIM> 
inline FirFilter<DIM,MaxDIM>::FirFilter()
{

}

template<int DIM, int MaxDIM> 
inline FirFilter<DIM,MaxDIM>::FirFilter(const FirFilter<DIM,MaxDIM>::MatrixCoeff& coeffs)
{
  computeMatrices(coeffs);
}

template<int DIM, int MaxDIM> 
inline void FirFilter<DIM,MaxDIM>::computeMatrices(const FirFilter<DIM,MaxDIM>::MatrixCoeff& coeffs)
{

  this->initState(coeffs.cols());
  this->initOutput(coeffs.rows());
  
  this->initMatrixA(coeffs.cols());
  this->initMatrixB(coeffs.cols(),1);
  this->initMatrixC(coeffs.rows(),coeffs.cols());
  this->initMatrixD(coeffs.rows(),1);
  
  this->m_A.setZero();
  this->m_A.block(1,0,this->getOrder()-1,this->getOrder()-1).setIdentity();
  this->m_B.setZero();
  this->m_B(0)=1;
  this->m_C=coeffs.block(0,1,this->getNumberOfOutputs(),this->getOrder()-1);
  this->m_D=coeffs.block(0,0,this->getNumberOfOutputs(),1);
}

template<int DIM, int MaxDIM> 
inline const typename FirFilter<DIM,MaxDIM>::Output&
 FirFilter<DIM,MaxDIM>::update(const FirFilter<DIM,MaxDIM>::Input& input)
{
  this->checkInput(input.rows());

  this->m_output = this->m_C*this->m_state + this->m_D*input;
  this->m_state.block(1,0,this->getOrder()-1,1)=this->m_state.block(0,0,this->getOrder()-1,1);
  this->m_state(0)=input(0);
  
  return this->m_output;
}

template<int DIM, int MaxDIM> 
inline void FirFilter<DIM,MaxDIM>::setStateFromIO(const FirFilter<DIM,MaxDIM>::InputWindow& past_inputs, 
                                                  const FirFilter<DIM,MaxDIM>::OutputWindow& past_outputs)
{
  this->checkOutput( past_outputs.rows() );
//  this->m_state = past_inputs; // ?? //
}





}

#endif
