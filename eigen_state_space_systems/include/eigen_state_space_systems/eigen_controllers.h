#ifndef eigen_controllers_201806131212
#define eigen_controllers_201806131212

#include <eigen_state_space_systems/eigen_state_space_systems.h>

namespace eigen_control_toolbox
{

  template< int S,          // State Dimension
            int I,          // Input Dimension  
            int O,          // Output Dimension
            int MaxS = S,   // g.t S if specified, it pre-allocates the max usable memory in the stack 
            int MaxI = I,   // g.t I if specified, it pre-allocates the max usable memory in the stack 
            int MaxO = O >  // g.t O if specified, it pre-allocates the max usable memory in the stack 
  class Controller: public DiscreteStateSpace<S,I,O,MaxS,MaxI,MaxO>
  {
  protected:
    Eigen::MatrixXd m_Baw;

  public:

    using MatrixA = typename DiscreteStateSpace<S,I,O,MaxS,MaxI,MaxO>::MatrixA;
    using MatrixB = typename DiscreteStateSpace<S,I,O,MaxS,MaxI,MaxO>::MatrixB;
    using MatrixC = typename DiscreteStateSpace<S,I,O,MaxS,MaxI,MaxO>::MatrixC;
    using MatrixD = typename DiscreteStateSpace<S,I,O,MaxS,MaxI,MaxO>::MatrixD;
    using Output  = typename DiscreteStateSpace<S,I,O,MaxS,MaxI,MaxO>::Output;

    Controller();   
    Controller( const MatrixA& A, 
                const MatrixB& B, 
                const MatrixC& C, 
                const MatrixD& D);
    void setMatrices( const MatrixA& A, 
                      const MatrixB& B, 
                      const MatrixC& C, 
                      const MatrixD& D);
    virtual void setAntiWindupMatrix(const Eigen::Ref< Eigen::MatrixXd > D);
    
    virtual bool importMatricesFromParam(const ros::NodeHandle& nh, const std::string& controller_name);

    virtual double update(const double& input);
    void antiwindup(const Output& saturated_output,
                    const Output& unsaturated_output);
    void antiwindup(const double& saturated_output, const double& unsaturated_output);

    void setPI(const double& Kp, const double& Ki, const double& sampling_period);
    bool importPIFromParam(const ros::NodeHandle &nh, const std::string& name);
    bool importProportionalFromParam(const ros::NodeHandle &nh, const std::string &name);
  };  



}


#include <eigen_state_space_systems/internal/eigen_controllers_impl.h>
#endif
