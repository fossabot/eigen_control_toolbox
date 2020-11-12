#ifndef   eigen_state_space_systems_20180506011741
#define   eigen_state_space_systems_20180506011741

#include <memory>
#include <Eigen/Core>
#include <ros/node_handle.h>
#include <eigen_matrix_utils/eigen_matrix_utils.h>

/*!
 *  \addtogroup eigen_control_toolbox
 *  @{
 */

//! @brief The implementation of all the classes and functions for the management of a Discreate Space System
namespace eigen_control_toolbox
{

/** 
 * @brief The function extract the Matrices descring the Discrete Space System. 
 * The Discrete Space System describe the mathematical equations
 * 
 * X(k) = A * X(k-1) + B*u(k)
 * 
 * Y(k) = C * X(k-1) + D*u(k)
 * 
 * where 'X', 'y', and 'u' are the state, the output and the input respectively
 * 
 * NOTE: the function will be moved to an "utility" library, in order to remove the 
 *       ROS dependency to the lib 
 * @param[in] nh Node Handle 
 * @param[in] name name of the namespace for the 
 * @param[out]  A Matrix A
 * @param[out]  B Matrix B
 * @param[out]  C Matrix C
 * @param[out]  D Matrix D
 * @param[out]  what a string with the error, if something wrong happen inside
 * @return bool false if the params are not found in the param server, true otherwise
 * */
bool importMatricesFromParam( const ros::NodeHandle&  nh, 
                              const std::string&      name,
                              Eigen::MatrixXd&        A,
                              Eigen::MatrixXd&        B,
                              Eigen::MatrixXd&        C,
                              Eigen::MatrixXd&        D,
                              std::string&            what);

/** 
 * @brief The BaseDiscreteStateSpace is the base class for future extension of the library
 * At this moment, only one class is inherited from the BaseDiscreteStateSpace
 * The class implements the basic operations, without caring about the mathematical 
 * dimension and description of the State, Inputs and Outputs 
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
  virtual ~BaseDiscreteStateSpace() = default;

  /**
   * The function is deprecated, in the future, the ROS dependecy from the library will be removed.
   */
  [[deprecated("Use the Ctor, or call the function standalone, and pass the output to the Ctor")]]
  bool importMatricesFromParam( const ros::NodeHandle&  nh, 
                                const std::string&      name);

  //! The function may change with respect to the actual representation of the problem
  virtual bool setMatrices( const Eigen::MatrixXd& A, 
                            const Eigen::MatrixXd& B, 
                            const Eigen::MatrixXd& C, 
                            const Eigen::MatrixXd& D,
                            std::string&           error) = 0;

  //! Set the sampling period of the dicrete system 
  void setSamplingPeriod(const double& sampling_period);

  //! Get the sampling period of the dicrete system
  double getSamplingPeriod() const;
};

//! Shared Ptr definition
typedef BaseDiscreteStateSpace::Ptr BaseDiscreteStateSpacePtr;

//! Shared Ptr definition
typedef BaseDiscreteStateSpace::ConstPtr BaseDiscreteStateSpaceConstPtr;

/**
 * @brief the function is designed for future extension, if different implementation will be inherited
 */
bool createDiscreteStateSpace(const ros::NodeHandle&  nh, const std::string& name, BaseDiscreteStateSpacePtr dss);


/** 
 * @brief The function extract the Matrices descring the Discrete Space System. 
 * The Discrete Space System describe the mathematical equations
 * 
 * X(k) = A * X(k-1) + B*u(k);
 * 
 * Y(k) = C * X(k)   + D*u(k);
 * 
 * where 'X', 'y', and 'u' are the state, the output and the input respectively
 * \tparam S State Dimension
 * \tparam I Input Dimension
 * \tparam O Output Dimension
 * \tparam MaxS = S if specified, it pre-allocates the max usable memory in the stack 
 * \tparam MaxI = I if specified, it pre-allocates the max usable memory in the stack 
 * \tparam MaxO = O if specified, it pre-allocates the max usable memory in the stack 
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
  // some useful constant at compile time are introduced
  enum { IW    = (I > 0 && S > 0 ? I*S : Eigen::Dynamic),               // dimension of the input window
         OW    = (O > 0 && S > 0 ? O*S : Eigen::Dynamic),               // dimension of the output window
         MaxIW = (MaxI>0 && MaxS>0 ? MaxI>0 && MaxS>0 : Eigen::Dynamic),
         MaxOW = (MaxO>0 && MaxS>0 ? MaxO>0 && MaxS>0 : Eigen::Dynamic),
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

  //! State: dimension S, if S==1, then the State type is a double
  using State = typename std::conditional<S==1, 
                    double, Eigen::Matrix<double,S,1,S_RC,S_MAX>>::type;

  //! Input: dimension I, if I==1, then the Input type is a double
  using Input   = typename std::conditional<I==1, 
                    double, Eigen::Matrix<double,I,1,I_RC,I_MAX>>::type;

  //! Output: dimension O, if O==1, then the Output type is a double
  using Output  = typename std::conditional<O==1, 
                    double, Eigen::Matrix<double,O,1,O_RC,O_MAX,1>>::type;

  //! A: dimension S x S, if S==1, then the matrix A type is a double
  using MatrixA = typename std::conditional<S==1, 
                    double, Eigen::Matrix<double,S,S,A_RC,S_MAX,S_MAX>>::type;

  //! B: dimension S x I, if S==1 and I==1, then the matrix B type is a double
  using MatrixB = typename std::conditional<S==1 && I==1, 
                    double, Eigen::Matrix<double,S,I,B_RC,S_MAX,I_MAX>>::type;

  //! C: dimension O x S, if O==1 and S==1, then the matrix C type is a double
  using MatrixC = typename std::conditional<O==1 && S==1, 
                    double, Eigen::Matrix<double,O,S,C_RC,O_MAX,S_MAX>>::type;

  //! D: dimension O x I, if O==1 and I==1, then the matrix D type is a double
  using MatrixD = typename std::conditional<O==1 && I==1,
                    double, Eigen::Matrix<double,O,I,D_RC,O_MAX, I_MAX>>::type;

  //! Past Inputs: dimension (IxS) x I, if I==1 and S==1, then it reduces to a double
  using InputWindow = typename std::conditional<IW==1, 
                    double, Eigen::Matrix<double,IW,1,IW_RC,IW_MAX>>::type;

  //! Past Outputs: dimension (OxS) x I, if I==1 and S==1, then it reduces to a double
  using OutputWindow = typename std::conditional<OW==1, 
                    double, Eigen::Matrix<double,OW,1,OW_RC,OW_MAX>>::type;

  //! Obesrvation Matrix: dimension (OxS) x I, if O==1, I==1 and S==1, then it reduces to a double
  using MatrixObs = typename std::conditional<OW==1 && S==1, 
                    double, Eigen::Matrix<double,OW,S,OBS_RC,OW_MAX, S_MAX>>::type;

  //! Output to Input Matrix: dimension (OxS) x (IxS), if O==1, I==1 and S==1, then it reduces to a double
  using MatrixI2O = typename std::conditional<OW==1 && IW==1, 
                    double, Eigen::Matrix<double,OW,IW,I2O_RC,OW_MAX, IW_MAX>>::type;

  DiscreteStateSpace()=default;
  virtual ~DiscreteStateSpace() = default;
  DiscreteStateSpace(const DiscreteStateSpace&) = delete;
  DiscreteStateSpace& operator=(const DiscreteStateSpace&)= delete;
  DiscreteStateSpace(DiscreteStateSpace&&) = delete;
  DiscreteStateSpace& operator=(DiscreteStateSpace&&) = delete;

  /**
   *  @brief The constructor get dynamic allocated matrixes as input
   *  In the case the template is dynamic (e.g., S==-1, or I==-1, or O==-1)
   *  the dimension of the matrix will define the dimension of the system
   *  
   *  In the case the template is static, the dimension of the matricies is checked 
   */
  DiscreteStateSpace( const Eigen::MatrixXd& A, 
                      const Eigen::MatrixXd& B, 
                      const Eigen::MatrixXd& C, 
                      const Eigen::MatrixXd& D);

  /**
   *  @brief The constructor get dynamic allocated matrixes as input
   *  In the case the template is dynamic (e.g., S==-1, or I==-1, or O==-1)
   *  the dimension of the matrix will define the dimension of the system
   *  
   *  In the case the template is static, the dimension of the matricies is checked 
   */
  virtual bool setMatrices(const Eigen::MatrixXd& A, 
                           const Eigen::MatrixXd& B, 
                           const Eigen::MatrixXd& C, 
                           const Eigen::MatrixXd& D,
                           std::string&           error);
   
  //! accessor
  const MatrixA& getAMatrix() const {return m_A;};

  //! accessor
  const MatrixB& getBMatrix() const {return m_B;};

  //! accessor
  const MatrixC& getCMatrix() const {return m_C;};

  //! accessor
  const MatrixD& getDMatrix() const {return m_D;};

//! accessor to last input
  const Input&  getInput()  const {return m_input;};

  //! accessor to last comupted output
  const Output&  getOutput()  const {return m_output;};

  //! accessor to the internal state
  const State&   getState()   const {return m_state;};

  //! accessor to the order (state dimension)
  int   getOrder()            const {return eigen_utils::rows(m_state);};

  //! accessor to the input dimension
  int   getNumberOfInputs()   const {return eigen_utils::rows(m_input);};

  //! accessor to the output dimension
  int   getNumberOfOutputs()  const {return eigen_utils::rows(m_output);};
  
  /**
   * @brief set the state. The dimension is checked
   * NOTE: the input can be a 'double' or a Eigen::Matrix according to the template definition
   * If the State dimension is 1, the tyep 'State' refers to a double
   */
  void  setState(const State& state);
  
  //! print the matricies of the system
  void print();
  
  /**
   * @brief Compute state from past inputs and outputs during a time window large as the state
   */
  virtual bool setStateFromIO(const InputWindow& past_inputs, const OutputWindow& past_outputs);
  
  /**
   * @brief Compute state from input and the output
   */
  bool setStateFromLastIO(const Input& inputs, const Output& outputs);
  
  /**
   * @brief update the system, to be called at each sampling iteration
   * @param input The type of the input is an Eigen::Matrix<double,I,1> if I is different from 1
   *              while is a 'doube' if I is equal to 1
   * @return output The type of the input is an Eigen::Matrix<double,I,1> if I is different from 1
   *              while is a 'doube' if I is equal to 1
   */
  virtual Output& update(const Input& input);
  
  //! @brief compute the observability matrix
  const MatrixObs& computeObservatibilityMatrix( );
  
  //! @brief compute the input to output matrix
  const MatrixI2O& computeInputToOutputMatrix( );

  bool initialized();
  
protected:
  State  m_state;   //< To store the state, and update at each cycle
  Input  m_input;   //< To store only the dimension of the input.
  Output m_output;  //< To store only the dimension of the output.
  
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

/*! @} End of Doxygen Groups*/


#include <eigen_state_space_systems/internal/eigen_state_space_systems_impl.h>

#endif
