#include <Eigen/Dense>

class SensorFusion
{
public:
  SensorFusion();
  virtual ~SensorFusion();

  enum StateMembers // state with respect to the global frame
  {
    StateX,
    StateY,
    StateZ,
    StateVx,
    StateVy,
    StateVz,
    StateRoll,
    StatePitch,
    StateYaw,

    StateDim,
  };

  enum ControlMembers // state with respect to the global frame
  {
    ControlRoll,
    ControlPitch,
    ControlYaw,
    ControlAx,
    ControlAy,
    ControlAz,

    ControlDim,
  };

  enum MeasurementMembers // state with respect to the global frame
  {
    MeasurementRange,

    MeasurementDim,
  };

  void initialize(const Eigen::VectorXd &initial_x,
                  const Eigen::MatrixXd &initial_covariance,
                  const Eigen::MatrixXd &initial_state_noise_r,
                  const Eigen::MatrixXd &initial_range_noise_q);

  void ekfPredict(const Eigen::VectorXd &u, const double &delta_t);
  void ekfCorrect(const double &z);

  void setStateNoiseR(const Eigen::MatrixXd &state_noise_r)
  {
    state_noise_r_ = state_noise_r;
    std::cout << "state_noise_r_ " << std::endl;
    std::cout << state_noise_r_ << std::endl;
  }

  void setRangeNoiseQ(const Eigen::MatrixXd &range_noise_q)
  {
    range_noise_q_ = range_noise_q;
    std::cout << "range_noise_q_ " << std::endl;
    std::cout << range_noise_q_ << std::endl;
  }

  Eigen::VectorXd getX() const
  {
    return x_;
  }

protected:
  bool initialized_;

  Eigen::VectorXd x_; // state
  Eigen::MatrixXd covariance_; // state covariance
  Eigen::MatrixXd state_noise_r_;
  Eigen::MatrixXd range_noise_q_;
};
