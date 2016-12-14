#include <sdre_controller_class.h>

#include <stdio.h>

#include <eigen3/unsupported/Eigen/src/MatrixFunctions/MatrixExponential.h>

SdreController::SdreController(
    const double & cart_mass,
    const double & pendulum1_mass,
    const double & pendulum2_mass,
    const double & pendulum1_length,
    const double & pendulum2_length)
  : cart_mass_(cart_mass),
    pendulum1_mass_(pendulum1_mass),
    pendulum2_mass_(pendulum2_mass),
    pendulum1_length_(pendulum1_length),
    pendulum2_length_(pendulum2_length),
    previous_time_(0.0) {
  d_ = Eigen::Matrix<double,3,3>::Zero();
  f_ = Eigen::Matrix<double,1,3>::Zero();
  Q_ = Eigen::Matrix<double,6,6>::Zero();
  ComputeCoef();
}

SdreController::~SdreController() {}

void SdreController::ComputeCoef() {
  d_(0,0) = cart_mass_ + pendulum1_mass_ + pendulum2_mass_;
  d_(0,1) = ((0.5 * pendulum1_mass_) + pendulum2_mass_) * pendulum1_length_;
  d_(0,2) = 0.5 * pendulum2_mass_ * pendulum2_length_;
  d_(1,1) = ((pendulum1_mass_ / 3.0) + pendulum2_mass_) * std::pow(pendulum1_length_,2.0);
  d_(1,2) = 0.5 * pendulum2_mass_ * pendulum1_length_ * pendulum2_length_;
  d_(3,3) = std::pow(pendulum2_length_,2.0) * pendulum2_mass_ / 3.0;
  d_(1,0) = d_(0,1);
  d_(2,0) = d_(0,2);
  d_(2,1) = d_(1,2);

  f_(1) = ((0.5 * pendulum1_mass_) + pendulum2_mass_) * pendulum1_length_ * 9.81;
  f_(2) = 0.5 * pendulum2_mass_ * pendulum2_length_ * 9.81;
}

double SdreController::ComputeCommand(
    const Eigen::Matrix<double, 6, 1> & X,
    const double & current_time) {

  double cos_theta1 = std::cos(X(1));
  double cos_theta2 = std::cos(X(2));
  double cos_delta_thetas = std::cos(X(1)-X(2));

  double sin_theta1 = std::sin(X(1));
  double sin_theta2 = std::sin(X(2));
  double sin_delta_thetas = std::sin(X(1)-X(2));

  Eigen::Matrix<double,3,3> D = d_;
  D(0,1) *= cos_theta1;
  D(0,2) *= cos_theta2;
  D(1,2) *= cos_delta_thetas;
  D(1,0) = D(0,1);
  D(2,0) = D(0,2);
  D(2,1) = D(1,2);

  Eigen::Matrix<double,3,3> C = Eigen::Matrix<double,3,3>::Zero();
  C(0,1) = -1.0 * d_(0,1) * sin_theta1 * X(5);
  C(0,2) = -1.0 * d_(0,2) * sin_theta2 * X(6);
  C(1,2) =  d_(1,2) * sin_delta_thetas * X(6);
  C(2,1) = -1.0 * d_(2,1) * sin_delta_thetas * X(5);

}


Eigen::Matrix<double,6,6> SdreController::SolveDare(
    const Eigen::Matrix<double,6,6> & Phi,
    const Eigen::Matrix<double,6,1> & Gamma,
    const Eigen::Matrix<double,6,6> & Q,
    const double & R)
{
}
