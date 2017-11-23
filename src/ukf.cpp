#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5;  //initial value 30

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2.3;  //initial value 30

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  // Parameters above this line are scaffolding, do not modify

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // State covariance matrix
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Sigma point spreading parameter
  lambda_ = 3 - n_x_;  // Adjustable

  is_initialized_ = false;

  // instance of Tools class
  Tools tools_;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  
  if(!is_initialized_)
  {
    // take first measurement
    if(MeasurementPackage::RADAR == meas_package.sensor_type_)
    {
      // convert state from polar to cartesian coordinates
      float rho = meas_package.raw_measurements_[0];
      float phi = meas_package.raw_measurements_[1];
      float rho_dot = meas_package.raw_measurements_[2];

      // initial position is directly derived from the polar coordinates
      float px_in = rho * cos(phi);
      float py_in = rho * sin(phi);

      // initial value of yaw psi is set to phi if rho_dot >= 0, to -phi otherwise
      // because these values are the middles of the ranges of possible values for psi
      // depending on whether the object is moving towards us or away from us.
      float psi_in;

      if(rho_dot >= 0)  // object moving away
      {
        psi_in = phi;
      } 
      else  // object moving closer
      {
        psi_in = -phi;
      }
      
      // for a given value of rho_dot, v can vary between rho_dot and +inf 
      // (if psi - phi = +- pi / 2). The probability distribution for the values
      // of v depends on the probability of each value of psi - phi between 0 and pi.
      // This second probability distribution is uniform (each angle value is equally
      // likely) so the center of the distribution for v corresponds to psi - phi = pi / 4,
      // which corresponds to a value of v = rho_dot * sqrt(2)
      float v_in = rho_dot * sqrt(2);

      // for the yaw rate psi_dot, we have no useful information at this stage so we
      // will just assume linear motion
      float psi_dot_in = 0;

      // feed these initial values to x_
      x_ << px_in, py_in, v_in, psi_in, psi_dot_in;
    }
    else if(MeasurementPackage::LASER == meas_package.sensor_type_)
    {
      // px and py are taken straight from the measurement
      float px_in = meas_package.raw_measurements_[0];
      float py_in = meas_package.raw_measurements_[1];

      // initial velocity is set to a sensible 'average' speed for a bicycle
      float v_in = 2.8;

      // all directions and yaw rates are equally likely
      float psi_in = 0;
      float psi_dot_in = 0;

      // feed these initial values to x_
      x_ << px_in, py_in, v_in, psi_in, psi_dot_in;
    }

    float previous_timestamp = meas_package.timestamp_;
    is_initialized_ = true;  // Initialization complete

    cout << x_ << endl;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
