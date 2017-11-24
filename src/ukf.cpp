#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

const double EPS = 0.0001;  //threshold for div by zero detection

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // Number of augmented sigma points
  n_sig_ = 2 * n_aug_ + 1;  // Adjustable?

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);

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


  // State covariance matrix
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 1, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 1;

  // Augmented sigma point spreading parameter
  lambda_ = 3 - n_aug_;  // Adjustable

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
      double rho = meas_package.raw_measurements_[0];
      double phi = meas_package.raw_measurements_[1];
      double rho_dot = meas_package.raw_measurements_[2];

      // initial position is directly derived from the polar coordinates
      double px_in = rho * cos(phi);
      double py_in = rho * sin(phi);

      // initial value of yaw psi is set to phi if rho_dot >= 0, to -phi otherwise
      // because these values are the middles of the ranges of possible values for psi
      // depending on whether the object is moving towards us or away from us.
      double psi_in;

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
      double v_in = rho_dot * sqrt(2);

      // for the yaw rate psi_dot, we have no useful information at this stage so we
      // will just assume linear motion
      double psi_dot_in = 0;

      // feed these initial values to x_
      x_ << px_in, py_in, v_in, psi_in, psi_dot_in;
    }
    else if(MeasurementPackage::LASER == meas_package.sensor_type_)
    {
      // px and py are taken straight from the measurement
      double px_in = meas_package.raw_measurements_[0];
      double py_in = meas_package.raw_measurements_[1];

      // initial velocity is set to a sensible 'average' speed for a bicycle
      double v_in = 2.8;

      // all directions and yaw rates are equally likely
      double psi_in = 0;
      double psi_dot_in = 0;

      // feed these initial values to x_
      x_ << px_in, py_in, v_in, psi_in, psi_dot_in;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;  // Initialization complete

    return;
  }

  //After initialization
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.;  //convert from us to s 

  Prediction(dt);
  cout << "\nPredicted state:\n" << x_ << endl;
  cout << "\nPredicted covariance:\n" << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

  GenerateAugmentedSigmaPoints(&Xsig_aug);
  
  PredictSigmaPoints(Xsig_aug, delta_t);
  
  PredictMeanAndCovariance();
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

void UKF::GenerateAugmentedSigmaPoints(MatrixXd *Xsig_out)
{
  VectorXd x_aug = VectorXd(n_aug_);  //augmented state vector
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);  //augmented state covariance matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);  //matrix for augmented sigma points

  //create the augmented state vector
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;  //longitudinal acceleration noise has zero mean
  x_aug(6) = 0;  //yaw acceleration noise has zero mean

  //create augmented state covariance matrix
  P_aug.setZero();
  P_aug.topLeftCorner(n_x_, n_x_) = P_;  //top left block is just P
  P_aug(n_x_, n_x_) = std_a_ * std_a_;   //last two diagonal elements are the variances
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;  //for long. and yaw accelerations

  MatrixXd A = P_aug.llt().matrixL();  //pre-compute the "square root" of matrix P

  //The first column of Xsig is x_ itself:
  Xsig_aug.col(0) = x_aug;

  for(unsigned int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i + 1)          = x_aug + sqrt(lambda_ + n_aug_) * A.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i);
  }

  *Xsig_out = Xsig_aug;
}

void UKF::PredictSigmaPoints(MatrixXd Xsig_aug, double delta_t)
{
  for(unsigned int i = 0; i < n_sig_; i++)
  {
    //for code readability
    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v  = Xsig_aug(2, i);
    double psi = Xsig_aug(3, i);
    double psi_dot = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_pdd = Xsig_aug(6, i);

    //precompute some useful values
    double sin_psi = sin(psi);
    double cos_psi = cos(psi);
    double sin_1 = sin(psi + psi_dot * delta_t) - sin_psi;
    double cos_1 = cos_psi - cos(psi + psi_dot * delta_t);

    //components 0 and 1 differ if psi_dot == 0:
    if(fabs(psi_dot) >= EPS)
    {
      Xsig_pred_(0, i) = Xsig_aug(0, i) +
                        v / psi_dot * sin_1 +
                        0.5 * pow(delta_t, 2) * cos_psi * nu_a;

      Xsig_pred_(1, i) = Xsig_aug(1, i) +
                        v / psi_dot * cos_1 +
                        0.5 * pow(delta_t, 2) * sin_psi * nu_a;
    } else
    {
      Xsig_pred_(0, i) = Xsig_aug(0, i) + 
                        v * cos_psi * delta_t +
                        0.5 * pow(delta_t, 2) * cos_psi * nu_a;              

      Xsig_pred_(1, i) = Xsig_aug(1, i) + 
                        v * sin_psi * delta_t +
                        0.5 * pow(delta_t, 2) * sin_psi * nu_a;
    }

    //other components are the same regardless of the value of psi_dot
    Xsig_pred_(2, i) = Xsig_aug(2, i) + delta_t * nu_a;
    Xsig_pred_(3, i) = Xsig_aug(3, i) + psi_dot * delta_t + 0.5 * pow(delta_t, 2) * nu_pdd;
    Xsig_pred_(4, i) = Xsig_aug(4, i) + delta_t * nu_pdd;
  }
}

void UKF::PredictMeanAndCovariance()
{
  //weights vector, will be used in the mean and covariance computations
  VectorXd weights = VectorXd(n_sig_);

  //set state vector and covariance matrix to empty
  x_.fill(0.0);
  P_.fill(0.0);

  //calculate weights and predict state vector at k+1|k
  for(unsigned int i = 0; i < n_sig_; i++)
  {
    //calculate i-th weight
    weights(i) = (0 == i) * lambda_ / (lambda_ + n_aug_) + (0 != i) / (2 * (lambda_ + n_aug_));
    //calculate x
    x_ += weights(i) * Xsig_pred_.col(i);
  }

  //predict state covariance matrix at timestep k+1|k
  for(unsigned int i = 0; i < n_sig_; i++)
    P_ += weights(i) * (Xsig_pred_.col(i) - x_) * (Xsig_pred_.col(i) - x_).transpose();
}

