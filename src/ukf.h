#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "tools.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* predicted measurement vector (radar)
  VectorXd z_pred_rad_;

  ///* Measurement sigma points matrix (radar)
  MatrixXd Z_sig_rad_;

  ///* Measurement covariance matrix (radar)
  MatrixXd S_rad_;

   ///* predicted measurement vector (laser)
  VectorXd z_pred_las_;

  ///* Measurement sigma points matrix (laser)
  MatrixXd Z_sig_las_;

  ///* Measurement covariance matrix (laser)
  MatrixXd S_las_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Number of sigma points
  int n_sig_;

  ///* Sigma point spreading parameter
  double lambda_a_;

  ///* Instance of Tools
  Tools tools_;

  ///* Previous measurement (to overcome issues with numerical instability)
  MeasurementPackage prev_measurement_;


  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();
  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);



  void InitializeStateWithRadar(MeasurementPackage meas_package);
  //TODO: Function description

  void InitializeStateWithLidar(MeasurementPackage meas_package);
  //TODO: Function description



  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  void GenerateAugmentedSigmaPoints(MatrixXd *Xsig_out);
  //Precondition: *Xsig_out points to an empty matrix with n_aug_ rows and n_sig_ columns
  //Postcondition: Xsig_out containts the state values for each of the sigma points

  void PredictSigmaPoints(MatrixXd Xsig_aug, double delta_t);
  //Precondition: Xsig_aug is a matrix with n_aug_ rows and n_sig_ columns, containing
  //  the augmented sigma points
  //Postcondition: Xsig_out is a n_x_ * n_sig_ matrix containing the predicted states
  //  for the sigma points

  void PredictMeanAndCovariance();
  //Postcondition: x_ and P_ are modified to show the predicted mean and covariance 
  //  derived from the sigma points' transformation.

  void PredictRadarMeasurement();
  //Postcondition: z_pred_rad_ contains predicted measurements (latest state estimation projected 
  //  to measurement space), Z_sig_rad_ contains the predicted measurement sigma points and S_rad_
  //  contains the predicted measurement covariance matrix.

  void UpdateRadarState(MeasurementPackage meas_package);
  //Precondition: meas_package contains the latest measurements. x_ and P_ contain predicted state
  //  and covariance matrix before the measurements come in.
  //Postcondition: x_ and P_ are updated after measurements are taken into account.
};

#endif /* UKF_H */
