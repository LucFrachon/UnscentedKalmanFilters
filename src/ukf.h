#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "tools.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using namespace std;
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

  //filenames for NIS calculations
  string rad_nis_filename_;
  string las_nis_filename_;

  //NIS out files
  ofstream outfile_rad_;
  ofstream outfile_las_;


  //timestep counter
  int t_counter_;
  //max timesteps
  int max_timesteps_;


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
  //Precondition: Before first execution, all vectors and matrices initialized to zero,
  //  parameters initialized to optimal values, is_initialized == false.
  //  Before subsequent executions, vectors and matrices hold results from previous step,
  //  is_initialized_ == true. The output files for radar and lidar NIS values are open.
  //Postcondition: Upon first execution, x_ is initialized using the first measured values
  //  and "sensible" values for components not included in measurements. P_ is initialized 
  //  to optimized values. is_initialized_ == true. weights_ received weighting values using
  //  the specified lambda_a_ value.
  //  Upon subsequent executions, x_ and P_ are updated through a predict and an update steps.
  //  Other matrices used in the calculations are updated as well with temporary values. 
  //  weights_ remains unchanged.
  //  The relevant NIS output file is updated with an additional NIS value. After the specified
  //  number of timesteps (max_timesteps_), both files are closed.


  void InitializeStateWithRadar(MeasurementPackage meas_package);
  //Precondition: meas_package is a measurement package that contains the first measurement 
  //  reading, if it's from the radar sensor. x_ and P_ are both initialized with zero values.
  //Postcondition: x_ received px and py calculated from the polar radar measurement. v, 
  //  psi and psi_dot are initialized to sensible values based on an educated guess.


  void InitializeStateWithLidar(MeasurementPackage meas_package);
  //Precondition: meas_package is a measurement package that contains the first measurement 
  //  reading, if it's from the lidar sensor. x_ and P_ are both initialized with zero values.
  //Postcondition: x_ received px and py directly from the lidar measurement. v, 
  //  psi and psi_dot are initialized to sensible values based on an educated guess and/or
  //  optimization.


  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);
  //Precondition: delta_t is the time between two steps, in s. x_ and P_ contain values 
  //  computed from the latest update step (or the initialization, if this is the second
  //  timestep). X_sig_pred_ contains zeros (if this is the second step) or values calculated
  //  at the previous Predict step.
  //Postcondition: x_ and P_ are changed with values predicted through augmented sigma point 
  //  transformation and subsequent estimation of the predicted mean and covariance (radar). 
  //  X_sig_pred_ is changed based on the values that x_ and P_ had before the start of this 
  //  Predict step.


  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);
  //Precondition: meas_package is a measurement package containing the latest lidar measurements.
  //  x_ and P_ contain the predicted state vector and covariance matrix at this timestep. The
  //  lidar NIS file contains NIS values from previous lidar update steps and is open.
  //Postcondition: x_ and P_ are updated based on the latest lidar measurements. The lidar NIS
  //  file receives an additional NIS value.


  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);
  //Precondition: meas_package is a measurement package containing the latest lidar measurements.
  //  x_ and P_ contain the predicted state vector and covariance matrix at this timestep.
  //  z_pred_rad_, Z_sig_rad_ and S_rad_ contain values calculated at the previous radar upadate.
  //  The radar NIS file contains NIS values from previous radar update steps and is open.
  //Postcondition: x_ and P_ are updated based on the latest lidar measurements. z_pred_rad_, 
  //  Z_sig_rad_ and S_rad_ contain temporary values. The radar NIS file receives an additional 
  //  NIS value.


  void GenerateAugmentedSigmaPoints(MatrixXd *Xsig_out);
  //Precondition: *Xsig_out points to an empty matrix with n_aug_ rows and n_sig_ columns.
  //Postcondition: Xsig_out containts the state values for each of the augmented sigma points, 
  //  calculated using the current state vector x_, the current state covariance matrix P_ 
  //  and the lambda_a_, std_a_ and std_yawdd_ parameters.

  void PredictSigmaPoints(MatrixXd Xsig_aug, double delta_t);
  //Precondition: Xsig_aug is a matrix with n_aug_ rows and n_sig_ columns, containing
  //  the augmented sigma points, passed on from GenerateAugmentedSigmnaPoints. Xsig_pred_ 
  // contains temporary values from the previous Predict step.
  //Postcondition: Xsig_pred_ is changed to contain the predicted sigma points after applying
  //  the process model to each.

  void PredictMeanAndCovariance();
  //Precondition: x_ and P_ contain values inherited from the previous Update step. Xsig_pred_
  //  contains the predicted sigma points from PredictSigmaPoints().
  //Postcondition: x_ and P_ are reset to zero then filled with the predicted mean and covariance 
  //  derived from the sigma points' transformation, using weights_.

  void PredictRadarMeasurement();
  //Precondition: z_pred_rad_, Z_sig_rad_ and S_rad_ contain values left over by previous Update
  //  step.
  //Postcondition: z_pred_rad_ contains predicted measurements (latest state estimation projected 
  //  to measurement space), Z_sig_rad_ contains the predicted measurement sigma points and S_rad_
  //  contains the predicted measurement covariance matrix.

  void UpdateRadarState(MeasurementPackage meas_package);
  //Precondition: meas_package contains the latest measurements. x_ and P_ contain predicted state
  //  and covariance matrix before the measurements come in.
  //Postcondition: x_ and P_ are updated after measurements are taken into account.

  void CalculateAndWriteNIS_radar(VectorXd z_diff);
  //Precondition: outfile_rad_ contains timesteps and NIS measurements from previous radar cycles.
  //  z_diff is the difference vector between the predicted and the actual measurements, as
  //  calculated within UpdateRadarState().
  //Postcondition: A new NIS value is computed and written in outfile_rad_, along with the current
  //  timestep.

  void CalculateAndWriteNIS_laser(VectorXd z_diff, MatrixXd S);
  //Precondition: outfile_las_ contains timesteps and NIS measurements from previous cycles.
  //  z_diff is the difference vector between the predicted and the actual measurements. S is the
  //  measurement covariance matrix. Both are calculated within UpdateLidar().
  //Postcondition: A new NIS value is computed and written in outfile_las_, along with the current
  //  timestep.

};

#endif /* UKF_H */
 