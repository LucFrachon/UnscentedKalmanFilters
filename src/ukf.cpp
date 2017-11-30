#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

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
  x_.fill(0.);

  // predicted measurement vector (radar)
  z_pred_rad_ = VectorXd(3);
  z_pred_rad_.fill(0.);

  // predicted measurement vector (laser)
  z_pred_las_ = VectorXd(2);
  z_pred_las_.fill(0.);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.);

  // Measurement sigma points matrix (radar)  
  Z_sig_rad_ = MatrixXd(3, n_sig_);
  Z_sig_rad_.fill(0.);

  // Measurement covariance matrix (radar)
  S_rad_ = MatrixXd(3, 3);
  S_rad_.fill(0.);

  // Predicted state sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);
  Xsig_pred_.fill(0.);

  // Measurement sigma points matrix (laser)  
  Z_sig_las_ = MatrixXd(2, n_sig_);
  Z_sig_las_.fill(0.);

  // Measurement covariance matrix (laser)
  S_las_ = MatrixXd(2, 2);
  S_las_.fill(0.);

  // Predicted state sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, n_sig_);
  Xsig_pred_.fill(0.);

  //weights vector, will be used in the mean and covariance computations
  weights_ = VectorXd(n_sig_);
  weights_.fill(0.);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.5;  //initial value 30, try 1.5

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;  //initial value 30, try 2.3

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

  // Augmented sigma points spreading parameter
  lambda_a_ = 3 - n_aug_;  // Adjustable

  // instance of Tools class
  Tools tools_;

  //initialization indicator  
  is_initialized_ = false;

  //previous measurement (to overcome issues with numerical instability)
  MeasurementPackage prev_measurement_;

  //filenames for NIS calculations
  string rad_nis_filename_ = "radar_nis.csv";
  string las_nis_filename_ = "lidar_nis.csv";

  //NIS out files
  ofstream outfile_rad_;
  outfile_rad_.open(rad_nis_filename_.c_str(), ofstream::out);
  ofstream outfile_las_;
  outfile_las_.open(las_nis_filename_.c_str(), ofstream::out);

}

UKF::~UKF() {
  outfile_rad_.close();
  outfile_las_.close();
}

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
      InitializeStateWithRadar(meas_package);
      is_initialized_ = true;  // Initialization complete
    }
    else if(MeasurementPackage::LASER == meas_package.sensor_type_)
    {
      InitializeStateWithLidar(meas_package);
      is_initialized_ = true;  // Initialization complete
    }
   
    //calculate weights to use in future steps
    for(unsigned int i = 0; i < n_sig_; i++)
    {
      //calculate i-th weight
      weights_(i) = (0 == i) * lambda_a_ / (lambda_a_ + n_aug_) + (0 != i) / (2 * (lambda_a_ + n_aug_));
    }

    cout << "Weights vector = \n" << weights_ << endl;

    time_us_ = meas_package.timestamp_;
    prev_measurement_ = meas_package;
   
    return;
  }

  //After initialization
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.;  //convert from us to s 

  cout << "\n\n**************** time = " << time_us_ / 1000000. << " **********************\n\n";

  cout << "x_ = \n" << x_ << endl;
  cout << "P_ = \n" << P_ << endl;

  //*********************************************************
  // PREDICT

  cout << "\n --- Prediction ---\n";

  try 
  {
  Prediction(dt);
  } catch (std::range_error e)
  {
    cout << "\nNumerical error - reinitializing state to previous measurement\n";
    if(MeasurementPackage::LASER == prev_measurement_.sensor_type_)
    {
      InitializeStateWithLidar(prev_measurement_);    

    } else if (MeasurementPackage::RADAR == prev_measurement_.sensor_type_)
    {
      InitializeStateWithRadar(prev_measurement_);
    }
    cin.ignore();

  }
  cout << "\nPredicted state:\n" << x_ << endl;
  cout << "\nPredicted covariance:\n" << P_ << endl;


  //*********************************************************
  // UPDATE with new measurement

  if(MeasurementPackage::RADAR == meas_package.sensor_type_ && true == use_radar_)
  {
    cout << "\n --- Radar update ---\n";
    UpdateRadar(meas_package);
    cout << "\nUpdated state:\n" << x_ << endl;
    cout << "\nUpdated covariance:\n" << P_ << endl;
  } 
  else if(MeasurementPackage::LASER == meas_package.sensor_type_ && true == use_laser_)
  {
    cout << "\n --- Lidar update ---\n";
    UpdateLidar(meas_package);
    cout << "\nUpdated state:\n" << x_ << endl;
    cout << "\nUpdated covariance:\n" << P_ << endl;
  }

  prev_measurement_ = meas_package;
  time_us_ = meas_package.timestamp_;

}

void UKF::InitializeStateWithRadar(MeasurementPackage meas_package)
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

  // State covariance matrix
  P_ << 2, 0, 0, 0,   0,
        0, 4, 0, 0,   0,
        0, 0, 1, 0,   0,
        0, 0, 0, 0.5, 0,
        0, 0, 0, 0, 0.5;
}


void UKF::InitializeStateWithLidar(MeasurementPackage meas_package)
{
  // px and py are taken straight from the measurement
  double px_in = meas_package.raw_measurements_[0];
  double py_in = meas_package.raw_measurements_[1];

  // initial velocity is set to a sensible 'average' speed for a bicycle
  double v_in = 2.8;

  // all directions and yaw rates are equally likely
  double psi_in = 0.;
  double psi_dot_in = 0.;  //use non-zero value for testing purposes

  // feed these initial values to x_
  x_ << px_in, py_in, v_in, psi_in, psi_dot_in;

  // State covariance matrix
  P_ << 2, 0, 0, 0,   0,
        0, 4, 0, 0,   0,
        0, 0, 1, 0,   0,
        0, 0, 0, 0.5, 0,
        0, 0, 0, 0, 0.5;
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the laPredictRadarMeasurementst
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  //local matrix to store state vectors of augmented sigma points
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);
  Xsig_aug.fill(0.);

  //fill Xsig_aug with sigma points
  GenerateAugmentedSigmaPoints(&Xsig_aug);
  
  //predict the state of sigma points at timestep k+1|k
  PredictSigmaPoints(Xsig_aug, delta_t);
  
  //compute mean and covariance of the state vector at k+1|k
  PredictMeanAndCovariance();

}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) 
{
  int n_z = 2;  //lidar measurement dimension

  //get measurements
  VectorXd z = VectorXd(n_z);
  z << meas_package.raw_measurements_(0), 
       meas_package.raw_measurements_(1);

  //updated state vector
  VectorXd x_update = VectorXd(n_x_);
  x_update.fill(0.);

  //updated state covariance vector
  MatrixXd P_update = MatrixXd(n_x_, n_x_);
  P_update.fill(0.);

  //measurement noise matrix
  MatrixXd R(n_z, n_z);
  R << pow(std_laspx_,2), 0                 ,
       0                , pow(std_laspy_, 2);

  //projection matrix
  MatrixXd H = MatrixXd(n_z, n_x_);
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0;

  //difference vector
  VectorXd y = z - H * x_;

  //intermediate result (to avoid repeated calculation)
  MatrixXd PHt = P_ * H.transpose();

  //measurement covariance
  MatrixXd S = H * PHt + R;

  //Kalman gain
  MatrixXd K = PHt * S.inverse();

  //update estimated state
  x_ += K * y;

  //update estimated state covariance
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size) ;
  P_ = (I - K * H) * P_;

  //calculate and write NIS to file
  //use y as z_diff and S
  tools_.CalculateAndWriteNIS(outfile_las_, y, S, time_us_);
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) 
{
  //make measurement prediction based on latest state
  PredictRadarMeasurement();

  //update state and covariance matrix with measurement
  UpdateRadarState(meas_package);

  //TODO: Calculate Radar NIS  -- see in UpdateRadarState() 
}


void UKF::GenerateAugmentedSigmaPoints(MatrixXd *Xsig_out)
{
  VectorXd x_aug = VectorXd(n_aug_);  //augmented state vector
  x_aug.fill(0.);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);  //augmented state covariance matrix
  P_aug.fill(0.);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);  //matrix for augmented sigma points
  Xsig_aug.fill(0.);

  //create the augmented state vector
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;  //longitudinal acceleration noise has zero mean
  x_aug(6) = 0;  //yaw acceleration noise has zero mean

  //create augmented state covariance matrix
  P_aug.topLeftCorner(n_x_, n_x_) = P_;  //top left block is just P
  P_aug(n_x_, n_x_) = std_a_ * std_a_;   //last two diagonal elements are the variances
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;  //for long. and yaw accelerations

  //pre-compute the "square root" of matrix P
  //MatrixXd A = tools_.SqrtMatrix(P_aug);
  MatrixXd A = P_aug.llt().matrixL();

  cout << "\nP_aug:\n = \n" << P_aug << endl;
  cout << "\nSquare root of P_aug:\nA = \n" << A << endl;

  //The first column of Xsig is x_ itself:
  Xsig_aug.col(0) = x_aug;

  for(unsigned int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i + 1)          = x_aug + sqrt(lambda_a_ + n_aug_) * A.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_a_ + n_aug_) * A.col(i);
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
    double nu_a = Xsig_aug(5, i);  //longitudinal acceleration noise
    double nu_pdd = Xsig_aug(6, i);  //yaw acceleration noise

    //precompute some useful values
    double sin_psi = sin(psi);
    double cos_psi = cos(psi);
    double sin_1 = sin(psi + psi_dot * delta_t) - sin_psi;
    double cos_1 = cos_psi - cos(psi + psi_dot * delta_t);

    //components 0 and 1 differ if psi_dot == 0:
    if(fabs(psi_dot) >= EPS)
    {
      Xsig_pred_(0, i) = px +
                         v / psi_dot * sin_1 +
                         0.5 * pow(delta_t, 2) * cos_psi * nu_a;

      Xsig_pred_(1, i) = py +
                         v / psi_dot * cos_1 +
                         0.5 * pow(delta_t, 2) * sin_psi * nu_a;
    } else
    {
      Xsig_pred_(0, i) = px + 
                        v * cos_psi * delta_t +
                        0.5 * pow(delta_t, 2) * cos_psi * nu_a;              

      Xsig_pred_(1, i) = py + 
                        v * sin_psi * delta_t +
                        0.5 * pow(delta_t, 2) * sin_psi * nu_a;
    }

    //other components are the same regardless of the value of psi_dot
    Xsig_pred_(2, i) = v + delta_t * nu_a;
    Xsig_pred_(3, i) = psi + psi_dot * delta_t + 0.5 * pow(delta_t, 2) * nu_pdd;
    Xsig_pred_(4, i) = psi_dot + delta_t * nu_pdd;

  }
  cout << "\n\nXsig_pred_ = \n" << Xsig_pred_ << endl;
}


void UKF::PredictMeanAndCovariance()
{
  //set state vector and covariance matrix to empty
  x_.fill(0.0);
  P_.fill(0.0);

  //predict state vector at k+1|k
  x_ = Xsig_pred_ * weights_;

  //predict state covariance matrix at timestep k+1|k
  for(unsigned int i = 0; i < n_sig_; i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = tools_.NormalizeAngle(x_diff(3));

    P_ += weights_(i) * (x_diff * x_diff.transpose());
  }
}


void UKF::PredictRadarMeasurement()
{
  //measurement vector dimension; for radars it's 3 (rho, phi, rho_dot)
  int n_z = 3;

  //vector of mean predicted measurement
  VectorXd z_pred(n_z);
  z_pred << 0., 0., 0.;

  //matrix of measurement sigma points
  MatrixXd Z_sig(n_z, n_sig_);
  Z_sig.fill(0.);

  //measurement noise matrix
  MatrixXd R(n_z, n_z);
  R << pow(std_radr_,2), 0                  , 0                 ,
       0               , pow(std_radphi_, 2), 0                 ,              
       0               , 0                  , pow(std_radrd_, 2);

  //matrix of measurement covariance (radar)
  MatrixXd S(n_z, n_z);
  S.fill(0.);

  //project sigma points onto measurement space
  for(unsigned int i = 0; i < n_sig_; i++)
  {
    //for readability
    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double psi = Xsig_pred_(3, i);
    double psi_dot = Xsig_pred_(4, i);

    Z_sig(0, i) = sqrt(px * px + py * py);

    if(fabs(px) >= EPS)  //avoid div by 0
    {
      Z_sig(1, i) = atan2(py, px);

    } else if(py >= 0)  //if px = 0 and py > 0, vehicle is straight up
    { 
      Z_sig(1, i) = M_PI / 2.;

    } else  //if px = 0 and py < 0, vehicle is straight down
    {
      Z_sig(1, i) = -M_PI / 2.;
    }
    
    if(fabs(Z_sig(0, i)) >= EPS)  //avoid div by 0
    {
      Z_sig(2, i) = (px * cos(psi) * v + py * sin(psi) * v) / Z_sig(0, i);
    } else
    {
      Z_sig(2, i) = 0;
    }  
  }
    //calculate mean predicted measurement
  for(unsigned int i = 0; i < n_sig_; i++)
  {
    z_pred += weights_(i) * Z_sig.col(i);
  }

    //calculate predicted measurement covariance matrix S
  for(unsigned int i = 0; i < n_sig_; i++)
  {
    //distance of predicted sigma point to predicted mean measurement 
    VectorXd z_diff = Z_sig.col(i) - z_pred;
    z_diff(1) = tools_.NormalizeAngle(z_diff(1));

    //compute covariance matrix
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise
  S += R;

  z_pred_rad_ = z_pred;
  Z_sig_rad_ = Z_sig;
  S_rad_ = S;
}


void UKF::UpdateRadarState(MeasurementPackage meas_package)
{
  int n_z = 3;  //radar measurement dimension

  //updated state vector
  VectorXd x_update = VectorXd(n_x_);
  x_update.fill(0.);

  //updated state covariance vector
  MatrixXd P_update = MatrixXd(n_x_, n_x_);
  P_update.fill(0.);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.);

  //calculate cross correlation matrix
  for(unsigned int i = 0; i < n_sig_; i++)
  {
    //residual
    VectorXd z_diff = Z_sig_rad_.col(i) - z_pred_rad_;
    //angle normalization
    z_diff(1) = tools_.NormalizeAngle(z_diff(1));;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    x_diff(3) = tools_.NormalizeAngle(x_diff(3));
    
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }
  
  //update state mean and covariance matrix with new measurement
  
  //calculate Kalman gain K;
  MatrixXd K = MatrixXd(n_x_, n_z);
  K = Tc * S_rad_.inverse();
    
  //latest incoming measure
  VectorXd z = meas_package.raw_measurements_;
  
  //residual
  VectorXd z_diff = z - z_pred_rad_;

  //angle normalization
  z_diff(1) = tools_.NormalizeAngle(z_diff(1));

  x_update = x_ + K * z_diff;
  P_update = P_ - K * S_rad_ * K.transpose();

  //calculate NIS
  tools_.CalculateAndWriteNIS(outfile_rad_, z_diff, S_rad_, time_us_);

  //store calculations in class state variables
  x_ = x_update;
  P_ = P_update;
}

