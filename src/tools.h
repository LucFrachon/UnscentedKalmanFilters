#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include <fstream>
#include <string>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /*
  Helper method to normalize angles
  */
  double NormalizeAngle(double angle);
  //Precondition: angle is a double
  //Postcondition: The return value is angle normalized to the [-pi, +pi] range by adding or substracting 
  //  2 * pi as many times as required.
  
  /* 
  Helper method to calculate square roots of a matrix. Includes checks to ensure no element of the 
  resulting matrix is negative nor complex
  */
  MatrixXd SqrtMatrix(MatrixXd M);
  //Precodition: M is a square matrix, pauseOnError indicates whether the program execution should
  //  pause when an error is found.
  //Postcondition: Returns the square root of the matrix M if it is positive and has no complex elements.
  //  If one of these conditions is not met, the function will return the identity matrix of same
  //  size as M. The code execution will stop if pauseOnError == true.
  
  void CalculateAndWriteNIS(ofstream outfile, VectorXd z_diff, MatrixXd S, float time_us);
  //Precondition: outfile is the file which will record the NIS values. z is an actual measurement vector,
  //  z_pred is the predicted measurement based on the current predicted state, S is the measurement 
  //  covariance matrix. time_s is a timestep expressed in microseconds.
  //Postcondition: NIS values are computed and written in outfile_name for each timestep.

};

#endif /* TOOLS_H_ */