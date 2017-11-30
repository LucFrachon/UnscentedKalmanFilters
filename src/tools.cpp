#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {


  VectorXd rmse(4);
  rmse << 0., 0., 0., 0.;

    if (0 == estimations.size())  //Zero-size estimation vector
    {
      cout << "Error: The estimation vector size should not be zero.\n";
      return rmse;
    }

  if (estimations.size() != ground_truth.size())  //Estimation and ground truth vectors have 
                                                    //different sizes
  {
      cout << "Error: The estimation and ground truth vectors should have the same size.\n";
      return rmse;
  }

    
  //Accumulate squared residuals
  for (unsigned int t = 0; t < estimations.size(); t++)
  {
      VectorXd residual = estimations[t] - ground_truth[t];
      residual = residual.array() * residual.array();
      rmse += residual;
  }

  //Calculate the mean
  rmse /= estimations.size();

  //Take the square root
  rmse = rmse.array().sqrt();

  return rmse;
}

double Tools::NormalizeAngle(double angle)
{
  double angle_out = angle;
  while (angle_out > M_PI) angle_out -= 2. * M_PI;
  while (angle_out < -M_PI) angle_out += 2. * M_PI;
  return angle_out;
}


MatrixXd Tools::SqrtMatrix(MatrixXd M)
{
  Eigen::LLT<MatrixXd> LLT = M.llt();

  if(Eigen::NumericalIssue == LLT.info())
  {
    cout << "\nProblem with LLT!\n";
    Eigen::EigenSolver<MatrixXd> es(M);
    cout << "Eigenvalues of original matrix: \n"
         << es.eigenvalues() << endl;
    throw std::range_error("LLT failed");
  }
  return LLT.matrixL();
}

void Tools::CalculateAndWriteNIS(ofstream outfile, VectorXd z_diff, MatrixXd S, float time_us)
{
  float nis = z_diff.transpose() * S.inverse() * z_diff;
  outfile << time_us << "," << nis << "\n";
}