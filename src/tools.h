#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

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
  void NormalizeAngle(float& angle);
  //Precondition: angle is a float
  //Postcondition: angle is normalized to the [-pi, +pi] range by adding or substracting 2 * pi as
  //many times as required.
  
};

#endif /* TOOLS_H_ */