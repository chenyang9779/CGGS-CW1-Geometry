#ifndef COMPUTE_SCALAR_MLS_H
#define COMPUTE_SCALAR_MLS_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Cholesky>

double nanValue = std::numeric_limits<double>::quiet_NaN();


Eigen::VectorXd compute_scalar_mls(const Eigen::MatrixXd& gridLocations,
                                   const Eigen::MatrixXd& pointCloud,
                                   const Eigen::MatrixXd& pointNormals,
                                   const int N,
                                   const double h,
                                   const double epsNormal){
  
  using namespace Eigen;
  using namespace std;
  
  //stub function to delete: just a sphere
  RowVector3d mean = gridLocations.colwise().mean();
  VectorXd MLSValues(gridLocations.rows());
  for (int i=0;i<gridLocations.rows();i++)
    MLSValues(i) = (gridLocations.row(i)-mean).norm();
  MLSValues.array()-=MLSValues.mean();
  
  //TODO: replace with the scalar MLS computation.
  return MLSValues;
}



#endif
