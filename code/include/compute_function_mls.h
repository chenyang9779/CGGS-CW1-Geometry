#ifndef COMPUTE_FUNCTION_MLS_H
#define COMPUTE_FUNCTION_MLS_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Cholesky>

double nanValue = std::numeric_limits<double>::quiet_NaN();

inline double wendlandWeight(double r, double h) {
  double s = 1.0 - r / h;
  return s*s*s*s*(4.0 * r / h + 1.0);
}


Eigen::VectorXd compute_function_mls(const Eigen::MatrixXd& gridLocations,
                                     const Eigen::MatrixXd& pointCloud,
                                     const Eigen::MatrixXd& pointNormals,
                                     const double h){
  
  using namespace Eigen;
  using namespace std;
  
  //stub function to delete: just a sphere
  // RowVector3d mean = gridLocations.colwise().mean();
  // VectorXd MLSValues(gridLocations.rows());
  // for (int i=0;i<gridLocations.rows();i++)
  //   MLSValues(i) = (gridLocations.row(i)-mean).norm();
  // MLSValues.array()-=MLSValues.mean();
  
  //TODO: replace with the scalar MLS computation.
  int nGrid = gridLocations.rows();
  int nPoints = pointCloud.rows();
  VectorXd MLSValues(nGrid);

  for (int i = 0; i < nGrid; i++){
    RowVector3d x = gridLocations.row(i);

    double topSum = 0.0;
    double bottomSum = 0.0;

    for (int j = 0; j < nPoints; j++){
      RowVector3d p = pointCloud.row(j);
      RowVector3d n = pointNormals.row(j);
      RowVector3d diff = x - p;
      double dist2 = diff.squaredNorm();
      if (dist2 < h * h){
        double dist = std::sqrt(dist2);
        double weight = wendlandWeight(dist, h);
        topSum += weight * n.dot(diff);
        bottomSum += weight;
      }
    }

    if (bottomSum == 0.0){
      MLSValues(i) = nanValue;
    } else {
      MLSValues(i) = topSum / bottomSum;
    }
  }

  return MLSValues;
}



#endif
