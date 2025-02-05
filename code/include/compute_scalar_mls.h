#ifndef COMPUTE_SCALAR_MLS_H
#define COMPUTE_SCALAR_MLS_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/QR> 

double nanValue = std::numeric_limits<double>::quiet_NaN();

inline std::vector<std::array<int,3>> generateEponents(int N){
  std::vector<std::array<int, 3>> exponents;
  for (int total = 0; total <= N; ++total) {
    for (int i = 0; i <= total; ++i) {
      for (int j = 0; j <= total - i; ++j) {
        int k = total - i - j;
        exponents.push_back({i, j, k});
      }
    }
  }
  return exponents;
}
inline double wendlandWeight(double r, double h) {
  if (r >= h){
    return 0.0;
  }
  double s = 1.0 - r / h;
  return s*s*s*s*(4.0 * r / h + 1.0);
}

inline Eigen::VectorXd monomials3D(double x, double y, double z, const std::vector<std::array<int, 3>> &exponents){
  int nTerm = exponents.size();
  Eigen::VectorXd basis(nTerm);

  for (int i = 0; i < nTerm; i++){
    basis(i) = std::pow(x, exponents[i][0]) * std::pow(y, exponents[i][1]) * std::pow(z, exponents[i][2]);
  }

  return basis;
}

Eigen::VectorXd compute_scalar_mls(const Eigen::MatrixXd& gridLocations,
                                   const Eigen::MatrixXd& pointCloud,
                                   const Eigen::MatrixXd& pointNormals,
                                   const int N,
                                   const double h,
                                   const double epsNormal){
  
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
  VectorXd MLSValues(nGrid);

  vector<std::array<int, 3>> exponents = generateEponents(N);
  int nBasis = exponents.size();

  // std::vector<Eigen::VectorXd> precomputedMonomials(pointCloud.rows());

  // for (int j = 0; j < pointCloud.rows(); j++) {
  //   precomputedMonomials[j] = monomials3D(pointCloud(j, 0), pointCloud(j, 1), pointCloud(j, 2), exponents);
  // }

  for (int i = 0; i < nGrid; i++){
    RowVector3d gridPt = gridLocations.row(i);

    vector<RowVector3d> localPoints;
    vector<double> localValues;
    vector<double> localWeights;

    for (int j = 0; j < pointCloud.rows(); j++){
      RowVector3d p = pointCloud.row(j);
      RowVector3d n = pointNormals.row(j);
      RowVector3d pP = p + n * epsNormal;
      RowVector3d pM = p - n * epsNormal;


      // if (diff.cwiseAbs().maxCoeff() > h) continue;
      RowVector3d diff;
      diff = p - gridPt;
      double dist2;
      // if (diff.cwiseAbs().maxCoeff() < h){
        dist2 = diff.squaredNorm();
        if (dist2 <= h*h) {
          localPoints.push_back(p);
          localValues.push_back(0.0);
          double weight = wendlandWeight(std::sqrt(dist2), h);
          localWeights.push_back(weight);
          
        }
      // }

      diff = pP - gridPt;
      // if (diff.cwiseAbs().maxCoeff() < h){
        dist2 = diff.squaredNorm();
        if (dist2 <= h*h) {
          localPoints.push_back(pP);
          localValues.push_back(+epsNormal);
          double weight = wendlandWeight(std::sqrt(dist2), h);
          localWeights.push_back(weight);
          
        }
      // }

      diff = pM - gridPt;
      // if (diff.cwiseAbs().maxCoeff() < h){
        dist2 = diff.squaredNorm();
        if (dist2 <= h*h) {
          localPoints.push_back(pM);
          localValues.push_back(-epsNormal);
          double weight = wendlandWeight(std::sqrt(dist2), h);
          localWeights.push_back(weight);
        }
      // }
    }
    int nLocal = localPoints.size();
    if (nLocal < nBasis){
      // std::cout << "Grid point " << i << " has too few local points (" << nLocal << "), setting NaN." << std::endl;
      MLSValues(i) = nanValue;
      continue;
    }

    MatrixXd A(nLocal, nBasis);
    VectorXd b(nLocal);

    for (int r = 0; r < nLocal; r++){
      double weight = localWeights[r];
      double sqrtWeight = std::sqrt(weight);
      A.row(r) = monomials3D(localPoints[r].x(), localPoints[r].y(), localPoints[r].z(), exponents) * sqrtWeight;
      b(r) = localValues[r] * sqrtWeight;
    }


    MatrixXd lhs = A.transpose() * A;
    VectorXd rhs = A.transpose() * b;
    
    Eigen::LDLT<MatrixXd> ldlt(lhs);
    
    VectorXd alpha;

    // if (ldlt.info() == Eigen::Success) {
      alpha = ldlt.solve(rhs);
    // } else {
    //     alpha = lhs.colPivHouseholderQr().solve(rhs);
    // }
    
    VectorXd gridBasis = monomials3D(gridPt.x(), gridPt.y(), gridPt.z(), exponents);

    // Store the result
    MLSValues(i) = gridBasis.dot(alpha);

  }
  return MLSValues;
}



#endif
