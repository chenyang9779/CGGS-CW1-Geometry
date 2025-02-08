#ifndef COMPUTE_SCALAR_MLS_H
#define COMPUTE_SCALAR_MLS_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/QR> 
#include <Eigen/SVD>

double nanValue = std::numeric_limits<double>::quiet_NaN();

inline std::vector<std::array<int,3>> generateEponents(int N){
  std::vector<std::array<int, 3>> exponents;
  // for (int i = 0; i <= N; i++) {
  //   for (int j = 0; j <= N - i; j++) {
  //     for (int k = 0; k <= N - i - j; k++) {
  //       exponents.push_back({i, j, k});
  //     }
  //   }
  // }
  for (int i = 0; i <= N; i++) {
    for (int j = 0; j <= N; j++) {
      for (int k = 0; k <= N ; k++) {
        if (i + j +k <= N) exponents.push_back({i, j, k});
      }
    }
  }
  return exponents;
}
inline double wendlandWeight(double r, double h) {
  double s = 1.0 - r / h;
  return s*s*s*s*(4.0 * r / h + 1.0);
}

inline double gaussianWeight(double r, double h) {
  return std::exp(- r * r / (h * h));
}

inline double singularEdgeWeight(double r, double epsilon) {
  return 1.0 / (r * r + epsilon * epsilon);
}




inline Eigen::VectorXd monomials3D(double x, double y, double z, const std::vector<std::array<int, 3>> &exponents, double scale){
  int nTerm = exponents.size();
  Eigen::VectorXd basis(nTerm);
  double x_scaled = x / (scale*scale);
  double y_scaled = y / (scale*scale);
  double z_scaled = z / (scale*scale);

  for (int i = 0; i < nTerm; i++){
    basis(i) = std::pow(x, exponents[i][0]) * 
               std::pow(y, exponents[i][1]) * 
               std::pow(z, exponents[i][2]);
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
        // double weight = gaussianWeight(std::sqrt(dist2), h);
        // double weight = singularEdgeWeight(std::sqrt(dist2), 0.0);
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
        // double weight = gaussianWeight(std::sqrt(dist2), h);
        // double weight = singularEdgeWeight(std::sqrt(dist2), epsNormal);
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
        // double weight = gaussianWeight(std::sqrt(dist2), h);
        // double weight = singularEdgeWeight(std::sqrt(dist2), epsNormal);
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
      RowVector3d pt_centered = localPoints[r] - gridPt;
      A.row(r) = monomials3D(localPoints[r][0], localPoints[r][1], localPoints[r][2], exponents, h) * sqrtWeight;
      b(r) = localValues[r] * sqrtWeight;
    }


    MatrixXd lhs = A.transpose() * A;
    VectorXd rhs = A.transpose() * b;
    
    VectorXd alpha;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(lhs, Eigen::ComputeThinU | Eigen::ComputeThinV);
    alpha = svd.solve(rhs);

    // Eigen::LDLT<MatrixXd> ldlt(lhs);
    // alpha = ldlt.solve(rhs);

    
    VectorXd gridBasis = monomials3D(gridPt[0], gridPt[1], gridPt[2], exponents, h);
    // if (i == 26090){
    //   cout << A <<endl;
    //   cout << rhs << endl;
    //   cout << alpha << endl;
    //   cout << gridBasis <<endl;
    //   // printf("%.9f\n", lhs);
    //   // printf("%.9f\n", rhs);
    // }
    // Store the result
    MLSValues(i) = gridBasis.dot(alpha);

  }
  return MLSValues;
}



#endif
