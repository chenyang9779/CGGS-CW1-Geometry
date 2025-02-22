#ifndef COMPUTE_SCALAR_MLS_H
#define COMPUTE_SCALAR_MLS_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <nanoflann.hpp>

double nanValue = std::numeric_limits<double>::quiet_NaN();

inline std::vector<std::array<int, 3>> generateEponents(int N)
{
  std::vector<std::array<int, 3>> exponents;
  for (int i = 0; i <= N; i++)
  {
    for (int j = 0; j <= N; j++)
    {
      for (int k = 0; k <= N; k++)
      {
        if (i + j + k <= N)
          exponents.push_back({i, j, k});
      }
    }
  }
  return exponents;
}

inline double wendlandWeight(double r, double h)
{
  double s = 1.0 - r / h;
  return s * s * s * s * (4.0 * r / h + 1.0);
}

inline Eigen::VectorXd monomials3D(double x, double y, double z, const std::vector<std::array<int, 3>> &exponents)
{
  int nTerm = exponents.size();
  Eigen::VectorXd basis(nTerm);
  for (int i = 0; i < nTerm; i++)
  {
    basis(i) = std::pow(x, exponents[i][0]) *
               std::pow(y, exponents[i][1]) *
               std::pow(z, exponents[i][2]);
  }
  return basis;
}

struct SampleCloud
{
  const Eigen::MatrixXd &pts;
  SampleCloud(const Eigen::MatrixXd &pts_) : pts(pts_) {}
  inline size_t kdtree_get_point_count() const { return pts.rows(); }
  inline double kdtree_get_pt(const size_t idx, int dim) const { return pts(idx, dim); }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX & /*bb*/) const { return false; }
};

Eigen::VectorXd compute_scalar_mls(const Eigen::MatrixXd &gridLocations,
                                   const Eigen::MatrixXd &pointCloud,
                                   const Eigen::MatrixXd &pointNormals,
                                   const int N,
                                   const double h,
                                   const double epsNormal)
{
  using namespace Eigen;
  using namespace std;
  using namespace nanoflann;

  // stub function to delete: just a sphere
  //  RowVector3d mean = gridLocations.colwise().mean();
  //  VectorXd MLSValues(gridLocations.rows());
  //  for (int i=0;i<gridLocations.rows();i++)
  //    MLSValues(i) = (gridLocations.row(i)-mean).norm();
  //  MLSValues.array()-=MLSValues.mean();

  // TODO: replace with the scalar MLS computation.
  int nGrid = gridLocations.rows();
  VectorXd MLSValues(nGrid);

  std::vector<std::array<int, 3>> exponents = generateEponents(N);
  int nBasis = exponents.size();

  int nPoints = pointCloud.rows();
  int nSamples = 3 * nPoints;
  MatrixXd samples(nSamples, 3);
  VectorXd sampleValues(nSamples);
  for (int i = 0; i < nPoints; i++)
  {
    samples.row(3 * i) = pointCloud.row(i);
    sampleValues(3 * i) = 0.0;
    samples.row(3 * i + 1) = pointCloud.row(i) + epsNormal * pointNormals.row(i);
    sampleValues(3 * i + 1) = epsNormal;
    samples.row(3 * i + 2) = pointCloud.row(i) - epsNormal * pointNormals.row(i);
    sampleValues(3 * i + 2) = -epsNormal;
  }

  SampleCloud sampleCloud(samples);
  typedef KDTreeSingleIndexAdaptor<
      L2_Simple_Adaptor<double, SampleCloud>,
      SampleCloud,
      3>
      my_kd_tree_t;

  my_kd_tree_t kdTree(3, sampleCloud, KDTreeSingleIndexAdaptorParams(10));
  kdTree.buildIndex();

  for (int i = 0; i < nGrid; i++)
  {
    RowVector3d gridPt = gridLocations.row(i);
    double query_pt[3] = {gridPt(0), gridPt(1), gridPt(2)};

    std::vector<nanoflann::ResultItem<unsigned int, double>> matches;
    nanoflann::SearchParameters params;
    const double search_radius = h * h;
    size_t nMatches = kdTree.radiusSearch(&query_pt[0], search_radius, matches, params);

    vector<RowVector3d> localPoints;
    vector<double> localVals;
    vector<double> localWeights;
    for (size_t m = 0; m < nMatches; m++)
    {
      size_t idx = matches[m].first;
      double dist2 = matches[m].second;
      double r = std::sqrt(dist2);
      if (r <= h)
      {
        localPoints.push_back(samples.row(idx));
        localVals.push_back(sampleValues(idx));
        double weight = wendlandWeight(r, h);
        localWeights.push_back(weight);
      }
    }
    int nLocal = localPoints.size();
    if (nLocal < nBasis)
    {
      // std::cout << "Grid point " << i << " has too few local points (" << nLocal << "), setting NaN." << std::endl;
      MLSValues(i) = nanValue;
      continue;
    }

    MatrixXd A(nLocal, nBasis);
    VectorXd b(nLocal);

    for (int r = 0; r < nLocal; r++)
    {
      double sqrtWeight = std::sqrt(localWeights[r]);
      A.row(r) = monomials3D(localPoints[r][0], localPoints[r][1], localPoints[r][2], exponents) * sqrtWeight;
      b(r) = localVals[r] * sqrtWeight;
    }

    MatrixXd lhs = A.transpose() * A;
    VectorXd rhs = A.transpose() * b;

    VectorXd alpha;
    JacobiSVD<MatrixXd> svd(lhs, ComputeThinU | ComputeThinV);
    alpha = svd.solve(rhs);

    VectorXd gridBasis = monomials3D(gridPt(0), gridPt(1), gridPt(2), exponents);

    // Store the result
    MLSValues(i) = gridBasis.dot(alpha);
  }
  return MLSValues;
}

#endif