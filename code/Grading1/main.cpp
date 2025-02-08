#include "readNOFF.h"
#include "compute_scalar_mls.h"
#include "serialization.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <random>
#include <chrono>
#include <filesystem>

using namespace Eigen;
using namespace std;


double tolerance = 1e-3;

namespace fs = std::filesystem;

MatrixXd grid_locations(const int gridRes,
                        const RowVector3d boundMin,
                        const RowVector3d boundMax){
  
  MatrixXd sampledLocations(gridRes*gridRes*gridRes, 3);
  RowVector3d span = (boundMax-boundMin)/(double)(gridRes-1);
  for (int k=0;k<gridRes;k++)
    for (int j=0;j<gridRes;j++)
      for (int i=0;i<gridRes;i++)
        sampledLocations.row(gridRes*gridRes*k + gridRes*j+i) = boundMin+RowVector3d{(double)i*span(0),(double)j*span(1), (double)k*span(2)};
  
  return sampledLocations;
}

bool compare_nan_vectors(const VectorXd& vec1,
                         const VectorXd& vec2) {
  
  if (vec1.size() != vec2.size()) {
    cout<<"Vector sizes are not the same!"<<endl;
    return false;
  }
  
  double maxDifference = 0.0;
  int maxDifferenceIndex = -1;
  
  for (int i = 0; i < vec1.size(); ++i) {
    bool isNanVec1 = std::isnan(vec1[i]);
    bool isNanVec2 = std::isnan(vec2[i]);
    
    if (isNanVec1 != isNanVec2) {
      cout<<"Location "<<i<<": GT is "<<vec1[i]<<", your result is "<<vec2[i]<<endl;
      return false;
    } else if (!isNanVec1 && !isNanVec2) {
      double difference = std::abs(vec1[i] - vec2[i]);
      if (difference > maxDifference) {
        maxDifference = difference;
        maxDifferenceIndex = i;
      }
    }
  }
  
  if (maxDifference > tolerance) {
    cout<<"Location "<<maxDifferenceIndex<<": GT is "<<vec1[maxDifferenceIndex]<<", your result is "<<vec2[maxDifferenceIndex]<<endl;
    return false;
  }
  
  return true;
}


int main()
{
  
  RowVector2d epsValues(0.005, 0.05);
  RowVector3i NValues(0,1,2);
  RowVector3d hValues(0.01, 0.05, 0.2);
  RowVector2i gridRes(8, 16);
  double section1Points = 30.0;
  
  std::string folderPath(DATA_PATH); // Replace with your folder path
  int pointGain=0, pointSum=0;
  for (const auto& entry : fs::directory_iterator(folderPath)) {
    if (entry.is_regular_file() && entry.path().extension() == ".off") {
      cout<<"Working on file "<<entry.path().filename()<<endl;
      std::string dataName = entry.path().string();
      dataName.erase(dataName.size() - 4, 4);
      std::ifstream ifs(dataName+"-section1.data", std::ofstream::binary);
      
      MatrixXd pointCloud, pcNormals;
      MatrixXi stubF;
      
      readNOFF(entry.path().string(), pointCloud, pcNormals, stubF);
      double diagLength = (pointCloud.colwise().maxCoeff() - pointCloud.colwise().minCoeff()).norm();
      RowVector3d boundMin = pointCloud.colwise().minCoeff()- 0.2*(pointCloud.colwise().maxCoeff() - pointCloud.colwise().minCoeff());
      RowVector3d boundMax = pointCloud.colwise().maxCoeff()+ 0.2*(pointCloud.colwise().maxCoeff() - pointCloud.colwise().minCoeff());
      
      for (int i=0;i<2;i++){
        for (int j=0;j<3;j++){
          for (int k=0;k<3;k++){
            for (int l=0;l<2;l++){
              cout<<"Testing combination eps = "<<epsValues(i)<<", N = "<<NValues(j)<<", h = "<<hValues(k)<<", gridRes = "<<gridRes(l)<<endl;
              MatrixXd sampledLocations = grid_locations(gridRes(l), boundMin, boundMax);
              auto start = std::chrono::high_resolution_clock::now();
              VectorXd MLSValues = compute_scalar_mls(sampledLocations, pointCloud,  pcNormals, NValues(j), hValues(k), epsValues(i));
              VectorXd MLSValuesGT;
              auto end = std::chrono::high_resolution_clock::now();
              auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
              std::cout << "Function took " << (double)(duration.count())/1000.0 << " seconds to execute." << std::endl;
              VectorXd durVectorGT;
              deserializeVector(MLSValuesGT, ifs);
              deserializeVector(durVectorGT, ifs);
              pointSum++;
              
              //comparing vectors
              if ((durVectorGT(0)*10.0 < (double)(duration.count())/1000.0)&&(durVectorGT(0)>1000.0)){
                cout<<"Running took too long! "<<endl;
                continue;  //No points gained
              }
              
              if (compare_nan_vectors(MLSValuesGT, MLSValues)){
                cout<<"Result is good!"<<endl;
                pointGain++;
              }
              
            }
          }
        }
      }
    }
  }
  cout<<"Total point gained: "<<pointGain<<"/"<<pointSum<<endl;
  cout<<"Grade for Section 1: "<<round((double)pointGain*section1Points/(double)pointSum)<<endl;
  
}

