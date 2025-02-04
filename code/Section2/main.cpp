#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>
#include <polyscope/volume_grid.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <vector>
#include <set>
#include <array>
#include <queue>
#include <chrono>
#include "readNOFF.h"
#include "compute_function_mls.h"

using namespace Eigen;
using namespace std;

//The mesh quantities
MatrixXd pointCloud, pcNormals;
VectorXd MLSValues;

int gridRes = 32;
double h = 0.05;
double diagLength;

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


inline glm::vec3 eigen2glm(const RowVector3d& v){ return glm::vec3{v(0), v(1), v(2)};}


int main()
{
  MatrixXi stubF;
  readNOFF(DATA_PATH "/fertility-2500.off",pointCloud, pcNormals, stubF);
  diagLength = (pointCloud.colwise().maxCoeff() - pointCloud.colwise().minCoeff()).norm();
  
  polyscope::init();
  polyscope::options::warnForInvalidValues = false;
  polyscope::registerPointCloud("Original Point Cloud", pointCloud);
  RowVector3d boundMin = pointCloud.colwise().minCoeff()- 0.0*(pointCloud.colwise().maxCoeff() - pointCloud.colwise().minCoeff());
  RowVector3d boundMax = pointCloud.colwise().maxCoeff()+ 0.0*(pointCloud.colwise().maxCoeff() - pointCloud.colwise().minCoeff());
  
  // register the grid
  polyscope::VolumeGrid* psGrid = polyscope::registerVolumeGrid(
                                                                "Implicit Function", {gridRes, gridRes, gridRes}, eigen2glm(boundMin), eigen2glm(boundMax));

  
  MatrixXd sampledLocations = grid_locations(gridRes, boundMin, boundMax);
  
  auto start = std::chrono::high_resolution_clock::now();
  MLSValues = compute_function_mls(sampledLocations, pointCloud, pcNormals, h*diagLength);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "Function took " << (double)(duration.count())/1000.0 << " seconds to execute." << std::endl;
 
  cout<<"done! "<<endl;

  std::vector<float> MLSValuesArray(MLSValues.size());
  for (int i=0;i<MLSValues.size();i++) MLSValuesArray[i] = MLSValues(i);
  
  polyscope::VolumeGridNodeScalarQuantity* psScalarValues =
  psGrid->addNodeScalarQuantity("MLS Values",MLSValuesArray);
  psScalarValues->setEnabled(true);
  
  polyscope::show();
  
  
}

