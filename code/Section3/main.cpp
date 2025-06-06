#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>
#include <polyscope/volume_grid.h>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>
#include <vector>
#include <chrono>
#include <set>
#include <array>
#include <queue>
#include "readNOFF.h"
#include "compute_scalar_mls_optimized.h"
#include <cstdlib>  // For std::stod and std::stoi


using namespace Eigen;
using namespace std;

//The mesh quantities
MatrixXd pointCloud, pcNormals;
VectorXd MLSValues;

// int gridRes = 32;
// double epsNormal = 0.01;
// int N = 2;
// double h = 0.1;
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


int main(int argc, char* argv[])
{   
    if (argc != 6) {
        cerr << "Usage: " << argv[0] << " <gridRes> <epsNormal> <N> <h> <fileName>\n";
        return 1;  // Exit with an error code
    }

    // Convert command-line arguments to proper data types
    int gridRes = stoi(argv[1]);
    double epsNormal = stod(argv[2]);
    int N = stoi(argv[3]);
    double h = stod(argv[4]);
    string file = argv[5];

    MatrixXi stubF;
    readNOFF(DATA_PATH "/" + file + ".off",pointCloud, pcNormals, stubF);
    diagLength = (pointCloud.colwise().maxCoeff() - pointCloud.colwise().minCoeff()).norm();
    
    polyscope::init();
    polyscope::options::warnForInvalidValues = false;
    auto* psCloud = polyscope::registerPointCloud("Original Point Cloud", pointCloud);
    psCloud->setPointRenderMode(polyscope::PointRenderMode::Quad);
    RowVector3d boundMin = pointCloud.colwise().minCoeff()- 0.0*(pointCloud.colwise().maxCoeff() - pointCloud.colwise().minCoeff());
    RowVector3d boundMax = pointCloud.colwise().maxCoeff()+ 0.0*(pointCloud.colwise().maxCoeff() - pointCloud.colwise().minCoeff());
    
    // register the grid
    polyscope::VolumeGrid* psGrid = polyscope::registerVolumeGrid(
                                                                  "Implicit Function", {gridRes, gridRes, gridRes}, eigen2glm(boundMin), eigen2glm(boundMax));

    
    MatrixXd sampledLocations = grid_locations(gridRes, boundMin, boundMax);
    
    auto start = std::chrono::high_resolution_clock::now();
    MLSValues = compute_scalar_mls(sampledLocations, pointCloud, pcNormals, N, h*diagLength, epsNormal*diagLength);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Function took " << (double)(duration.count())/1000.0 << " seconds to execute." << std::endl;
    cout << gridRes << "_" << epsNormal << "_" << N << "_" << h << "_" << (int)(duration.count())/1000.0 << endl;
    cout<<"done! "<<endl;

    std::vector<float> MLSValuesArray(MLSValues.size());
    for (int i=0;i<MLSValues.size();i++) MLSValuesArray[i] = MLSValues(i);
    
    polyscope::VolumeGridNodeScalarQuantity* psScalarValues = psGrid->addNodeScalarQuantity("MLS Values",MLSValuesArray);
    psScalarValues->setEnabled(true);
    // polyscope::registerSurfaceMesh();
    polyscope::show();
    
}

