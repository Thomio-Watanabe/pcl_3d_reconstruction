
#include "Reconstruct.hpp"

// This 3d reconstruction implementation is intended to reconstruct 3d models
// with point clouds generated in underwater environments.

using namespace pcl;
using namespace std;

int main(int argc, char** argv)
{
    reconstruction::Reconstruct underwaterReconstruction;

    // Open Point Cloud file
    underwaterReconstruction.openPCL(argc,argv);

    // Filtering statistically
    int meanK = atoi(argv[2]);
    float stdDevMulThresh = atof(argv[3]);
    PointCloud<PointXYZRGB>::Ptr filteredCloud(new PointCloud<PointXYZRGB>);
    underwaterReconstruction.statisticalFilter(meanK,stdDevMulThresh,filteredCloud);

    underwaterReconstruction.visualizeCloud(filteredCloud,string("Filtered Cloud"));


    // Smoothing and normal estimation based on polynomial
    float kdtreeRadius = atof(argv[4]);
    PointCloud<PointXYZRGB>::Ptr smoothedCloud(new PointCloud<PointXYZRGB>);
    underwaterReconstruction.polynomialInterpolation(kdtreeRadius,filteredCloud,smoothedCloud);

    underwaterReconstruction.visualizeCloud(smoothedCloud,string("Smoothed Cloud"));

    return 0;
}

