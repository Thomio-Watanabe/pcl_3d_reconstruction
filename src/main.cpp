
#include "Reconstruct.hpp"

// This 3d reconstruction implementation is intended to reconstruct 3d models
// with point clouds generated in underwater environments.

using namespace pcl;
using namespace std;

int main(int argc, char** argv)
{
    reconstruction::Reconstruct underwaterReconstruction;

    // Open Point Cloud file
    PointCloud<PointXYZRGB>::Ptr sourceCloud(new PointCloud<PointXYZRGB>);
    underwaterReconstruction.openPCL(argc,argv,sourceCloud);

    // Filtering statistically
    int meanK = atoi(argv[2]);
    float stdDevMulThresh = atof(argv[3]);
    PointCloud<PointXYZRGB>::Ptr filteredCloud(new PointCloud<PointXYZRGB>);
    underwaterReconstruction.statisticalFilter(meanK,stdDevMulThresh,sourceCloud,filteredCloud);

    // Smoothing and normal estimation based on polynomial
    float kdtreeRadius = atof(argv[4]);
    PointCloud<PointXYZRGB>::Ptr smoothedCloud(new PointCloud<PointXYZRGB>);
    underwaterReconstruction.polynomialInterpolation(kdtreeRadius,filteredCloud,smoothedCloud);

    // Visualize clouds
    reconstruction::Reconstruct::CloudContainer outputClouds;
    outputClouds.push_back( make_pair(string("Source Cloud"),sourceCloud) );
    outputClouds.push_back( make_pair(string("Filtered Cloud"),filteredCloud) );
    outputClouds.push_back( make_pair(string("Smoothed Cloud"),smoothedCloud) );
    underwaterReconstruction.visualizeClouds(outputClouds);

    return 0;
}
