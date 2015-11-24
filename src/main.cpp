
#include "Reconstruct.hpp"

using namespace pcl;
using namespace std;

int main(int argc, char** argv)
{
    reconstruction::Reconstruct underwaterReconstruction;

    // Open Point Cloud file
    PointCloud<PointXYZRGB>::Ptr sourceCloud(new PointCloud<PointXYZRGB>);
    underwaterReconstruction.openPCL(argc,argv,sourceCloud);

    // Filtering statistically
    int meanK = 500;    // atoi(argv[2]);
    float stdDevMulThresh = 0.001;   // atof(argv[3]); % default = 0.05
    PointCloud<PointXYZRGB>::Ptr filteredCloud(new PointCloud<PointXYZRGB>);
    underwaterReconstruction.statisticalFilter(meanK,stdDevMulThresh,sourceCloud,filteredCloud);

    // Smoothing and normal estimation based on polynomial
    float kdtreeRadius = 1.0; // atof(argv[4]); % default = 1.0
    PointCloud<PointXYZRGB>::Ptr smoothedCloud(new PointCloud<PointXYZRGB>);
    underwaterReconstruction.polynomialInterpolation(kdtreeRadius,filteredCloud,smoothedCloud);

    // Save smoothed cloud
    underwaterReconstruction.savePCL("smoothedCloud.ply",smoothedCloud);

    // Visualize clouds
    reconstruction::Reconstruct::CloudContainer outputClouds;
    outputClouds.push_back( make_pair(string("Source Cloud"),sourceCloud) );
    outputClouds.push_back( make_pair(string("Filtered Cloud"),filteredCloud) );
    outputClouds.push_back( make_pair(string("Smoothed Cloud"),smoothedCloud) );
    underwaterReconstruction.visualizeClouds(outputClouds);

    return 0;
}
