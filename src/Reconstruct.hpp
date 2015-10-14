#ifndef _3D_RECONSTRUCTION_HPP_
#define _3D_RECONSTRUCTION_HPP_

// ---------------------------------------------------------------------------
// Author: Thomio Watanabe
// Date: October 2015
// Description: PCL class implemented to easy 3d models reconstruction.
// ---------------------------------------------------------------------------

#include <pcl/visualization/pcl_visualizer.h>


namespace reconstruction{
    class Reconstruct{
        public:
            typedef std::vector< std::pair<std::string,pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > CloudContainer;

            Reconstruct();

            void showHelp(char *programName);

            void openPCL(int argc, char** argv,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud);

            void statisticalFilter(
                    int meanK,
                    float stdDevMulThresh,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud);

            void polynomialInterpolation(
                    float kdtreeRadius,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothedCloud);

            void visualizeClouds(CloudContainer outputClouds);
    };
}

#endif
