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
        private:
            // Source RGB point cloud
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud;

        public:
            Reconstruct();

            void showHelp(char *programName);

            void openPCL(int argc, char** argv);

            void statisticalFilter(
                    int meanK,
                    float stdDevMulThresh,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud);

            void polynomialInterpolation(
                    float kdtreeRadius,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothedCloud);

            void visualizeCloud(
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud,
                    std::string windowTitle);
    };
}


#endif
