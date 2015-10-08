
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>

// This function displays the help
void showHelp(char *program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h: Show this help." << std::endl; 
}


int main(int argc, char** argv)
{
    // Show help
    if(pcl::console::find_switch(argc,argv,"-h") || pcl::console::find_switch(argc,argv,"--help")){
        showHelp(argv[0]);
        return 0; 
    }
	
    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    std::vector<int> filenames;
    bool file_is_pcd = false;
	
    filenames = pcl::console::parse_file_extension_argument(argc,argv,".ply");

    if(filenames.size() != 1){
        filenames = pcl::console::parse_file_extension_argument(argc,argv,".pcd");

        if(filenames.size() != 1)
        {
            showHelp(argv[0]);
            return -1;
        } else{
            file_is_pcd = true;
        }
    }
	

    // Load file | Works with PCD and PLY files
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    if(file_is_pcd){
        if(pcl::io::loadPCDFile(argv[filenames[0]],*cloud) < 0){
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
            showHelp(argv[0]);
            return -1;
        } 
    } else {
        if(pcl::io::loadPLYFile(argv[filenames[0]],*cloud) < 0){
            std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
            showHelp (argv[0]);
            return -1;
        }
    }

	// Statistical filter object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statisticalFilter;
	statisticalFilter.setInputCloud(cloud);

	// Every point must have 10 neighbors within 15cm, or it will be removed
    int meanK = 50;
    meanK = atoi(argv[2]);
    std::cout << "Loading filter meanK value = " << meanK << std::endl;
    statisticalFilter.setMeanK( meanK );

    float stdDevMulThresh = 1.0;
    stdDevMulThresh = atof( argv[3] );
    std::cout << "Loading filter stdDevMulThresh value = " << stdDevMulThresh << std::endl;
    statisticalFilter.setStddevMulThresh( stdDevMulThresh );
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    statisticalFilter.filter(*filteredCloud);


    // Smoothing and normal estimation based on polynomial reconstruction
    // http://pointclouds.org/documentation/tutorials/resampling.php#moving-least-squares
    //  attempts to recreate the missing parts of the surface by higher order polynomial interpolations 
    // Init object
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(filteredCloud);
    mls.setPolynomialFit(true);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    mls.setSearchMethod(tree);

    float kdtreeRadius = 0.4;
    kdtreeRadius = atof(argv[4]);
    std::cout << "Loading kdtree radius value = " << kdtreeRadius << std::endl;
    mls.setSearchRadius(kdtreeRadius);

    // Reconstruct
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    mls.process(*outputCloud);


    // Visualize the output point cloud
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Smoothed cloud"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(outputCloud);
    viewer->addPointCloud(outputCloud,rgb,"smoothed cloud");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return 1;
}

