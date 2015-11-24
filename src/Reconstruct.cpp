#include "Reconstruct.hpp"

#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>

using namespace pcl;
using namespace std;
using namespace reconstruction;

Reconstruct::Reconstruct()
{}


void Reconstruct::showHelp(char *programName){
    cout << "-- Reconstruct: usage: " << programName << " cloud_filename.[pcd|ply]" << endl;
    cout << "-- Reconstruct: exit program. " << endl;
    exit(EXIT_FAILURE);
}


void Reconstruct::openPCL(int argc, char** argv, PointCloud<PointXYZRGB>::Ptr sourceCloud){
    // Fetch point cloud filename in arguments | Works with PCD and PLY files
    vector<int> filenames;
    bool file_is_pcd = false;
	
    filenames = console::parse_file_extension_argument(argc,argv,".ply");

    if(filenames.size() != 1){
        filenames = console::parse_file_extension_argument(argc,argv,".pcd");

        if(filenames.size() != 1)
        {
            showHelp(argv[0]);
        } else{
            file_is_pcd = true;
        }
    }
	
    // Load file | Works with PCD and PLY files
    if(file_is_pcd){
        if(io::loadPCDFile(argv[filenames[0]],*sourceCloud) < 0){
            cout << "-- Reconstruct: error loading point cloud " << argv[filenames[0]] << endl << endl;
            showHelp(argv[0]);
        } 
    }else {
        if(io::loadPLYFile(argv[filenames[0]],*sourceCloud) < 0){
            cout << "-- Reconstruct: error loading point cloud " << argv[filenames[0]] << endl << endl;
            showHelp (argv[0]);
        }
    }

    cout << "-- Reconstruct: point cloud " << argv[filenames[0]] << " successfully loaded" << endl;
}


void Reconstruct::savePCL(string pclName, PointCloud<PointXYZRGB>::Ptr outputCloud){
        if ( pclName.find(".ply") ){
            io::savePLYFileBinary(pclName,*outputCloud);
        } else if ( pclName.find(".pcd") ){
            io::savePCDFileBinary(pclName,*outputCloud);
        } else{
            cout << "-- Reconstruct: FAIL --> Output format is not .ply or .pcd" << endl;
        }
        
}


void Reconstruct::statisticalFilter(int meanK,float stdDevMulThresh,
    PointCloud<PointXYZRGB>::Ptr sourceCloud,
    PointCloud<PointXYZRGB>::Ptr filteredCloud)
{
    cout << "-- Reconstruct: running statistical filter. "<< endl;

	// Statistical filter object
	StatisticalOutlierRemoval<PointXYZRGB> statisticalFilter;
	statisticalFilter.setInputCloud(sourceCloud);

	// Every point must have 10 neighbors within 15cm, or it will be removed
    cout << "-- Reconstruct: loading filter meanK value = " << meanK << endl;
    statisticalFilter.setMeanK( meanK );

    cout << "-- Reconstruct: loading filter stdDevMulThresh value = " << stdDevMulThresh << endl;
    statisticalFilter.setStddevMulThresh( stdDevMulThresh );
    statisticalFilter.filter(*filteredCloud);
}


void Reconstruct::polynomialInterpolation(float kdtreeRadius,
            PointCloud<PointXYZRGB>::Ptr filteredCloud,
            PointCloud<PointXYZRGB>::Ptr smoothedCloud){

    cout << "-- Reconstruct: running polynomial interpolation. "<< endl;

    // http://pointclouds.org/documentation/tutorials/resampling.php#moving-least-squares
    // attempts to recreate the missing parts of the surface by higher order polynomial interpolations 
    // Init object
    MovingLeastSquares<PointXYZRGB, PointXYZRGB> mls;
    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(filteredCloud);
    mls.setPolynomialFit(true);
    int polynomialOrder = 2;
    cout << "-- Reconstruct: set polynomial order = " << polynomialOrder << endl;
    mls.setPolynomialOrder(polynomialOrder);


    // SAMPLE_LOCAL_PLANE   RANDOM_UNIFORM_DENSITY VOXEL_GRID_DILATION
//    mls.setUpsamplingMethod(MovingLeastSquares<PointXYZRGB, PointXYZRGB>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingMethod(MovingLeastSquares<PointXYZRGB, PointXYZRGB>::RANDOM_UNIFORM_DENSITY);
    float upsamplingRadius = 0.10;
    float upsamplingStepSize = 0.03;
    cout << "-- Reconstruct: set upsampling radius = " << upsamplingRadius << endl;
    cout << "-- Reconstruct: set upsampling step size = " << upsamplingStepSize << endl;
    mls.setUpsamplingRadius(upsamplingRadius);
    mls.setUpsamplingStepSize(upsamplingStepSize);
    // add multiple threads ??

    search::KdTree<PointXYZRGB>::Ptr tree (new search::KdTree<PointXYZRGB>);
    mls.setSearchMethod(tree);

    cout << "-- Reconstruct: loading kdtree radius value = " << kdtreeRadius << endl;
    mls.setSearchRadius(kdtreeRadius);

    mls.process(*smoothedCloud);
}

void Reconstruct::visualizeClouds(CloudContainer outputClouds)
{
   // Visualize the output point cloud
    cout << "-- Reconstruct: rendering point clouds. " << endl;
    cout << "-- Reconstruct: press \"q\" to quit. " << endl;

    typedef boost::shared_ptr<visualization::PCLVisualizer> PCLVisualizer;
    vector<PCLVisualizer> visualizerContainer;
    PCLVisualizer viewer;

    for(CloudContainer::iterator it =  outputClouds.begin(); it != outputClouds.end(); ++it ){
        viewer = boost::shared_ptr<visualization::PCLVisualizer> (new visualization::PCLVisualizer( it->first ));
        visualization::PointCloudColorHandlerRGBField<PointXYZRGB> rgb( it->second );
        viewer->addPointCloud( it->second, rgb, it->first);
        visualizerContainer.push_back(viewer);
    }

    bool visualize = true;
    while(visualize){
        for(vector<PCLVisualizer>::iterator it =  visualizerContainer.begin(); it != visualizerContainer.end(); ++it ){
            (*it)->spinOnce(100);       // Calls the interactor and updates the screen once.
            if( (*it)->wasStopped() )
                visualize = false;
        }
//        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}



