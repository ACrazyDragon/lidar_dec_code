/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar *lidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud = lidar->scan();
    //renderRays(viewer,lidar->position,PointCloud);
    renderPointCloud(viewer,PointCloud, "PointCloud",Color(1,1,1));

    // TODO:: Create point processor
    //ProcessPointClouds *ProcessPoint = new ProcessPointClouds(); ??????????????????????????
    ProcessPointClouds<pcl::PointXYZ> PointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = PointProcessor.SegmentPlane(PointCloud, 100, 2);
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
   //clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = PointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        PointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        //render box;
        Box box = PointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
  
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI>* PointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
   // ProcessPointClouds<pcl::PointXYZI> *PointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = PointProcessorI -> loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
   //renderPointCloud(viewer,inputCloud,"inputCloud");
  //renderPointCloud(viewer,inputCloud,"inputCloud");
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = PointProcessorI->FilterCloud(inputCloud,0.3,Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f( 30, 6, 0.5, 1));

   std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = PointProcessorI->SegmentPlane(cloudFiltered, 50, 0.3);
   renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    //clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = PointProcessorI->Clustering(segmentCloud.first, 0.53, 10, 500);
    renderPointCloud(viewer,cloudClusters[2],"obstCloud",Color(1,0,0));
    //std::cout<<"cloud size: "<<cloudClusters.size()<<std::endl;


    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
 
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        PointProcessorI->numPoints(cluster);
        //renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        std::cout<<"ClusterId: " << clusterId<<std::endl;
        //render box;
        Box box = PointProcessorI->BoundingBox(cluster);
       renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    
  
}

 /* 
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* PointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud){
     //renderPointCloud(viewer,inputCloud,"inputCloud");
     pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = PointProcessorI->FilterCloud(inputCloud,0.3,Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f( 30, 6, 0.5, 1));
     renderPointCloud(viewer,cloudFiltered, "Filtered");

    
     std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = PointProcessorI->SegmentPlane(cloudFiltered, 50, 0.3);
     KdTree* tree = new KdTree;
     for(int i=0;i<segmentCloud.first->points.size();i++){
         tree->insert(segmentCloud.first->points[i],i); }

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = PointProcessorI -> euclideanCluster(segmentCloud.first, tree, 0.5,30,250);
    //renderPointCloud(viewer,segmentCloud.second, "planefield", Color(0,1,1));
     renderPointCloud(viewer,segmentCloud.second, "planefield", Color(0,1,0));

    // renderPointCloud(viewer,segmentCloud.first, "obsfield", Color(1,1,0));
    renderPointCloud(viewer,segmentCloud.first, "obsfield", Color(1,0,0));
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
    std::cout << "cluster size ";
    PointProcessorI->numPoints(cluster);
    renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
    
    Box box = PointProcessorI->BoundingBox(cluster);
    renderBox(viewer,box,clusterId);
    ++clusterId;
  }

}*/


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    //Stream PCD
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    while (!viewer->wasStopped ()){

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        renderPointCloud(viewer,inputCloudI,"inputCloud");

        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
        }




     //cityBlock(viewer,pointProcessorI);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }

     
}