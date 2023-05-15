// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
//#include "quiz/cluster/kdtree.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*cloudFiltered);

   // interesting region
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

  //remove ego car roof points
     std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);




    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename  pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    for(int index : inliers-> indices)
        planeCloud -> points.push_back(cloud -> points[index]);
    //Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    //  TODO:: Fill in this function to find inliers for the cloud.
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    // pcl::SACSegmentation<PointT> seg;
    // seg.setOptimizeCoefficients (true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);
    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);
    // if(inliers->indices.size()==0){
    //     std::cout<<"could not find a plane model for the given dataset."<<std::endl;
    // }

//ransac3d
std::unordered_set<int> inliersResult;
srand(time(NULL));
while(maxIterations--){
        std::unordered_set<int> inliers;
        while(inliers.size()<3)
            inliers.insert(rand()%(cloud->points.size()));
        float x1, y1,z1, x2, y2,z2, x3, y3,z3;
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
        float i = (y2 - y1)*(z3 - z1) - (z2-z1)*(y3 - y1);
        float j = (z2 - z1)*(x3 - x1) - (x2-x1)*(z3 - z1);
        float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3-x1);
        float D = -(i*x1 + j*y1 + k*z1);
        for(int index=0;index< cloud->points.size();index++){
            if(inliers.count(index)>0)
                continue;
            PointT  point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;
            float d = fabs(i*x4 + j*y4 + k*z4 +D)/sqrt(i*i + j*j + k*k);
            if(d< distanceThreshold)
                inliers.insert(index);
        }
        if(inliers.size() > inliersResult.size()){
            inliersResult = inliers;
        }



	// std::unordered_set<int> inliersResult;
	// srand(time(NULL));
	// while(maxIterations--){
	// 	std::unordered_set<int> inliers;
	// 	while(inliers.size()<2)
	// 		inliers.insert(rand()%(cloud->points.size()));
	// 	float x1, y1, x2, y2;
	// 	auto itr = inliers.begin();
	// 	x1 = cloud->points[*itr].x;
	// 	y1 = cloud->points[*itr].y;
	// 	itr++;
	// 	x2 = cloud->points[*itr].x;
	// 	y2 = cloud->points[*itr].y;
	// 	float a = (y1 -y2);
	// 	float b = (x2 -x1);
	// 	float c = (x1*y2- x2*y1);
	// 	for(int index=0;index< cloud->points.size();index++){
	// 		if(inliers.count(index)>0)
	// 			continue;
	// 		PointT  point = cloud->points[index];
    //         // std::cout<<" : "<< cloud->points[index];
	// 		float x3 = point.x;
	// 		float y3 = point.y;
	// 		float d = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b);
	// 		if(d<=distanceThreshold)
	// 			inliers.insert(index);
	// 	}
	// 	if(inliers.size()>inliersResult.size()){
	// 		inliersResult = inliers;
	// 	}
     }


     pcl::PointIndices::Ptr inliers2(new pcl::PointIndices());
     for(int index=0; index< cloud->points.size();index++){
        //  PointT  point = cloud->points[index];
         if(inliersResult.count(index)){
             inliers2-> indices.push_back(index);
         }

     }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers2,cloud);
    return segResult;
}

 
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    for(pcl::PointIndices getIndices: cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(int index : getIndices.indices)
            cloudCluster-> points.push_back(cloud->points[index]);
        cloudCluster -> width = cloudCluster -> points.size();
        cloudCluster -> height = 1;
        cloudCluster -> is_dense = true;
        clusters.push_back(cloudCluster);
    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
/* 
template<typename PointT>
void clusterHelper(int indice,const std::vector<std::vector<float>>points, std::vector<int>& cluster, std::vector<bool> &processed, KdTree *tree, float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);
	std::vector<int> nearest = tree->search(points[indice],distanceTol);
	for(int id : nearest){
		if(!processed[id])
			clusterHelper(id,points,cluster, processed, tree, distanceTol);
	}
}

*/

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int idx, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[idx] = true;
	cluster.push_back(idx);
	std::vector<int> nearest = tree->search(cloud->points[idx], distanceTol);
	for(auto id : nearest)
	{
		if(!processed[id])
			clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
	}
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processed(cloud->points.size(),false);
    for(size_t idx=0;idx< cloud->points.size(); ++idx){
        if(processed[idx] == false){
            std::vector<int> cluster_idx;
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
            clusterHelper(idx,cloud, cluster_idx, processed, tree, distanceTol);
            if(cluster_idx.size() >= minSize && cluster_idx.size() <= maxSize){
                for(int i=0; i<cluster_idx.size(); i++){
                    PointT point;
                    point = cloud->points[cluster_idx[i]];
                    cloudCluster->points.push_back(point);
                }
                cloudCluster->width = cloudCluster->points.size();
                cloudCluster->height = 1;
                clusters.push_back(cloudCluster);
            }else{
                for(int i=1;i<cluster_idx.size();i++){
                    processed[cluster_idx[i]] = false;
                }
            }
        }
    }
    return clusters;
}


/* 
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(),false);

	int i=0;
	while(i < points.size()){
		if(processed[i]){
			i++;
			continue;
		}
		std::vector<int> cluster;
		clusterHelper(i,points,cluster,processed, tree,distanceTol);
		clusters.push_back(cluster);
		i++;
	}
 
	return clusters;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    KdTree* tree = new KdTree;
    for (int i=0; i<(cloud->points.size()); i++) 
    	tree->insert(cloud->points[i],i);
  	// std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, clusterTolerance);
      

    // tree->setInputCloud (cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);


    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    for(pcl::PointIndices getIndices: cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(int index : getIndices.indices)
            cloudCluster-> points.push_back(cloud->points[index]);
        cloudCluster -> width = cloudCluster -> points.size();
        cloudCluster -> height = 1;
        cloudCluster -> is_dense = true;
        clusters.push_back(cloudCluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
*/

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}