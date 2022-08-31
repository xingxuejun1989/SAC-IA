#include <vector>
#include <Eigen/Core>
#include <pcl/memory.h>

#include <pcl/common/common.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>


int main(int argc, char** argv)
{

    std::string source_filename = "G:/paper/testdata/1_Project1_scan_3.ply";
    std::string target_filename = "G:/paper/testdata/1_Project1_scan_4.ply";
    std::string save_filename = "G:/paper/testdata/1_Project1_scan_3-1.ply";
    double featureRadius = 0.05;
    double downsample_distance = 0.01;
    double MinSampleDistance = 0.1;
    double MaxCorrespondenceDistance = 0.1;
    int MaxIteration = 1000;
    double FitScore = 0.05;

    /*******************************************************************************************************************
     *Enter a point cloud with normals
     ******************************************************************************************************************/
    if (source_filename.empty())
    {
        std::cout << "The input source point cloud filename is wrong." << std::endl;
        system("pause");
        return -1;
    }

    if (target_filename.empty())
    {
        std::cout << "The input target point cloud filename is wrong." << std::endl;
        system("pause");
        return -1;
    }
    
    pcl::PointCloud<pcl::PointNormal>::Ptr source_clouds(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr target_clouds(new pcl::PointCloud<pcl::PointNormal>);


    //Read point cloud, file type is ply.
    if (pcl::io::loadPLYFile<pcl::PointNormal>(source_filename, *source_clouds) == -1)
    {
        std::cout << "Read file xyz fail" << std::endl;
        system("pause");
        return - 1;
    }
    
    size_t n=source_clouds->points.size();
    if (n < 5)
    {
        std::cout << "Read file normal fail" << std::endl;
        system("pause");
        return -1;
    }
    std::cout << "Example of output source point cloud data." << std::endl;
    for (int i = 0; i < 5; i++)
    {
        std::cout << source_clouds->points[i].x << " "
            << source_clouds->points[i].y << " "
            << source_clouds->points[i].z << " ";
        std::cout << source_clouds->points[i].normal_x << " "
            << source_clouds->points[i].normal_y << " "
            << source_clouds->points[i].normal_z << " "<< std::endl;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr source_clouds_t(new pcl::PointCloud<pcl::PointNormal>);

    pcl::copyPointCloud(*source_clouds, *source_clouds_t);

    if (pcl::io::loadPLYFile<pcl::PointNormal>(target_filename, *target_clouds) == -1)
    {
        std::cout << "Read file xyz fail" << std::endl;
        system("pause");
        return -1;
    }
   

    n = target_clouds->points.size();
    if (n < 5)
    {
        std::cout << "Read file normal fail" << std::endl;
        system("pause");
        return -1;
    }
    std::cout << "Example of output target point cloud data." << std::endl;
    for (int i = 0; i < 5; i++)
    {
        std::cout << target_clouds->points[i].x << " "
            << target_clouds->points[i].y << " "
            << target_clouds->points[i].z << " ";
        std::cout << target_clouds->points[i].normal_x << " "
            << target_clouds->points[i].normal_y << " "
            << target_clouds->points[i].normal_z << " " << std::endl;
    }

    clock_t tick1 = clock();

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_xyz(new pcl::PointCloud<pcl::PointXYZ>);


    //Calculate the bounding box diameter.
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_bounding;
    pcl::copyPointCloud(*source_clouds, *point_xyz);
    feature_bounding.setInputCloud(point_xyz);
    feature_bounding.compute();

    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
   
    feature_bounding.getAABB(min_point_AABB, max_point_AABB);

    double diameter = sqrt((min_point_AABB.x - max_point_AABB.x) * (min_point_AABB.x - max_point_AABB.x)
        + (min_point_AABB.y - max_point_AABB.y) * (min_point_AABB.y - max_point_AABB.y)
        + (min_point_AABB.z - max_point_AABB.z) * (min_point_AABB.z - max_point_AABB.z));

    pcl::copyPointCloud(*target_clouds, *point_xyz);
    feature_bounding.setInputCloud(point_xyz);
    feature_bounding.compute();
    feature_bounding.getAABB(min_point_AABB, max_point_AABB);
    double diameter_t= sqrt((min_point_AABB.x - max_point_AABB.x) * (min_point_AABB.x - max_point_AABB.x)
        + (min_point_AABB.y - max_point_AABB.y) * (min_point_AABB.y - max_point_AABB.y)
        + (min_point_AABB.z - max_point_AABB.z) * (min_point_AABB.z - max_point_AABB.z));

   
    if (diameter < diameter_t)
        diameter = diameter_t;

    float downsample = diameter * downsample_distance;
    std::cout << source_clouds->points.size() << std::endl;
    pcl::VoxelGrid<pcl::PointNormal> vox_grid;
    vox_grid.setInputCloud(source_clouds);
    vox_grid.setLeafSize(downsample, downsample, downsample);
    pcl::PointCloud<pcl::PointNormal>::Ptr tempCloud(new pcl::PointCloud<pcl::PointNormal>);
    vox_grid.filter(*tempCloud);
    source_clouds = tempCloud;
    std::cout << source_clouds->points.size() << std::endl;

    std::cout << target_clouds->points.size() << std::endl;
    pcl::VoxelGrid<pcl::PointNormal> vox_grid1;
    vox_grid1.setInputCloud(target_clouds);
    vox_grid1.setLeafSize(downsample, downsample, downsample);
    pcl::PointCloud<pcl::PointNormal>::Ptr tempCloud2(new pcl::PointCloud<pcl::PointNormal>);
    vox_grid1.filter(*tempCloud2);
    target_clouds = tempCloud2;
    std::cout << target_clouds->points.size() << std::endl;
    //calculate features;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);

    
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> featureEstimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMothodPtr(new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr source_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr source_normal(new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normal(new pcl::PointCloud<pcl::Normal>);

    //Establishes the search radius for the feature.
    double feature_radius = featureRadius * diameter;

    pcl::copyPointCloud(*source_clouds, *source_xyz);
    pcl::copyPointCloud(*source_clouds, *source_normal);

    featureEstimation.setInputCloud(source_xyz);
    featureEstimation.setInputNormals(source_normal);
    featureEstimation.setSearchMethod(searchMothodPtr);
    featureEstimation.setRadiusSearch(feature_radius);
    featureEstimation.compute(*source_features);

    
    pcl::copyPointCloud(*target_clouds, *target_xyz);
    pcl::copyPointCloud(*target_clouds, *target_normal);
    featureEstimation.setInputCloud(target_xyz);
    featureEstimation.setInputNormals(target_normal);
    featureEstimation.setSearchMethod(searchMothodPtr);
    featureEstimation.setRadiusSearch(feature_radius);
    featureEstimation.compute(*target_features);


    // Define the registration algorithm
    pcl::SampleConsensusInitialAlignment<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> sacIA;

    sacIA.setMinSampleDistance(MinSampleDistance*diameter);
    sacIA.setMaxCorrespondenceDistance(MaxCorrespondenceDistance* diameter);
    sacIA.setMaximumIterations(MaxIteration);
   
    //Set up point clouds and features.
    sacIA.setInputTarget(target_clouds);
    sacIA.setTargetFeatures(target_features);

    sacIA.setInputSource(source_clouds);
    sacIA.setSourceFeatures(source_features);

    //Registration.
    pcl::PointCloud<pcl::PointNormal> registration_output;
    sacIA.align(registration_output);
    double score = sacIA.getFitnessScore(FitScore * diameter);

    clock_t tick2 = clock();
    
    //result.
    Eigen::Matrix4f pose = sacIA.getFinalTransformation();
    pcl::transformPointCloudWithNormals(*source_clouds_t, registration_output, pose);
    pcl::io::savePLYFileASCII(save_filename, registration_output);
    std::cout<<"time ="<<(double)(tick2- tick1)/ CLOCKS_PER_SEC<<std::endl
        <<"score = "<< score << std::endl << pose << std::endl;

    system("pause");
    return EXIT_SUCCESS;
}
