#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

void calculateTemplate(_In_ const pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr,
    _In_ const float normalRadius,
    _In_ const float featureRadius,
    _Out_ pcl::PointCloud<pcl::FPFHSignature33>::Ptr& featurePtr)
{
    // 输出初始化
    pcl::PointCloud<pcl::Normal>::Ptr normalPtr(new pcl::PointCloud<pcl::Normal>);
    featurePtr = pcl::PointCloud<pcl::FPFHSignature33>::Ptr(new pcl::PointCloud<pcl::FPFHSignature33>);

    // 搜索方法
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMothodPtr(new pcl::search::KdTree<pcl::PointXYZ>);

    
    // 计算法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloudPtr);
    normalEstimation.setSearchMethod(searchMothodPtr);
    normalEstimation.setRadiusSearch(normalRadius);
    normalEstimation.compute(*normalPtr);

    // FPFH特征计算
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> featureEstimation;
    featureEstimation.setInputCloud(cloudPtr);
    featureEstimation.setInputNormals(normalPtr);
    featureEstimation.setSearchMethod(searchMothodPtr);
    featureEstimation.setRadiusSearch(featureRadius);
    featureEstimation.compute(*featurePtr);
}

void normalverification(_Out_ pcl::PointCloud<pcl::Normal>::Ptr& normal)
{
    size_t n=normal->points.size();
    for (int i = 0; i < n; i++)
    {
        float nx = normal->points[i].normal_x;
        float ny = normal->points[i].normal_y;
        float nz = normal->points[i].normal_z;

        float norm = nx * nx + ny * ny + nz * nz;
        if (norm < 0.01)
        {
            //std::cout << "normal 0." << std::endl;
            normal->points[i].normal_x = 1.0;
            normal->points[i].normal_y = 0.0;
            normal->points[i].normal_z = 0.0;
        }
    }
}
/***********************************************************************************************************************
 * 主函数调用
 **********************************************************************************************************************/
int main(int argc, char** argv)
{
    /*******************************************************************************************************************
     *Enter a point cloud with normals
     ******************************************************************************************************************/
    //
    std::string source_filename = "G:/paper/testdata/bun000.ply";
    std::string target_filename = "G:/paper/testdata/bun045.ply";
    double featureRadius = 0.005;
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
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_clouds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr source_normal(new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_clouds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr target_normal(new pcl::PointCloud<pcl::Normal>);

    //Read point cloud, file type is ply.
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(source_filename, *source_clouds) == -1)
    {
        std::cout << "Read file xyz fail" << std::endl;
        system("pause");
        return - 1;
    }
    if (pcl::io::loadPLYFile<pcl::Normal>(source_filename, *source_normal) == -1)
    {
        std::cout << "Read file normal fail" << std::endl;
        system("pause");
        return -1;
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
        std::cout << source_normal->points[i].normal_x << " "
            << source_normal->points[i].normal_y << " "
            << source_normal->points[i].normal_z << " "<< std::endl;
    }
   
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(target_filename, *target_clouds) == -1)
    {
        std::cout << "Read file xyz fail" << std::endl;
        system("pause");
        return -1;
    }
    if (pcl::io::loadPLYFile<pcl::Normal>(target_filename, *target_normal) == -1)
    {
        std::cout << "Read file normal fail" << std::endl;
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
        std::cout << target_normal->points[i].normal_x << " "
            << target_normal->points[i].normal_y << " "
            << target_normal->points[i].normal_z << " " << std::endl;
    }
    
    //calculate features;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);

    
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> featureEstimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchMothodPtr(new pcl::search::KdTree<pcl::PointXYZ>);

    normalverification(source_normal);
    featureEstimation.setInputCloud(source_clouds);
    featureEstimation.setInputNormals(source_normal);
    featureEstimation.setSearchMethod(searchMothodPtr);
    featureEstimation.setRadiusSearch(featureRadius);
    featureEstimation.compute(*source_features);

    normalverification(target_normal);
    n = target_normal->points.size();
    for (int i = 0; i < n; i++)
    {
        float nx = target_normal->points[i].normal_x;
        float ny = target_normal->points[i].normal_y;
        float nz = target_normal->points[i].normal_z;

        float norm = nx * nx + ny * ny + nz * nz;
        if (norm < 0.01)
        {
            std::cout << "normal 0." << std::endl;
        }
    }
    featureEstimation.setInputCloud(target_clouds);
    featureEstimation.setInputNormals(target_normal);
    featureEstimation.setSearchMethod(searchMothodPtr);
    featureEstimation.setRadiusSearch(featureRadius);
    featureEstimation.compute(*target_features);


    // 定义匹配对象
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacIA;

    // 设置匹配参数
    sacIA.setMinSampleDistance(0.05f);
    sacIA.setMaxCorrespondenceDistance(0.01f * 0.01f);
    sacIA.setMaximumIterations(500);
    // 设置目标点云与特征
    sacIA.setInputTarget(target_clouds);
    sacIA.setTargetFeatures(target_features);

    sacIA.setInputSource(source_clouds);
    sacIA.setSourceFeatures(source_features);

    double score=sacIA.getFitnessScore(0.01f * 0.01f);
    pcl::PointCloud<pcl::PointXYZ> registration_output;
    sacIA.align(registration_output);
    Eigen::Matrix4f pose = sacIA.getFinalTransformation();
    

    pcl::PointCloud<pcl::PointNormal>::Ptr source(new pcl::PointCloud<pcl::PointNormal>);
    
    n = source_clouds->points.size();
    for (int i = 0; i < n; i++)
    {
        source->points.push_back(pcl::PointNormal(source_clouds->points[i].x,
            source_clouds->points[i].y,
            source_clouds->points[i].z,
            source_normal->points[i].normal_x,
            source_normal->points[i].normal_y, 
            source_normal->points[i].normal_z, 0.0));
    }
  
    pcl::PointCloud<pcl::PointNormal> transformed_cloud;
    pcl::transformPointCloudWithNormals(*source, transformed_cloud, pose);
    pcl::io::savePLYFileASCII("G:/paper/testdata/bun000-2.ply", transformed_cloud);

    //
    system("pause");
    std::ifstream input_stream("./data/object_templates.txt");
    std::cout << "xx" << std::endl;
    while (input_stream.good())
    {
       
        /*std::string pcd_filename;
        std::getline(input_stream, pcd_filename);
        if (pcd_filename.empty() || pcd_filename.at(0) == '#')
            continue;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(pcd_filename, *cloudPtr);

        pcl::PointCloud<pcl::FPFHSignature33>::Ptr featurePtr;
        calculateTemplate(cloudPtr, 0.02f, 0.02f, featurePtr);
       // std::cout << "计算路径为"<< pcd_filename<<"的点云特征完成"<<std::endl;
       
        clouds.push_back(cloudPtr);
        features.push_back(featurePtr);*/
    }
    input_stream.close();

    /*******************************************************************************************************************
     * 加载目标点云
     ******************************************************************************************************************/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTtarget(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile("./data/person.pcd", *cloudTtarget);
    std::cout << "目标点云读取完成" << std::endl;

   
    // 直通滤波
    {
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloudTtarget);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0, 1.0);
        pass.filter(*cloudTtarget);

       // std::cout << "目标点云直通滤波完成" << std::endl;
    }
    /*
    // 体素网格降采样
    {
        pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
        vox_grid.setInputCloud(cloudTtarget);
        vox_grid.setLeafSize(0.005f, 0.005f, 0.005f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
        vox_grid.filter(*tempCloud);
        cloudTtarget = tempCloud;

       // std::cout << "目标点云降采样完成" << std::endl;
    }

    // 计算待匹配点云特征
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr targetFeaturePtr;
    calculateTemplate(cloudTtarget, 0.02f, 0.02f, targetFeaturePtr);

    // 定义匹配对象
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sacIA;

    // 设置匹配参数
    sacIA.setMinSampleDistance(0.05f);
    sacIA.setMaxCorrespondenceDistance(0.01f * 0.01f);
    sacIA.setMaximumIterations(500);

    // 设置目标点云与特征
    sacIA.setInputTarget(cloudTtarget);
    sacIA.setTargetFeatures(targetFeaturePtr);

    // 遍历模板，获取匹配结果
    std::vector<float> scores;
    std::vector<Eigen::Matrix4f> transformations;
    for (size_t i = 0; i < clouds.size(); i++)
    {
        sacIA.setInputSource(clouds[i]);
        sacIA.setSourceFeatures(features[i]);

        pcl::PointCloud<pcl::PointXYZ> registration_output;
        sacIA.align(registration_output);

        //std::cout << "第 " << i << " 个模板匹配成功" << std::endl;

        scores.push_back((float)sacIA.getFitnessScore(0.01f * 0.01f));
        transformations.push_back(sacIA.getFinalTransformation());
    }

    // 获取最佳匹配
    float lowest_score = std::numeric_limits<float>::infinity();
    int indexBest = 0;
    for (std::size_t i = 0; i < scores.size(); ++i)
    {
        if (scores[i] < lowest_score)
        {
            lowest_score = scores[i];
            indexBest = (int)i;
        }
    }
    auto transformBest = transformations[indexBest];

    // 输出匹配得分(一般小于0.0002是比较好的)
    printf("Best fitness score: %f\n", lowest_score);

    // 输出旋转矩阵与平移向量
    Eigen::Matrix3f rotation = transformBest.block<3, 3>(0, 0);
    Eigen::Vector3f translation = transformBest.block<3, 1>(0, 3);

    printf("\n");
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
    printf("\n");
    printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

    // 保存最佳匹配模板
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(*clouds[indexBest], transformed_cloud, transformBest);
    pcl::io::savePCDFileBinary("output.pcd", transformed_cloud);
    */
    return EXIT_SUCCESS;
}
