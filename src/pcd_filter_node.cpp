#include <ros/ros.h>
#include <ros/node_handle.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <string>
#include <algorithm> 
#include <filesystem> 

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv) {
    // --- 1. Инициализация ROS ---
    ros::init(argc, argv, "pcd_filter_node");
    ros::NodeHandle nh;         // Публичный NodeHandle
    ros::NodeHandle pnh("~");   // Приватный NodeHandle для параметров

    ROS_INFO("Starting PCD Filter Node...");

    // --- 2. Получение параметров ---
    std::string input_pcd_path;
    std::string output_pcd_path; 
    std::string filter_type_str = "sor"; // "sor", "ror", "none"
    bool overwrite_output = false;     

    // Параметры SOR
    int sor_mean_k = 50;
    double sor_stddev_mul_thresh = 1.0;

    // Параметры ROR
    double ror_radius_search = 0.1;
    int ror_min_neighbors = 5;

    if (!pnh.getParam("input_pcd_path", input_pcd_path)) {
        ROS_FATAL("Required parameter 'input_pcd_path' is missing!");
        return 1;
    }
    if (!pnh.getParam("output_pcd_path", output_pcd_path)) {
         // Если путь вывода не указан, генерируем его на основе входного
         std::filesystem::path input_path(input_pcd_path); 
         std::string stem = input_path.stem().string();
         std::string extension = input_path.extension().string();
         std::filesystem::path parent_path = input_path.parent_path(); 
         std::string parent_path_str = parent_path.string();
         if (parent_path_str.empty() || parent_path_str == "."){ 
             parent_path_str = ".";
         }
         output_pcd_path = parent_path_str + "/" + stem + "_filtered" + extension;
         ROS_WARN("Parameter 'output_pcd_path' not specified. Defaulting to '%s'", output_pcd_path.c_str());
    }

    pnh.param<std::string>("filter_type", filter_type_str, "sor");
    pnh.param<bool>("overwrite_output", overwrite_output, false);

    std::transform(filter_type_str.begin(), filter_type_str.end(), filter_type_str.begin(), ::tolower);

    bool filter_enabled = true;
    if (filter_type_str == "sor") {
        pnh.param<int>("sor/mean_k", sor_mean_k, 50);
        pnh.param<double>("sor/stddev_thresh", sor_stddev_mul_thresh, 1.0);
        ROS_INFO("Filter type: SOR (MeanK=%d, StddevThresh=%.2f)", sor_mean_k, sor_stddev_mul_thresh);
    } else if (filter_type_str == "ror") {
        pnh.param<double>("ror/radius_search", ror_radius_search, 0.1);
        pnh.param<int>("ror/min_neighbors", ror_min_neighbors, 5);
        ROS_INFO("Filter type: ROR (Radius=%.3f, MinNeighbors=%d)", ror_radius_search, ror_min_neighbors);
    } else if (filter_type_str == "none") {
        ROS_INFO("Filter type: None. Filtering disabled.");
        filter_enabled = false;
    } else {
        ROS_WARN("Unknown filter type '%s'. Disabling filtering.", filter_type_str.c_str());
        filter_enabled = false;
    }
    ROS_INFO("Input PCD path: %s", input_pcd_path.c_str());
    ROS_INFO("Output PCD path: %s", output_pcd_path.c_str());
    ROS_INFO("Overwrite output: %s", overwrite_output ? "true" : "false");

    // Проверка существования выходного файла, если перезапись запрещена
    if (!overwrite_output && std::filesystem::exists(output_pcd_path)) { 
         ROS_ERROR("Output file '%s' already exists and overwrite is disabled. Exiting.", output_pcd_path.c_str());
         return 1;
    }


    // --- 3. Загрузка входного PCD файла ---
    PointCloud::Ptr cloud_raw(new PointCloud);
    ROS_INFO("Loading input PCD file: %s", input_pcd_path.c_str());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd_path, *cloud_raw) == -1) {
        ROS_ERROR("Couldn't read file %s", input_pcd_path.c_str());
        return 1;
    }
     if (cloud_raw->empty()) {
         ROS_ERROR("Input cloud %s is empty!", input_pcd_path.c_str());
         return 1;
    }
    ROS_INFO("Loaded %ld data points from %s", cloud_raw->width * cloud_raw->height, input_pcd_path.c_str());


    // --- 4. Фильтрация ---
    PointCloud::Ptr cloud_filtered(new PointCloud);
    if (filter_enabled) {
        ROS_INFO("Applying filter '%s'...", filter_type_str.c_str());
        try {
            if (filter_type_str == "sor") {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                sor.setInputCloud(cloud_raw);
                sor.setMeanK(sor_mean_k);
                sor.setStddevMulThresh(sor_stddev_mul_thresh);
                sor.filter(*cloud_filtered);
            } else if (filter_type_str == "ror") {
                pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
                ror.setInputCloud(cloud_raw);
                ror.setRadiusSearch(ror_radius_search);
                ror.setMinNeighborsInRadius(ror_min_neighbors);
                ror.filter(*cloud_filtered);
            }
            ROS_INFO("Filtering complete. Points before: %ld, Points after: %ld",
                     cloud_raw->size(), cloud_filtered->size());

             if (cloud_filtered->empty()) {
                  ROS_WARN("Filtered point cloud is empty! Check filter parameters.");
             }

        } catch (const std::exception& e) {
             ROS_ERROR("Exception during filtering process: %s", e.what());
             return 1;
        } catch (...) {
             ROS_ERROR("Unknown exception during filtering process.");
             return 1;
        }
    } else {
        ROS_INFO("Filtering is disabled. Using raw cloud as output.");
        cloud_filtered = cloud_raw;
    }


    // --- 5. Сохранение результата ---
    ROS_INFO("Saving %s point cloud to: %s", filter_enabled ? "filtered" : "raw", output_pcd_path.c_str());
    try {
         // Создаем директорию для сохранения, если ее нет
         std::filesystem::path output_dir = std::filesystem::path(output_pcd_path).parent_path(); 
         if (!output_dir.empty() && !std::filesystem::exists(output_dir)) { 
              ROS_INFO("Creating output directory: %s", output_dir.c_str());
              if (!std::filesystem::create_directories(output_dir)) { 
                   ROS_ERROR("Failed to create output directory!");
              }
         }

        if (pcl::io::savePCDFileBinary(output_pcd_path, *cloud_filtered) == 0) { // 0 = успех
            ROS_INFO("Successfully saved output PCD file.");
        } else {
            ROS_ERROR("Failed to save output PCD file!");
            return 1;
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while saving output PCD file %s: %s", output_pcd_path.c_str(), e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("Unknown exception while saving output PCD file %s", output_pcd_path.c_str());
        return 1;
    }

    ROS_INFO("PCD Filter Node finished successfully.");
    return 0;
}