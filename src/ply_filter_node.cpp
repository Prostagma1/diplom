#include <ros/ros.h>
#include <ros/node_handle.h>
#include <string>
#include <vector>
#include <filesystem>
#include <stdexcept>

#include "diplom/pointcloud_utils.h"

PointCloud applyRorFilter(const PointCloud& input_cloud,
                          double radius_search,
                          int min_neighbors,
                          double voxel_leaf_size)
{
    if (input_cloud.empty()) {
         ROS_WARN("[ROR Node] Input cloud is empty. Returning empty cloud.");
         return input_cloud;
    }
     if (radius_search <= 0.0 || min_neighbors < 0 || voxel_leaf_size <= 0.0) {
        ROS_ERROR("[ROR Node] Invalid params (radius=%.3f, neighbors=%d, leaf=%.4f). Returning original.",
                 radius_search, min_neighbors, voxel_leaf_size);
        return input_cloud;
    }

    PointCloud filtered_cloud;
    filtered_cloud.reserve(input_cloud.size());
    const double radius_search_sq = radius_search * radius_search;
    const size_t n_points = input_cloud.size();

    ROS_INFO("[ROR Node] Preparing Voxel Grid (leaf size = %.4f)...", voxel_leaf_size);
    AlignedBoundingBox bbox = calculateBoundingBoxEigen(input_cloud);
    if (bbox.isEmpty()) {
        ROS_ERROR("[ROR Node] Invalid bounding box. Returning original cloud.");
        return input_cloud;
    }

    const double inv_leaf_size = 1.0 / voxel_leaf_size;
    VoxelGridMap grid = buildVoxelGrid(input_cloud, voxel_leaf_size, bbox);
    ROS_INFO("[ROR Node] Voxel Grid built (%zu occupied). Filtering %zu points...", grid.size(), n_points);

    int progress_pct = -1;
    for (size_t i = 0; i < n_points; ++i) {
        int neighbor_count = countNeighborsVoxelGrid(i, input_cloud, grid, radius_search_sq, inv_leaf_size, bbox);
        if (neighbor_count >= min_neighbors) {
            filtered_cloud.push_back(input_cloud[i]);
        }

        int current_pct = static_cast<int>( (double)(i+1) / n_points * 100.0 );
        if (current_pct >= progress_pct + 10) {
             ROS_INFO("[ROR Node] Progress %d%%...", current_pct);
             progress_pct = current_pct;
        }
    }
    if (progress_pct < 100 && n_points > 0) ROS_INFO("[ROR Node] Progress 100%...");
    ROS_INFO("[ROR Node] ROR Filtering complete.");

    return filtered_cloud;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "ror_filter_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ROS_INFO("Starting ROR Filter Node...");

    // Параметры ROR
    std::string input_ply_path, output_ply_path;
    bool overwrite_output = false;
    double ror_radius_search = 0.1;
    int ror_min_neighbors = 5;
    double ror_voxel_leaf_size = -1.0; 

    // Получение параметров
    if (!pnh.getParam("input_ply_path", input_ply_path)) { ROS_FATAL("[ROR Node] Missing 'input_ply_path' param!"); return 1; }
    if (!pnh.getParam("output_ply_path", output_ply_path)) {
         try { std::filesystem::path ip(input_ply_path); output_ply_path = (ip.parent_path().string().empty() ? "." : ip.parent_path().string()) + "/" + ip.stem().string() + "_filtered_ror.ply"; ROS_WARN("[ROR Node] Defaulting 'output_ply_path' to '%s'", output_ply_path.c_str()); }
         catch (const std::exception& e) { ROS_ERROR("[ROR Node] Error generating default output path: %s", e.what()); return 1; }
    }
    pnh.param<bool>("overwrite_output", overwrite_output, overwrite_output);
    pnh.param<double>("radius_search", ror_radius_search, ror_radius_search); 
    pnh.param<int>("min_neighbors", ror_min_neighbors, ror_min_neighbors);
    pnh.param<double>("voxel_leaf_size", ror_voxel_leaf_size, ror_voxel_leaf_size);

    if (ror_voxel_leaf_size <= 0.0) {
        ror_voxel_leaf_size = ror_radius_search;
        ROS_INFO("[ROR Node] Auto-setting 'voxel_leaf_size' to %.4f", ror_voxel_leaf_size);
    }
    ROS_INFO("[ROR Node] Params: Input='%s', Output='%s', Overwrite=%s", input_ply_path.c_str(), output_ply_path.c_str(), overwrite_output ? "true" : "false");
    ROS_INFO("[ROR Node] ROR Params: Radius=%.3f, MinNeighbors=%d, VoxelLeaf=%.4f", ror_radius_search, ror_min_neighbors, ror_voxel_leaf_size);
    if (ror_radius_search <= 0.0 || ror_min_neighbors < 0) { ROS_ERROR("[ROR Node] Invalid ROR params."); return 1; }

    try {
        if (!overwrite_output && std::filesystem::exists(output_ply_path)) { ROS_ERROR("[ROR Node] Output '%s' exists.", output_ply_path.c_str()); return 1; }
        if (!std::filesystem::exists(input_ply_path)) { ROS_ERROR("[ROR Node] Input '%s' not found.", input_ply_path.c_str()); return 1; }
        if (!std::filesystem::is_regular_file(input_ply_path)) { ROS_ERROR("[ROR Node] Input '%s' not a file.", input_ply_path.c_str()); return 1; }
    } catch (const std::exception& e) { ROS_ERROR("[ROR Node] Filesystem error: %s", e.what()); return 1; }

    // Загрузка
    PointCloud cloud_raw;
    ROS_INFO("[ROR Node] Loading...");
    if (!loadPlyFile(input_ply_path, cloud_raw)) return 1;
    ROS_INFO("[ROR Node] Loaded %zu points.", cloud_raw.size());

    // Фильтрация
    PointCloud cloud_filtered;
    if (cloud_raw.empty()) {
        ROS_INFO("[ROR Node] Input empty, skipping filter.");
        cloud_filtered = cloud_raw;
    } else {
        ROS_INFO("[ROR Node] Applying ROR filter...");
        ros::Time start = ros::Time::now();
        try {
             cloud_filtered = applyRorFilter(cloud_raw, ror_radius_search, ror_min_neighbors, ror_voxel_leaf_size);
        } catch (const std::exception& e) {
             ROS_FATAL("[ROR Node] Exception during filtering: %s. Exiting.", e.what()); return 1;
        } catch (...) {
             ROS_FATAL("[ROR Node] Unknown exception during filtering. Exiting."); return 1;
        }
        ros::Duration dur = ros::Time::now() - start;
        ROS_INFO("[ROR Node] Filtering took %.3fs. Before: %zu, After: %zu", dur.toSec(), cloud_raw.size(), cloud_filtered.size());
        if (cloud_filtered.empty() && !cloud_raw.empty()) ROS_WARN("[ROR Node] Filtered cloud is empty!");
    }

    // Сохранение
    ROS_INFO("[ROR Node] Saving %zu points to %s...", cloud_filtered.size(), output_ply_path.c_str());
    try {
         std::filesystem::path op(output_ply_path);
         if (!op.parent_path().empty() && !std::filesystem::exists(op.parent_path())) {
             ROS_INFO("[ROR Node] Creating output directory: %s", op.parent_path().c_str());
             std::filesystem::create_directories(op.parent_path());
         }
    } catch (const std::exception& e) { ROS_WARN("[ROR Node] Could not create output directory: %s", e.what()); }

    if (!savePlyFile(output_ply_path, cloud_filtered)) return 1; 
    ROS_INFO("[ROR Node] Save successful.");

    ROS_INFO("ROR Filter Node finished.");
    return 0;
}