#include <ros/ros.h>
#include <ros/node_handle.h>
#include <string>
#include <vector>
#include <filesystem>
#include <stdexcept>
#include <cmath>
#include <numeric>
#include <algorithm>

#include "diplom/pointcloud_utils.h"
#include <Eigen/Eigenvalues>


PointCloud applyMLSSmoothing(const PointCloud& input_cloud,
                             int k,
                             double radius_factor,
                             int grid_resolution,
                             double padding,
                             int knn_for_radius_estimation)
{
    ROS_INFO("[MLS Node] Starting MLS Smoothing...");
    PointCloud smoothed_cloud;
    if (input_cloud.empty()) { ROS_WARN("[MLS Node] Input cloud is empty."); return smoothed_cloud; }
    if (k <= 2 || radius_factor <= 0.0 || grid_resolution < 2 || padding < 0.0 || knn_for_radius_estimation <= 0) {
        ROS_ERROR("[MLS Node] Invalid params (k=%d, factor=%.2f, res=%d, pad=%.2f, knn_h=%d). Aborting.", k, radius_factor, grid_resolution, padding, knn_for_radius_estimation);
        return input_cloud;
    }

    const size_t n_points = input_cloud.size();
    ROS_INFO("[MLS Node] Params: k=%d, radius_factor=%.2f, grid_res=%d, padding=%.2f, knn_for_h=%d", k, radius_factor, grid_resolution, padding, knn_for_radius_estimation);

    // 1. Bbox и Voxel Grid 
    AlignedBoundingBox initial_bbox = calculateBoundingBoxEigen(input_cloud);
    if (initial_bbox.isEmpty()){ ROS_ERROR("[MLS Node] Cannot process empty bounding box."); return input_cloud; }
    double estimated_leaf_size = initial_bbox.diagonal().norm() / 100.0; 
    if (estimated_leaf_size <= 0) estimated_leaf_size = 0.05;
    double inv_leaf_size = 1.0 / estimated_leaf_size;
    VoxelGridMap voxel_grid = buildVoxelGrid(input_cloud, estimated_leaf_size, initial_bbox);
    ROS_INFO("[MLS Node] Voxel grid built (leaf size ~ %.4f).", estimated_leaf_size);

    // 2. Оценка радиуса h
    ROS_INFO("[MLS Node] Estimating influence radius h using %d neighbors...", knn_for_radius_estimation);
    double total_avg_knn_dist_sq = 0.0; size_t valid_points_for_h = 0;
    for (size_t i = 0; i < n_points; ++i) {
        auto neighbors_h = findKNNSorted(input_cloud[i], knn_for_radius_estimation + 1, input_cloud, voxel_grid, inv_leaf_size, initial_bbox);
        double current_point_dist_sq_sum = 0.0; size_t neighbors_found_h = 0;
        for(const auto& pair : neighbors_h) { if (pair.second != i) { current_point_dist_sq_sum += pair.first; neighbors_found_h++; if(neighbors_found_h >= knn_for_radius_estimation) break; } }
        if (neighbors_found_h > 0) { total_avg_knn_dist_sq += current_point_dist_sq_sum / neighbors_found_h; valid_points_for_h++; }
    }
    if (valid_points_for_h == 0) { ROS_ERROR("[MLS Node] Could not estimate average KNN distance. Aborting."); return input_cloud; }
    double avg_dist_knn = std::sqrt(total_avg_knn_dist_sq / valid_points_for_h);
    double h = radius_factor * avg_dist_knn; double h_squared = h * h;
    if (h <= 0.0) { ROS_ERROR("[MLS Node] Estimated h=%.4f is non-positive. Aborting.", h); return input_cloud; }
    ROS_INFO("[MLS Node] Estimated avg KNN dist = %.4f, Influence radius h = %.4f", avg_dist_knn, h);

    // 3. Создание сетки реконструкции
    ROS_INFO("[MLS Node] Generating reconstruction grid...");
    AlignedBoundingBox padded_bbox = initial_bbox;
    Point bbox_size = initial_bbox.sizes(); Point padding_vec = bbox_size * padding;
    padded_bbox.min() -= padding_vec; padded_bbox.max() += padding_vec;
    Point padded_size = padded_bbox.sizes(); double max_dim = padded_size.maxCoeff();
    if (max_dim <= 0) { ROS_ERROR("[MLS Node] Invalid padded bounding box size. Aborting."); return input_cloud; }
    double grid_step = max_dim / (grid_resolution - 1);
    PointCloud grid_points;
    int count_x = std::max(1, static_cast<int>(std::ceil(padded_size.x() / grid_step)) + 1);
    int count_y = std::max(1, static_cast<int>(std::ceil(padded_size.y() / grid_step)) + 1);
    int count_z = std::max(1, static_cast<int>(std::ceil(padded_size.z() / grid_step)) + 1);
    grid_points.reserve(count_x * count_y * count_z);
    for (int ix = 0; ix < count_x; ++ix) for (int iy = 0; iy < count_y; ++iy) for (int iz = 0; iz < count_z; ++iz) {
        grid_points.push_back(padded_bbox.min() + Point(ix * grid_step, iy * grid_step, iz * grid_step));
    }
    ROS_INFO("[MLS Node] Generated %zu grid points (step=%.4f).", grid_points.size(), grid_step);

    // 4. Выполнение MLS для каждой точки сетки
    ROS_INFO("[MLS Node] Performing MLS projection...");
    smoothed_cloud.reserve(grid_points.size());
    const double h_search_radius_sq = h_squared * 9.0;

    int progress_pct = -1; size_t grid_point_count = grid_points.size();
    for(size_t i = 0; i < grid_point_count; ++i) {
        const Point& query_pt = grid_points[i];
        auto neighbors = findKNNSorted(query_pt, k, input_cloud, voxel_grid, inv_leaf_size, initial_bbox, h_search_radius_sq); 

        if (neighbors.size() < 3) continue; 

        double total_weight = 0.0; Point weighted_centroid = Point::Zero();
        std::vector<double> weights; weights.reserve(neighbors.size());
        for (const auto& np : neighbors) { double w = std::exp(-np.first / h_squared); weights.push_back(w); weighted_centroid += w * input_cloud[np.second]; total_weight += w; }
        if (total_weight <= std::numeric_limits<double>::epsilon()) continue;
        weighted_centroid /= total_weight;

        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (size_t j = 0; j < neighbors.size(); ++j) { Point diff = input_cloud[neighbors[j].second] - weighted_centroid; cov += weights[j] * (diff * diff.transpose()); }
        cov /= total_weight;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(cov);
        if (eigensolver.info() != Eigen::Success) { ROS_WARN_THROTTLE(5.0,"[MLS Node] Eigen decomposition failed."); continue; }
        Point normal = eigensolver.eigenvectors().col(0); 

        Point projected_pt = query_pt - normal.dot(query_pt - weighted_centroid) * normal;
        smoothed_cloud.push_back(projected_pt);

        // Progress Reporting
        int current_pct = static_cast<int>( (double)(i+1) / grid_point_count * 100.0 );
        if (current_pct >= progress_pct + 5) { ROS_INFO("[MLS Node] Projection Progress %d%%...", current_pct); progress_pct = current_pct; }
    }
    if (progress_pct < 100 && !grid_points.empty()) ROS_INFO("[MLS Node] Projection Progress 100%...");

    ROS_INFO("[MLS Node] MLS Smoothing finished. Generated %zu smoothed points.", smoothed_cloud.size());
    return smoothed_cloud;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mls_smoother_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ROS_INFO("Starting MLS Smoother Node...");

    std::string input_ply_path, output_ply_path;
    bool overwrite_output = false;
    int mls_k = 15;
    double mls_radius_factor = 1.5;
    int mls_grid_res = 50;
    double mls_padding = 0.05;
    int mls_knn_for_radius = 6;

    if (!pnh.getParam("input_ply_path", input_ply_path)) { ROS_FATAL("[MLS Node] Missing 'input_ply_path' param!"); return 1; }
    if (!pnh.getParam("output_ply_path", output_ply_path)) {
        try { std::filesystem::path ip(input_ply_path); output_ply_path = (ip.parent_path().string().empty() ? "." : ip.parent_path().string()) + "/" + ip.stem().string() + "_smoothed_mls.ply"; ROS_WARN("[MLS Node] Defaulting 'output_ply_path' to '%s'", output_ply_path.c_str()); }
        catch (const std::exception& e) { ROS_ERROR("[MLS Node] Error generating default output path: %s", e.what()); return 1; }
    }
    pnh.param<bool>("overwrite_output", overwrite_output, overwrite_output);
    pnh.param<int>("k", mls_k, mls_k); 
    pnh.param<double>("radius_factor", mls_radius_factor, mls_radius_factor);
    pnh.param<int>("grid_res", mls_grid_res, mls_grid_res);
    pnh.param<double>("padding", mls_padding, mls_padding);
    pnh.param<int>("knn_for_radius", mls_knn_for_radius, mls_knn_for_radius);

    ROS_INFO("[MLS Node] Params: Input='%s', Output='%s', Overwrite=%s", input_ply_path.c_str(), output_ply_path.c_str(), overwrite_output ? "true" : "false");
    ROS_INFO("[MLS Node] MLS Params: k=%d, radius_factor=%.2f, grid_res=%d, padding=%.2f, knn_for_h=%d", mls_k, mls_radius_factor, mls_grid_res, mls_padding, mls_knn_for_radius);
    if (mls_k <= 2 || mls_radius_factor <= 0.0 || mls_grid_res < 2 || mls_padding < 0.0 || mls_knn_for_radius <= 0) { ROS_ERROR("[MLS Node] Invalid MLS params."); return 1; }

    try {
        if (!overwrite_output && std::filesystem::exists(output_ply_path)) { ROS_ERROR("[MLS Node] Output '%s' exists.", output_ply_path.c_str()); return 1; }
        if (!std::filesystem::exists(input_ply_path)) { ROS_ERROR("[MLS Node] Input '%s' not found.", input_ply_path.c_str()); return 1; }
        if (!std::filesystem::is_regular_file(input_ply_path)) { ROS_ERROR("[MLS Node] Input '%s' not a file.", input_ply_path.c_str()); return 1; }
    } catch (const std::exception& e) { ROS_ERROR("[MLS Node] Filesystem error: %s", e.what()); return 1; }

    // Загрузка
    PointCloud cloud_raw;
    ROS_INFO("[MLS Node] Loading...");
    if (!loadPlyFile(input_ply_path, cloud_raw)) return 1; 
    ROS_INFO("[MLS Node] Loaded %zu points.", cloud_raw.size());

    // Сглаживание
    PointCloud cloud_smoothed;
    if (cloud_raw.empty()) {
        ROS_INFO("[MLS Node] Input empty, skipping smoothing.");
        cloud_smoothed = cloud_raw;
    } else {
        ROS_INFO("[MLS Node] Applying MLS smoothing...");
        ros::Time start = ros::Time::now();
         try {
             cloud_smoothed = applyMLSSmoothing(cloud_raw, mls_k, mls_radius_factor, mls_grid_res, mls_padding, mls_knn_for_radius);
         } catch (const std::exception& e) {
             ROS_FATAL("[MLS Node] Exception during smoothing: %s. Exiting.", e.what()); return 1;
        } catch (...) {
             ROS_FATAL("[MLS Node] Unknown exception during smoothing. Exiting."); return 1;
        }
        ros::Duration dur = ros::Time::now() - start;
        ROS_INFO("[MLS Node] Smoothing took %.3fs. Before: %zu, After: %zu", dur.toSec(), cloud_raw.size(), cloud_smoothed.size());
         if (cloud_smoothed.empty() && !cloud_raw.empty()) ROS_WARN("[MLS Node] Smoothed cloud is empty!");
    }

    // Сохранение
    ROS_INFO("[MLS Node] Saving %zu points to %s...", cloud_smoothed.size(), output_ply_path.c_str());
     try {
         std::filesystem::path op(output_ply_path);
         if (!op.parent_path().empty() && !std::filesystem::exists(op.parent_path())) {
             ROS_INFO("[MLS Node] Creating output directory: %s", op.parent_path().c_str());
             std::filesystem::create_directories(op.parent_path());
         }
    } catch (const std::exception& e) { ROS_WARN("[MLS Node] Could not create output directory: %s", e.what()); }

    if (!savePlyFile(output_ply_path, cloud_smoothed)) return 1; 
    ROS_INFO("[MLS Node] Save successful.");

    ROS_INFO("MLS Smoother Node finished.");
    return 0;
}