#include <ros/ros.h>
#include <ros/node_handle.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <limits>
#include <filesystem> 
#include <stdexcept>
#include <unordered_map>
#include <tuple>

// --- 1. Определения структуры данных ---
struct Point {
    double x = 0.0, y = 0.0, z = 0.0;
};

using PointCloud = std::vector<Point>;

// Структура для хранения границ облака
struct BoundingBox {
    Point min_pt = {std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    Point max_pt = {-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()};
    bool valid = false; 
};


inline double distance_sq(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return dx * dx + dy * dy + dz * dz;
}

// --- 2. Функции для работы с PLY файлами ---

// Загружает облако точек из ASCII PLY файла
bool loadPlyFile(const std::string& filename, PointCloud& cloud) {
    cloud.clear();
    std::ifstream ifs(filename);
    if (!ifs.is_open()) {
        ROS_ERROR("Could not open PLY file: %s", filename.c_str());
        return false;
    }

    std::string line;
    long point_count = 0;
    int property_count = 0; 
    bool header_parsed = false;
    bool format_ok = false;
    bool properties_ok = false; 

    // Чтение заголовка
    while (std::getline(ifs, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string keyword;
        ss >> keyword;

        if (keyword == "ply") continue;
        if (keyword == "format") {
             std::string format_type;
             double version;
             ss >> format_type >> version;
             if (format_type == "ascii" && version >= 1.0) {
                 format_ok = true;
             } else {
                  ROS_ERROR("Unsupported PLY format in %s. Only 'format ascii 1.0' (or higher) is supported.", filename.c_str());
                  ifs.close(); return false;
             }
        } else if (keyword == "comment") {
            continue;
        } else if (keyword == "element") {
            std::string element_name;
            long count_in_element = 0;
            ss >> element_name >> count_in_element;
            if (element_name == "vertex") {
                point_count = count_in_element;
                 if (point_count < 0) {
                     ROS_ERROR("PLY file %s reports negative vertex count: %ld", filename.c_str(), point_count);
                     ifs.close(); return false;
                 }
                 if (point_count == 0) {
                 }
            }
        } else if (keyword == "property") {
             std::string type, name;
             ss >> type >> name;
             std::transform(name.begin(), name.end(), name.begin(),
                            [](unsigned char c){ return std::tolower(c); });
             if ((type == "float" || type == "double" || type == "float32" || type == "float64") &&
                 (name == "x" || name == "y" || name == "z")) {
                 property_count++;
                 if (property_count >= 3) properties_ok = true;
             }
        } else if (keyword == "end_header") {
            header_parsed = true;
            break;
        } else {
            ROS_WARN("Ignoring unknown header line in %s: %s", filename.c_str(), line.c_str());
        }
    }

    if (!header_parsed) {
        ROS_ERROR("Could not find 'end_header' in PLY file: %s", filename.c_str());
        ifs.close(); return false;
    }
     if (!format_ok){
         ROS_ERROR("PLY format line missing or invalid in %s", filename.c_str());
         ifs.close(); return false;
     }
     if (point_count == 0) { 
          ROS_INFO("PLY file %s loaded successfully (0 vertices defined in header).", filename.c_str());
          ifs.close(); return true; 
     }
     if (!properties_ok) {
         ROS_ERROR("Valid vertex properties (float/double x, y, z) not found in header of %s", filename.c_str());
         ifs.close(); return false;
     }

    // Чтение данных точек
    cloud.reserve(point_count);
    for (long i = 0; i < point_count; ++i) {
        if (!std::getline(ifs, line)) {
            ROS_ERROR("Premature end of file in %s. Expected %ld points, read %ld.", filename.c_str(), point_count, (long)cloud.size());
             ifs.close(); return false; 
        }
         if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty()){ 
            i--; 
            ROS_WARN("Skipping empty line in data section of %s at point index %ld", filename.c_str(), i+1);
            continue;
        }

        std::stringstream ss(line);
        Point p;
        if (!(ss >> p.x >> p.y >> p.z)) {
            ROS_ERROR("Error parsing point data at line %ld ('%s') in %s. Expected format: x y z [...]", (long)(cloud.size() + 1 + 10), line.c_str(), filename.c_str());
            ifs.close(); return false;
        }
        cloud.push_back(p);
    }

    ifs.close();

     if (cloud.size() != point_count) {
          ROS_WARN("Read %ld points from %s, but header specified %ld. Using read count.", (long)cloud.size(), filename.c_str(), point_count);
     }

    return true;
}

// Сохраняет облако точек в ASCII PLY файл
bool savePlyFile(const std::string& filename, const PointCloud& cloud) {
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        ROS_ERROR("Could not open PLY file for writing: %s", filename.c_str());
        return false;
    }

    ofs << "ply\n";
    ofs << "format ascii 1.0\n";
    ofs << "comment Generated by ply_filter_node (ROR only)\n";
    ofs << "element vertex " << cloud.size() << "\n";
    ofs << "property double x\n"; 
    ofs << "property double y\n";
    ofs << "property double z\n";
    ofs << "end_header\n";

    ofs << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (const auto& p : cloud) {
        ofs << p.x << " " << p.y << " " << p.z << "\n";
    }

    if (!ofs) { 
         ROS_ERROR("Error occurred while writing data to PLY file: %s", filename.c_str());
         ofs.close(); 
         return false;
    }

    ofs.close(); 

    if (!ofs) {
         ROS_ERROR("Error occurred while closing PLY file: %s", filename.c_str());
         return false;
    }

    return true;
}

// --- 3. Voxel Grid Структуры и Функции  ---

// Индекс вокселя (ix, iy, iz)
struct VoxelIndex {
    int x, y, z;

    bool operator==(const VoxelIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Хеш-функция для VoxelIndex для unordered_map
struct VoxelIndexHash {
    std::size_t operator()(const VoxelIndex& idx) const {
        size_t seed = 0;
        auto hash_combine = [&seed](int val) {
             seed ^= std::hash<int>{}(val) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };
        hash_combine(idx.x);
        hash_combine(idx.y);
        hash_combine(idx.z);
        return seed;
    }
};

// Карта: Индекс вокселя -> Вектор индексов точек в этом вокселе
using VoxelGridMap = std::unordered_map<VoxelIndex, std::vector<size_t>, VoxelIndexHash>;

// Функция для вычисления Bounding Box облака точек
BoundingBox calculateBoundingBox(const PointCloud& cloud) {
    BoundingBox bbox;
    if (cloud.empty()) {
        return bbox; 
    }

    bbox.min_pt = cloud[0];
    bbox.max_pt = cloud[0];

    for (size_t i = 1; i < cloud.size(); ++i) {
        const auto& p = cloud[i];
        bbox.min_pt.x = std::min(bbox.min_pt.x, p.x);
        bbox.min_pt.y = std::min(bbox.min_pt.y, p.y);
        bbox.min_pt.z = std::min(bbox.min_pt.z, p.z);
        bbox.max_pt.x = std::max(bbox.max_pt.x, p.x);
        bbox.max_pt.y = std::max(bbox.max_pt.y, p.y);
        bbox.max_pt.z = std::max(bbox.max_pt.z, p.z);
    }
    bbox.valid = true;
    return bbox;
}

// Функция для построения Voxel Grid
VoxelGridMap buildVoxelGrid(const PointCloud& cloud, double leaf_size, const BoundingBox& bbox) {
    VoxelGridMap grid;
    if (!bbox.valid || leaf_size <= 0.0) {
        ROS_ERROR("Cannot build voxel grid: Invalid bounding box or non-positive leaf size (%.3f).", leaf_size);
        return grid; 
    }

    grid.reserve(cloud.size() / 5); 

    const double inv_leaf_size = 1.0 / leaf_size; 

    for (size_t i = 0; i < cloud.size(); ++i) {
        const auto& p = cloud[i];
        int ix = static_cast<int>(std::floor((p.x - bbox.min_pt.x) * inv_leaf_size));
        int iy = static_cast<int>(std::floor((p.y - bbox.min_pt.y) * inv_leaf_size));
        int iz = static_cast<int>(std::floor((p.z - bbox.min_pt.z) * inv_leaf_size));

        grid[{ix, iy, iz}].push_back(i);
    }
    return grid;
}

// Функция поиска соседей в заданном радиусе (квадрат радиуса) с использованием Voxel Grid
int countNeighborsVoxelGrid(size_t query_idx, const PointCloud& cloud, const VoxelGridMap& grid,
                             double radius_search_sq, double inv_leaf_size, const BoundingBox& bbox)
{
    int neighbor_count = 0;
    const Point& query_point = cloud[query_idx];

    int center_ix = static_cast<int>(std::floor((query_point.x - bbox.min_pt.x) * inv_leaf_size));
    int center_iy = static_cast<int>(std::floor((query_point.y - bbox.min_pt.y) * inv_leaf_size));
    int center_iz = static_cast<int>(std::floor((query_point.z - bbox.min_pt.z) * inv_leaf_size));

    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                VoxelIndex neighbor_voxel_idx = {center_ix + dx, center_iy + dy, center_iz + dz};

                auto it = grid.find(neighbor_voxel_idx);
                if (it != grid.end()) {
                    const std::vector<size_t>& point_indices = it->second;
                    for (size_t candidate_idx : point_indices) {
                        if (candidate_idx == query_idx) {
                            continue;
                        }
                        if (distance_sq(query_point, cloud[candidate_idx]) <= radius_search_sq) {
                            neighbor_count++;
                        }
                    }
                }
            }
        }
    }
    return neighbor_count;
}


// --- 4. Реализация фильтра ROR ---
PointCloud applyRorFilterVoxelGrid(const PointCloud& input_cloud, double radius_search, int min_neighbors, double voxel_leaf_size) {
    if (input_cloud.empty() || radius_search <= 0.0 || min_neighbors < 0 || voxel_leaf_size <= 0.0) {
        ROS_WARN("ROR Filter VoxelGrid: Invalid input parameters (radius=%.3f > 0, min_neighbors=%d >= 0, leaf_size=%.4f > 0) or empty cloud. Returning original cloud.",
                 radius_search, min_neighbors, voxel_leaf_size);
        return input_cloud;
    }

    PointCloud filtered_cloud;
    filtered_cloud.reserve(input_cloud.size()); 
    const double radius_search_sq = radius_search * radius_search; 
    const size_t n_points = input_cloud.size();

    if (n_points == 0) return filtered_cloud; 

    ROS_INFO("ROR Filter VoxelGrid: Preparing Voxel Grid (leaf size = %.4f)...", voxel_leaf_size);

    // 1. Вычисляем Bounding Box
    BoundingBox bbox = calculateBoundingBox(input_cloud);
    if (!bbox.valid) {
        ROS_ERROR("ROR Filter VoxelGrid: Could not calculate valid bounding box for non-empty cloud. Returning original cloud.");
        return input_cloud; 
    }
    ROS_DEBUG("ROR Filter VoxelGrid: BBox Min(%.3f, %.3f, %.3f), Max(%.3f, %.3f, %.3f)",
              bbox.min_pt.x, bbox.min_pt.y, bbox.min_pt.z,
              bbox.max_pt.x, bbox.max_pt.y, bbox.max_pt.z);


    // 2. Строим Voxel Grid
    const double inv_leaf_size = 1.0 / voxel_leaf_size;
    VoxelGridMap grid = buildVoxelGrid(input_cloud, voxel_leaf_size, bbox);
    ROS_INFO("ROR Filter VoxelGrid: Voxel Grid built (%zu voxels occupied).", grid.size());


    // 3. Фильтруем точки, используя поиск по сетке
    ROS_INFO("ROR Filter VoxelGrid: Checking neighbors using grid (radius=%.3f, min_neighbors=%d)...", radius_search, min_neighbors);

    for (size_t i = 0; i < n_points; ++i) {
        int neighbor_count = countNeighborsVoxelGrid(i, input_cloud, grid, radius_search_sq, inv_leaf_size, bbox);

        if (neighbor_count >= min_neighbors) {
            filtered_cloud.push_back(input_cloud[i]);
        }
    }

    return filtered_cloud;
}


// --- 5. Основная функция ---
int main(int argc, char** argv) {
    ros::init(argc, argv, "ply_filter_node");
    ros::NodeHandle nh;       
    ros::NodeHandle pnh("~"); 

    ROS_INFO("Starting PLY Filter Node (ROR Voxel Grid only)...");

    // --- Параметры ---
    std::string input_ply_path, output_ply_path;
    std::string filter_type_str = "ror"; 
    bool overwrite_output = false;

    // Параметры ROR
    double ror_radius_search = 0.1;
    int ror_min_neighbors = 5;
    double ror_voxel_leaf_size = -1.0; // Отрицательное значение для авто-выбора

    if (!pnh.getParam("input_ply_path", input_ply_path)) {
        ROS_FATAL("Required parameter 'input_ply_path' is missing!");
        return 1;
    }
     if (!pnh.getParam("output_ply_path", output_ply_path)) {
         try {
             std::filesystem::path input_path(input_ply_path);
             std::string stem = input_path.stem().string(); 
             std::filesystem::path parent_path = input_path.parent_path(); 
             std::string parent_path_str = parent_path.string();
             if (parent_path_str.empty() || parent_path_str == ".") { parent_path_str = "."; }

             output_ply_path = parent_path_str + "/" + stem + "_filtered_ror.ply"; 
             ROS_WARN("Parameter 'output_ply_path' not specified. Defaulting to '%s'", output_ply_path.c_str());
         } catch (const std::exception& e) {
              ROS_ERROR("Error generating default output path using <filesystem>: %s. Please specify 'output_ply_path'.", e.what());
              return 1;
         }
    }


    pnh.param<std::string>("filter_type", filter_type_str, filter_type_str);
    pnh.param<bool>("overwrite_output", overwrite_output, overwrite_output);
    std::transform(filter_type_str.begin(), filter_type_str.end(), filter_type_str.begin(),
                   [](unsigned char c){ return std::tolower(c); });

    bool filter_enabled = true;
    if (filter_type_str == "ror") {
        pnh.param<double>("ror_radius_search", ror_radius_search, ror_radius_search);
        pnh.param<int>("ror_min_neighbors", ror_min_neighbors, ror_min_neighbors);
        pnh.param<double>("ror_voxel_leaf_size", ror_voxel_leaf_size, ror_voxel_leaf_size); 

        if (ror_voxel_leaf_size <= 0.0) {
            ror_voxel_leaf_size = ror_radius_search; 
            ROS_INFO("Parameter 'ror_voxel_leaf_size' not specified or non-positive. Auto-setting to ror_radius_search: %.4f", ror_voxel_leaf_size);
        }

        ROS_INFO("Filter type: ROR (Radius=%.3f, MinNeighbors=%d, VoxelLeafSize=%.4f)",
                 ror_radius_search, ror_min_neighbors, ror_voxel_leaf_size);

        if (ror_radius_search <= 0.0 || ror_min_neighbors < 0) {
             ROS_ERROR("ROR parameters invalid (ror_radius_search > 0, ror_min_neighbors >= 0).");
             return 1;
        }
    } else if (filter_type_str == "none") {
        filter_enabled = false;
        ROS_INFO("Filter type: None. Filtering disabled.");
    } else {
        filter_enabled = false;
        ROS_WARN("Unknown filter type '%s'. Disabling filtering. Valid types: 'ror', 'none'.", filter_type_str.c_str());
    }

    ROS_INFO("Input PLY path: %s", input_ply_path.c_str());
    ROS_INFO("Output PLY path: %s", output_ply_path.c_str());
    ROS_INFO("Overwrite output: %s", overwrite_output ? "true" : "false");


     try {
         if (!overwrite_output && std::filesystem::exists(output_ply_path)) {
             ROS_ERROR("Output file '%s' already exists and overwrite_output is set to false. Exiting.", output_ply_path.c_str());
             return 1;
         }
     } catch (const std::filesystem::filesystem_error& e) {
          ROS_ERROR("Filesystem error checking output path %s: %s", output_ply_path.c_str(), e.what());
     } catch (const std::exception& e) {
         ROS_ERROR("Generic error checking output path %s: %s", output_ply_path.c_str(), e.what());
         return 1; 
     }

    // --- Загрузка ---
    PointCloud cloud_raw;
    ROS_INFO("Loading input PLY file: %s", input_ply_path.c_str());
    if (!loadPlyFile(input_ply_path, cloud_raw)) {
        return 1;
    }
    ROS_INFO("Loaded %zu data points from %s", cloud_raw.size(), input_ply_path.c_str());
    if (cloud_raw.empty() && filter_enabled) {
        ROS_WARN("Input cloud is empty. Filtering will result in an empty cloud.");
    }


    // --- Фильтрация ---
    PointCloud cloud_filtered;
    if (filter_enabled && !cloud_raw.empty()) {
        ROS_INFO("Applying filter '%s'...", filter_type_str.c_str());
        try {
            if (filter_type_str == "ror") {
                cloud_filtered = applyRorFilterVoxelGrid(cloud_raw, ror_radius_search, ror_min_neighbors, ror_voxel_leaf_size);
            }

            ROS_INFO("Filtering complete. Points before: %zu, Points after: %zu",
                     cloud_raw.size(), cloud_filtered.size());
             if (cloud_filtered.empty() && !cloud_raw.empty()) {
                  ROS_WARN("Filtered point cloud is empty! This might be intended or due to aggressive filter parameters.");
             }
        } catch (const std::exception& e) {
             ROS_ERROR("Exception during filtering process: %s", e.what());
             return 1;
        } catch (...) {
             ROS_ERROR("Unknown exception during filtering process.");
             return 1;
        }
    } else {
        if (!filter_enabled) ROS_INFO("Filtering is disabled. Using raw cloud as output.");
        else ROS_INFO("Input cloud is empty. Using empty cloud as output.");
        cloud_filtered = cloud_raw; 
    }


    // --- Сохранение результата ---
    ROS_INFO("Saving %s point cloud (%zu points) to: %s",
             filter_enabled ? "filtered" : "raw",
             cloud_filtered.size(),
             output_ply_path.c_str());
    try {
         std::filesystem::path output_path_obj(output_ply_path);
         std::filesystem::path output_dir = output_path_obj.parent_path();
         if (!output_dir.empty() && !std::filesystem::exists(output_dir)) {
              ROS_INFO("Output directory '%s' does not exist. Attempting to create it.", output_dir.c_str());
              try {
                  if (!std::filesystem::create_directories(output_dir)) {
                       ROS_WARN("Failed to create output directory '%s'. Saving might fail if path is invalid or permissions are missing.", output_dir.c_str());
                  }
              } catch(const std::filesystem::filesystem_error& e) {
                  ROS_ERROR("Filesystem error creating directory '%s': %s. Saving might fail.", output_dir.c_str(), e.what());
              }
         }

        if (savePlyFile(output_ply_path, cloud_filtered)) {
            ROS_INFO("Successfully saved output PLY file.");
        } else {
            return 1;
        }
    } catch (const std::filesystem::filesystem_error& e) {
        ROS_ERROR("Filesystem exception while saving output PLY file %s: %s", output_ply_path.c_str(), e.what());
        return 1;
    } catch (const std::exception& e) {
        ROS_ERROR("Generic exception while saving output PLY file %s: %s", output_ply_path.c_str(), e.what());
        return 1;
    } catch (...) {
        ROS_ERROR("Unknown exception while saving output PLY file %s", output_ply_path.c_str());
        return 1;
    }

    ROS_INFO("PLY Filter Node finished successfully.");
    return 0;
}