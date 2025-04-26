#ifndef POINTCLOUD_UTILS_H
#define POINTCLOUD_UTILS_H

#include <vector>
#include <string>
#include <unordered_map>
#include <limits>
#include <utility>
#include <Eigen/Core>
#include <Eigen/Geometry> 

// --- Базовые типы данных ---
using Point = Eigen::Vector3d;
using PointCloud = std::vector<Point>;
using AlignedBoundingBox = Eigen::AlignedBox3d;

// --- Структуры Voxel Grid ---
struct VoxelIndex {
    int x, y, z;
    bool operator==(const VoxelIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelIndexHash {
    std::size_t operator()(const VoxelIndex& idx) const;
};

using VoxelGridMap = std::unordered_map<VoxelIndex, std::vector<size_t>, VoxelIndexHash>;

// --- Объявления функций ---

// Загрузка/сохранение PLY
bool loadPlyFile(const std::string& filename, PointCloud& cloud);
bool savePlyFile(const std::string& filename, const PointCloud& cloud);

// Bounding Box
AlignedBoundingBox calculateBoundingBoxEigen(const PointCloud& cloud);

// Voxel Grid
VoxelGridMap buildVoxelGrid(const PointCloud& cloud, double leaf_size, const AlignedBoundingBox& bbox);

// Поиск соседей
std::vector<std::pair<double, size_t>> findKNNSorted(
    const Point& query_point,
    int k,
    const PointCloud& cloud,
    const VoxelGridMap& grid,
    double inv_leaf_size,
    const AlignedBoundingBox& bbox,
    double max_dist_sq = std::numeric_limits<double>::max());

int countNeighborsVoxelGrid(
    size_t query_idx,
    const PointCloud& cloud,
    const VoxelGridMap& grid,
    double radius_search_sq,
    double inv_leaf_size,
    const AlignedBoundingBox& bbox);

inline double distance_sq(const Point& p1, const Point& p2) {
    return (p1 - p2).squaredNorm();
}


#endif // POINTCLOUD_UTILS_H