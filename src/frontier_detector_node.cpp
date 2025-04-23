#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

#include <vector>
#include <cmath>
#include <memory>
#include <mutex>
#include <algorithm>
#include <limits>
#include <numeric>       
#include <queue>         
#include <unordered_map> 
#include <tuple>        

// --- Структура для 3D индекса ячейки ---
struct VoxelIndex {
    int x, y, z;

    // Оператор сравнения для использования в std::map 
    bool operator<(const VoxelIndex& other) const {
        return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
    }

    // Оператор равенства для использования в std::unordered_map
    bool operator==(const VoxelIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// --- Хеш-функция для VoxelIndex (для std::unordered_map) ---
namespace std {
    template <>
    struct hash<VoxelIndex> {
        std::size_t operator()(const VoxelIndex& idx) const {
            size_t h1 = std::hash<int>{}(idx.x);
            size_t h2 = std::hash<int>{}(idx.y);
            size_t h3 = std::hash<int>{}(idx.z);
            size_t seed = 0;
            seed ^= h1 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= h2 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= h3 + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
} // namespace std


class FrontierDetector
{
public:
    FrontierDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh),
        tf_listener_(tf_buffer_),
        octree_(nullptr),
        processing_active_(false)
    {
        loadParams();
        setupCommunications();
        ROS_INFO("FrontierDetector node initialized successfully.");
    }


private:
    // --- Загрузка параметров ---
    void loadParams() {
        pnh_.param<std::string>("map_frame", map_frame_id_, "map");
        pnh_.param<std::string>("base_frame", base_frame_id_, "base_link");
        pnh_.param<std::string>("octomap_topic", octomap_topic_, "/octomap_full");

        pnh_.param<double>("altitude_tolerance", altitude_tolerance_, 0.2);
        pnh_.param<double>("search_radius_xy", search_radius_xy_, 10.0);
        pnh_.param<double>("update_rate", update_rate_, 1.0);

        pnh_.param<int>("min_free_neighbors", min_free_neighbors_, 2);
        pnh_.param<bool>("filter_by_obstacle_distance", filter_by_obstacle_distance_, true);
        pnh_.param<double>("min_obstacle_distance", min_obstacle_distance_, 0.3);

        pnh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.5);
        pnh_.param<int>("min_cluster_size", min_cluster_size_, 10);
        pnh_.param<int>("max_cluster_size", max_cluster_size_, std::numeric_limits<int>::max());

        pnh_.param<double>("frontier_marker_scale", frontier_marker_scale_, 0.1);
        pnh_.param<double>("centroid_marker_scale", centroid_marker_scale_, 0.3);

        ROS_INFO("Parameters loaded:");
        ROS_INFO("  Frames: map='%s', base='%s'", map_frame_id_.c_str(), base_frame_id_.c_str());
        ROS_INFO("  OctoMap topic: %s", octomap_topic_.c_str());
        ROS_INFO("  Search: alt_tolerance=%.2f, radius_xy=%.2f", altitude_tolerance_, search_radius_xy_);
        ROS_INFO("  Frontier criteria: min_free_neighbors=%d", min_free_neighbors_);
        ROS_INFO("  Obstacle filter: enabled=%s, min_dist=%.2f", filter_by_obstacle_distance_ ? "true" : "false", min_obstacle_distance_);
        ROS_INFO("  Clustering (Grid): tolerance=%.2f, min_size=%d, max_size=%d", cluster_tolerance_, min_cluster_size_, max_cluster_size_);
        ROS_INFO("  Markers: frontier_scale=%.2f, centroid_scale=%.2f", frontier_marker_scale_, centroid_marker_scale_);
    }

    // --- Настройка подписчиков и издателей ---
    void setupCommunications() {
        octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &FrontierDetector::octomapCallback, this);
        frontier_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("frontier_markers", 5);
        centroid_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("frontier_centroids", 5);
        process_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &FrontierDetector::processTimerCallback, this);
        ROS_INFO("Subscribers, publishers, and timer set up.");
    }

    // --- Обработчик OctoMap ---
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        if (msg->data.empty()) {
             ROS_WARN_THROTTLE(5.0, "Received empty OctoMap message. Ignoring.");
             return;
        }

        // Используем AbstractOcTree для большей гибкости, но кастуем к OcTree, т.к. нам нужны его методы
        std::shared_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::fullMsgToMap(*msg));
        if (!abstract_tree) {
            ROS_ERROR("Failed to deserialize OctoMap message!");
            return;
        }

        std::shared_ptr<octomap::OcTree> new_tree = std::dynamic_pointer_cast<octomap::OcTree>(abstract_tree);
        if (!new_tree) {
            ROS_ERROR("Deserialized map is not an OcTree! Type: %s", abstract_tree->getTreeType().c_str());
            return;
        }

        // Потокобезопасное обновление
        {
            std::lock_guard<std::mutex> lock(octree_mutex_);
            octree_ = new_tree;
        }

        if (!processing_active_) {
             processing_active_ = true;
             ROS_INFO("First OctoMap received. Frontier detection is now active.");
        }
    }

    // --- Основной цикл обработки по таймеру ---
    void processTimerCallback(const ros::TimerEvent& event) {
        if (!processing_active_) {
            return;
        }

        // 1. Получаем копию указателя на дерево и позу дрона
        std::shared_ptr<const octomap::OcTree> current_tree;
        octomap::point3d drone_pos;

        { // Блокируем только для копирования указателя
            std::lock_guard<std::mutex> lock(octree_mutex_);
            if (!octree_) {
                ROS_WARN_THROTTLE(2.0, "OctoMap is not available (null pointer). Skipping cycle.");
                return;
            }
            current_tree = octree_;
        }

        if (!getCurrentDronePose(drone_pos)) {
            ROS_WARN_THROTTLE(2.0, "Failed to get current drone pose. Skipping cycle.");
            return;
        }

        // 2. Находим сырые фронтиры
        std::vector<octomap::point3d> raw_frontiers;
        findFrontiers(current_tree, drone_pos, raw_frontiers);

        // 3. Фильтруем фронтиры по близости к препятствиям
        std::vector<octomap::point3d> filtered_frontiers = raw_frontiers;
        if (filter_by_obstacle_distance_) {
            filterFrontiersByObstacleDistance(current_tree, filtered_frontiers);
        }

        // 4. Кластеризуем и получаем ПАРЫ (центроид, размер)
        std::vector<std::pair<geometry_msgs::Point, int>> cluster_centroid_info;
        if (!filtered_frontiers.empty()) {
            clusterFrontiersGrid(filtered_frontiers, cluster_centroid_info); 
        } else {
            // ROS_INFO_THROTTLE(5.0, "No frontiers to cluster.");
        }

        // 5. Публикуем маркеры для визуализации
        publishMarkers<octomap::point3d>(filtered_frontiers, frontier_marker_pub_, "frontiers", frontier_marker_scale_, createColor(0.0, 1.0, 1.0, 0.8));
        publishCentroidMarkers(cluster_centroid_info, centroid_marker_pub_, "frontier_centroids", centroid_marker_scale_, createColor(1.0, 0.0, 0.0, 0.9));
    }

    // --- Получение текущей позы дрона ---
    bool getCurrentDronePose(octomap::point3d& pose) const {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(map_frame_id_, base_frame_id_, ros::Time(0), ros::Duration(0.5));
            pose.x() = static_cast<float>(transform_stamped.transform.translation.x);
            pose.y() = static_cast<float>(transform_stamped.transform.translation.y);
            pose.z() = static_cast<float>(transform_stamped.transform.translation.z);
            return true;
        } catch (const tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(5.0, "TF lookup failed from '%s' to '%s': %s", map_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
            return false;
        }
    }


    // --- Поиск Фронтиров ---
    // Использует 6-связность для соседей
    void findFrontiers(const std::shared_ptr<const octomap::OcTree>& tree,
                       const octomap::point3d& drone_pos,
                       std::vector<octomap::point3d>& frontiers) const
    {
        frontiers.clear();
        if (!tree) {
            ROS_ERROR("findFrontiers called with null OctoMap tree pointer!");
            return;
        }

        const float map_res = static_cast<float>(tree->getResolution()); 
        const float min_z = drone_pos.z() - static_cast<float>(altitude_tolerance_);
        const float max_z = drone_pos.z() + static_cast<float>(altitude_tolerance_);

        // Определяем область поиска 
        const octomap::point3d search_min(drone_pos.x() - static_cast<float>(search_radius_xy_),
                                          drone_pos.y() - static_cast<float>(search_radius_xy_),
                                          min_z);
        const octomap::point3d search_max(drone_pos.x() + static_cast<float>(search_radius_xy_),
                                          drone_pos.y() + static_cast<float>(search_radius_xy_),
                                          max_z);


        // Итерация по листовым вокселям в области поиска
        for (octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(search_min, search_max), end = tree->end_leafs_bbx(); it != end; ++it)
        {
            const octomap::point3d voxel_center = it.getCoordinate();

            // 1. Проверка высоты
            if (voxel_center.z() < min_z || voxel_center.z() > max_z) {
                continue;
            }

            // 2. Воксель должен быть известен и свободен
            if (!tree->isNodeOccupied(*it))
            {
                int unknown_neighbor_count = 0;
                int free_neighbor_count = 0;
                const octomap::OcTreeKey current_key = it.getKey();
                octomap::OcTreeKey neighbor_key;

                // Проверяем 6 прямых соседей
                static const int dx[] = {-1, 1, 0, 0, 0, 0};
                static const int dy[] = { 0, 0,-1, 1, 0, 0};
                static const int dz[] = { 0, 0, 0, 0,-1, 1};

                for (int i = 0; i < 6; ++i) {
                    neighbor_key[0] = current_key[0] + dx[i];
                    neighbor_key[1] = current_key[1] + dy[i];
                    neighbor_key[2] = current_key[2] + dz[i];

                    const octomap::OcTreeNode* neighbor_node = tree->search(neighbor_key);

                    if (neighbor_node == nullptr) {
                        unknown_neighbor_count++; // Сосед неизвестен
                    } else if (!tree->isNodeOccupied(neighbor_node)) {
                        free_neighbor_count++; // Сосед известен и свободен
                    }
                }

                // 3. Критерии фронтира: Есть неизвестный сосед И достаточно свободных соседей
                if (unknown_neighbor_count >= 1 && free_neighbor_count >= min_free_neighbors_) {
                    frontiers.push_back(voxel_center);
                }
            } // end if (free voxel)
        } // end for (voxel iteration)
    }

    // --- Фильтрация фронтиров по близости к препятствиям ---
    void filterFrontiersByObstacleDistance(const std::shared_ptr<const octomap::OcTree>& tree,
                                             std::vector<octomap::point3d>& frontiers) const
    {
        if (!tree || frontiers.empty()) {
            return;
        }
        const float min_obstacle_distance_float = static_cast<float>(min_obstacle_distance_);
        frontiers.erase(
            std::remove_if(frontiers.begin(), frontiers.end(),
                [&](const octomap::point3d& frontier_pt) {
                    // Удаляем точку, если она слишком близко к препятствию
                    return isCloseToObstacle(tree, frontier_pt, min_obstacle_distance_float);
                }),
            frontiers.end());
    }

    // --- Проверка близости точки к занятому вокселю (используем float) ---
    bool isCloseToObstacle(const std::shared_ptr<const octomap::OcTree>& tree,
                           const octomap::point3d& point,
                           float min_dist) const 
    {
        const float half_size = min_dist;
        const octomap::point3d search_min = point - octomap::point3d(half_size, half_size, half_size);
        const octomap::point3d search_max = point + octomap::point3d(half_size, half_size, half_size);
        const float min_dist_sq = min_dist * min_dist; // Сравниваем квадраты расстояний

        // Итерируем по листовым узлам в кубе поиска
        for (octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(search_min, search_max), end = tree->end_leafs_bbx(); it != end; ++it)
        {
            // Если воксель занят и его центр достаточно близок
            if (tree->isNodeOccupied(*it))
            {
                if ((it.getCoordinate() - point).norm_sq() <= min_dist_sq) {
                     return true; // Нашли близкое препятствие
                }
            }
        }
        return false; // Не нашли близких препятствий
    }

    // --- Кластеризация фронтиров с использованием пространственной сетки ---
    void clusterFrontiersGrid(const std::vector<octomap::point3d>& frontiers,
                              std::vector<std::pair<geometry_msgs::Point, int>>& centroid_info) const
    {
        centroid_info.clear();
        const size_t num_points = frontiers.size();
        if (num_points == 0 || cluster_tolerance_ <= 0.0) {
            return;
        }

        // 1. Определяем границы и параметры сетки
        octomap::point3d min_coord = frontiers[0];
        octomap::point3d max_coord = frontiers[0];
        for (size_t i = 1; i < num_points; ++i) {
            min_coord.x() = std::min(min_coord.x(), frontiers[i].x());
            min_coord.y() = std::min(min_coord.y(), frontiers[i].y());
            min_coord.z() = std::min(min_coord.z(), frontiers[i].z());
            max_coord.x() = std::max(max_coord.x(), frontiers[i].x());
            max_coord.y() = std::max(max_coord.y(), frontiers[i].y());
            max_coord.z() = std::max(max_coord.z(), frontiers[i].z());
        }

        // Используем float для внутренних вычислений кластеризации
        const float cell_size = static_cast<float>(cluster_tolerance_);
        const float inv_cell_size = 1.0f / cell_size;

        // Функция для получения индекса ячейки по координатам точки
        auto get_voxel_index = [&](const octomap::point3d& p) -> VoxelIndex {
            // Сдвигаем начало координат в min_coord и делим на размер ячейки
            return {
                static_cast<int>(std::floor((p.x() - min_coord.x()) * inv_cell_size)),
                static_cast<int>(std::floor((p.y() - min_coord.y()) * inv_cell_size)),
                static_cast<int>(std::floor((p.z() - min_coord.z()) * inv_cell_size))
            };
        };

        // 2. Создаем и заполняем сетку
        // Ключ - индекс вокселя, Значение - вектор индексов точек в этом вокселе
        std::unordered_map<VoxelIndex, std::vector<size_t>> grid_map;
        grid_map.reserve(num_points); 
        for (size_t i = 0; i < num_points; ++i) {
            grid_map[get_voxel_index(frontiers[i])].push_back(i);
        }

        // 3. Кластеризация
        std::vector<bool> visited(num_points, false);
        const float cluster_tolerance_sq = cell_size * cell_size; // Используем квадрат расстояния

        for (size_t i = 0; i < num_points; ++i) {
            if (!visited[i]) {
                std::queue<size_t> q; // Очередь для BFS (хранит индексы точек)
                std::vector<size_t> current_cluster_indices; // Индексы точек в текущем кластере

                q.push(i);
                visited[i] = true;
                current_cluster_indices.push_back(i);

                while (!q.empty()) {
                    size_t current_idx = q.front();
                    q.pop();
                    const octomap::point3d& current_point = frontiers[current_idx];
                    const VoxelIndex current_voxel_idx = get_voxel_index(current_point);

                    // Ищем соседей в кубе 3x3x3 ячеек вокруг текущей
                    for (int dx = -1; dx <= 1; ++dx) {
                        for (int dy = -1; dy <= 1; ++dy) {
                            for (int dz = -1; dz <= 1; ++dz) {
                                VoxelIndex neighbor_voxel_idx = {current_voxel_idx.x + dx, current_voxel_idx.y + dy, current_voxel_idx.z + dz};

                                // Проверяем, есть ли такая ячейка в карте
                                auto it = grid_map.find(neighbor_voxel_idx);
                                if (it != grid_map.end()) {
                                    // Итерируем по точкам в соседней (или текущей) ячейке
                                    for (size_t neighbor_point_idx : it->second) {
                                        // Проверяем, не посещена ли точка
                                        if (!visited[neighbor_point_idx]) {
                                            const octomap::point3d& neighbor_point = frontiers[neighbor_point_idx];
                                            // Проверяем фактическое расстояние
                                            if ((current_point - neighbor_point).norm_sq() <= cluster_tolerance_sq) {
                                                visited[neighbor_point_idx] = true;
                                                q.push(neighbor_point_idx);
                                                current_cluster_indices.push_back(neighbor_point_idx);
                                            }
                                        }
                                    } // end for points in neighbor cell
                                } // end if cell exists
                            } // dz
                        } // dy
                    } // dx
                } // Конец BFS для одного кластера

                // 4. Проверяем размер и сохраняем кластер
                int cluster_size = static_cast<int>(current_cluster_indices.size());
                if (cluster_size >= min_cluster_size_ && cluster_size <= max_cluster_size_) {
                    octomap::point3d centroid_sum(0.0f, 0.0f, 0.0f);
                    for (size_t idx : current_cluster_indices) {
                        centroid_sum += frontiers[idx];
                    }

                    // Конвертируем в double для geometry_msgs::Point
                    geometry_msgs::Point centroid_msg;
                    centroid_msg.x = static_cast<double>(centroid_sum.x() / cluster_size);
                    centroid_msg.y = static_cast<double>(centroid_sum.y() / cluster_size);
                    centroid_msg.z = static_cast<double>(centroid_sum.z() / cluster_size);

                    // Сохраняем пару (центроид, размер)
                    centroid_info.emplace_back(centroid_msg, cluster_size);
                }
            } // end if (!visited[i])
        } // end for (points iteration)
    }


    // --- Публикация маркеров ---
    template<typename PointType>
    void publishMarkers(const std::vector<PointType>& points,
        const ros::Publisher& pub,
        const std::string& ns,
        double scale,
        const std_msgs::ColorRGBA& color) const
    {
        if (pub.getNumSubscribers() == 0) return;

        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;

        // Маркер для очистки предыдущих маркеров этого namespace
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = map_frame_id_;
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = ns;
        clear_marker.id = marker_id++; // Используем id 0 для очистки
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        if (!points.empty()) {
            marker_array.markers.reserve(points.size() + 1); // +1 для clear_marker
            for (const auto& point_data : points) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = map_frame_id_;
                marker.header.stamp = ros::Time::now();
                marker.ns = ns;
                marker.id = marker_id++;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;

                assignMarkerPosition(point_data, marker.pose.position);

                marker.pose.orientation.w = 1.0; 
                marker.scale.x = scale;
                marker.scale.y = scale;
                marker.scale.z = scale;
                marker.color = color;
                marker.lifetime = ros::Duration();

                marker_array.markers.push_back(marker);
            }
        }
        pub.publish(marker_array);
    }


    // Метод для присваивания позиции маркера
    void assignMarkerPosition(const octomap::point3d& p, geometry_msgs::Point& gp) const {
        gp.x = static_cast<double>(p.x());
        gp.y = static_cast<double>(p.y());
        gp.z = static_cast<double>(p.z());
    }

    // --- Вспомогательная функция для создания цвета ---
    std_msgs::ColorRGBA createColor(float r, float g, float b, float a) const {
        std_msgs::ColorRGBA color;
        color.r = r; color.g = g; color.b = b; color.a = a;
        return color;
    }

    // --- Публикация маркеров центроидов ---
    void publishCentroidMarkers(const std::vector<std::pair<geometry_msgs::Point, int>>& centroid_info,
        const ros::Publisher& pub,
        const std::string& ns,
        double scale_xy, 
        const std_msgs::ColorRGBA& color) const
    {
        if (pub.getNumSubscribers() == 0) return;

        visualization_msgs::MarkerArray marker_array;
        int marker_id = 0;

        // Очистка старых маркеров
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = map_frame_id_;
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = ns;
        clear_marker.id = marker_id++; // ID 0
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        if (!centroid_info.empty()) {
            marker_array.markers.reserve(centroid_info.size() + 1);
            for (const auto& info_pair : centroid_info) {
                const geometry_msgs::Point& centroid = info_pair.first;
                const int cluster_size = info_pair.second;

                visualization_msgs::Marker marker;
                marker.header.frame_id = map_frame_id_;
                marker.header.stamp = ros::Time::now();
                marker.ns = ns;
                marker.id = marker_id++;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;

                marker.pose.position = centroid;
                marker.pose.orientation.w = 1.0;

                marker.scale.x = scale_xy; 
                marker.scale.y = scale_xy;
                marker.scale.z = static_cast<double>(cluster_size); 

                marker.color = color;
                marker.lifetime = ros::Duration();

                marker_array.markers.push_back(marker);
            }
        }
        pub.publish(marker_array);
    }


    // --- Переменные класса ---
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Коммуникации
    ros::Subscriber octomap_sub_;
    ros::Publisher frontier_marker_pub_;
    ros::Publisher centroid_marker_pub_;
    ros::Timer process_timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Состояние
    std::mutex octree_mutex_; 
    std::shared_ptr<octomap::OcTree> octree_;
    bool processing_active_; 

    // Параметры 
    std::string map_frame_id_;
    std::string base_frame_id_;
    std::string octomap_topic_;
    double altitude_tolerance_;
    double search_radius_xy_;
    double update_rate_;
    int min_free_neighbors_;
    bool filter_by_obstacle_distance_;
    double min_obstacle_distance_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;
    double frontier_marker_scale_;
    double centroid_marker_scale_;
};

// --- main ---
int main(int argc, char** argv)
{
    ros::init(argc, argv, "frontier_detector_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); 

    try {
        FrontierDetector detector(nh, pnh);
        ros::spin(); 
    } catch (const std::exception& e) {
        ROS_FATAL("Unhandled exception in FrontierDetector: %s", e.what());
        return 1;
    } catch (...) {
         ROS_FATAL("Unknown unhandled exception in FrontierDetector");
         return 1;
    }
    return 0;
}