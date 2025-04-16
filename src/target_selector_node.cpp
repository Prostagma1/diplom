#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

// OctoMap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <vector>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <limits>
#include <string>
#include <algorithm>

struct FrontierCentroidInfo {
    geometry_msgs::Point point;
    int size;
};
class TargetLocationValidator
{
public:
    TargetLocationValidator(double drone_radius) :
        octree_(nullptr),
        drone_radius_(drone_radius),
        current_altitude_(0.0) 
    {
         if (drone_radius_ <= 0) {
             ROS_WARN("[TargetValidator] Drone safety radius is non-positive (%.2f).", drone_radius_);
         }
    }

    // Проверяет, свободна ли область вокруг точки на заданной высоте
    bool isLocationValid(const geometry_msgs::Point& point) const {
        if (!octree_) {
            ROS_ERROR_THROTTLE(5.0,"[TargetValidator] OctoMap is null! Cannot check validity.");
            return false; // Невалидно, если карты нет
        }

        const double x = point.x;
        const double y = point.y;
        const double z = current_altitude_;

        octomap::point3d center(x, y, z);
        double check_radius = drone_radius_;
        octomap::point3d min_bbx = center - octomap::point3d(check_radius, check_radius, check_radius);
        octomap::point3d max_bbx = center + octomap::point3d(check_radius, check_radius, check_radius);

        for (octomap::OcTree::leaf_bbx_iterator it = octree_->begin_leafs_bbx(min_bbx, max_bbx),
                                              end = octree_->end_leafs_bbx(); it != end; ++it)
        {
            if (octree_->isNodeOccupied(*it)) {
                ROS_DEBUG("[TargetValidator] Collision detected near target (%.2f, %.2f, %.2f)", x, y, z);
                return false;
            }
        }
        ROS_DEBUG("[TargetValidator] Target location (%.2f, %.2f, %.2f) is valid.", x, y, z);
        return true;
    }

    void updateOctoMap(std::shared_ptr<const octomap::OcTree> tree) {
        octree_ = tree;
        ROS_DEBUG("[TargetValidator] Updated OctoMap pointer.");
    }

    void setCurrentAltitude(double altitude) {
        current_altitude_ = altitude;
         ROS_DEBUG("[TargetValidator] Updated validation altitude to %.2f", current_altitude_);
    }

private:
    std::shared_ptr<const octomap::OcTree> octree_;
    double drone_radius_;
    double current_altitude_;
};


// --- Основной класс выбора цели ---
class TargetSelector
{
public:
    TargetSelector(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh),
        tf_listener_(tf_buffer_),
        octree_(nullptr),
        have_map_(false),
        have_centroids_(false),
        target_validator_(0.0) 
    {
        loadParams(); 
        target_validator_ = TargetLocationValidator(drone_radius_); 
        setupCommunications(); 
        ROS_INFO("Target Selector node initialized.");
    }

private:
void loadParams() {
    pnh_.param<std::string>("map_frame", map_frame_id_, "map");
    pnh_.param<std::string>("base_frame", base_frame_id_, "base_link");
    pnh_.param<std::string>("octomap_topic", octomap_topic_, "/octomap_full");
    pnh_.param<std::string>("centroids_topic", centroids_topic_, "/frontier_centroids");
    pnh_.param<std::string>("selected_target_topic", selected_target_topic_, "/selected_target_point");

    pnh_.param<double>("drone_radius", drone_radius_, 0.3); 
    pnh_.param<double>("update_rate", update_rate_, 0.5);
    pnh_.param<double>("min_target_distance", min_target_distance_, 0.5);
    pnh_.param<double>("inf_gain_box_size", inf_gain_box_size_, 2.0); 
    pnh_.param<double>("target_reached_threshold", target_reached_threshold_, 0.3);
    pnh_.param<double>("score_improvement_factor", score_improvement_factor_, 1.5);

    // --- Веса для единой формулы оценки ---
    pnh_.param<double>("score_weight_size", score_weight_size_, 500.0);        // Вес для размера кластера
    pnh_.param<double>("score_weight_unknown", score_weight_unknown_, 200.0);  // Вес для доли неизв. вокселей
    pnh_.param<double>("score_weight_distance", score_weight_distance_, 1.0);  // Вес для расстояния (отрицательный вклад)

    ROS_INFO("Target Selector Parameters:");
    ROS_INFO("  Frames: map='%s', base='%s'", map_frame_id_.c_str(), base_frame_id_.c_str());
    ROS_INFO("  Topics: octomap='%s', centroids='%s', selected_target='%s'",
             octomap_topic_.c_str(), centroids_topic_.c_str(), selected_target_topic_.c_str());
    ROS_INFO("  Drone radius: %.2f m", drone_radius_);
    ROS_INFO("  Update rate: %.2f Hz", update_rate_);
    ROS_INFO("  Target selection: min_dist=%.2f, reached_thresh=%.2f, score_improv=%.2f",
             min_target_distance_, target_reached_threshold_, score_improvement_factor_);
    ROS_INFO("  Scoring Weights: size=%.1f, unknown=%.1f, distance=%.1f",
             score_weight_size_, score_weight_unknown_, score_weight_distance_); 
    ROS_INFO("  InfoGain Box Size: %.2f", inf_gain_box_size_);

    // Проверка корректности весов, должны быть >= 0
    if (score_weight_size_ < 0 || score_weight_unknown_ < 0 || score_weight_distance_ < 0) {
        ROS_WARN("Score weights should be non-negative. Using absolute values.");
        score_weight_size_ = std::abs(score_weight_size_);
        score_weight_unknown_ = std::abs(score_weight_unknown_);
        score_weight_distance_ = std::abs(score_weight_distance_);
    }
     if (score_weight_distance_ == 0 && score_weight_unknown_ == 0 && score_weight_size_ > 0) {
        ROS_WARN("Only cluster size affects the score. Distance and unknown ratio are ignored.");
    } else if (score_weight_distance_ == 0 && score_weight_size_ == 0 && score_weight_unknown_ > 0) {
        ROS_WARN("Only unknown ratio affects the score. Size and distance are ignored.");
    }
}

    void setupCommunications() {
        centroids_sub_ = nh_.subscribe(centroids_topic_, 5, &TargetSelector::centroidsCallback, this);
        octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &TargetSelector::octomapCallback, this);
        selected_target_pub_ = nh_.advertise<geometry_msgs::PointStamped>(selected_target_topic_, 1); 
        visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("target_selection_visualization", 5); 
        selection_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &TargetSelector::selectionTimerCallback, this);
        ROS_INFO("Target Selector subscribers, publisher, and timer set up.");
    }

    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
        ROS_DEBUG("TargetSelector: Received OctoMap message.");
        if (msg->data.empty()) {
            ROS_WARN("TargetSelector: Received empty OctoMap data. Ignoring.");
            return;
        }

        octomap::AbstractOcTree* abstract_tree_raw = octomap_msgs::fullMsgToMap(*msg);
        if (!abstract_tree_raw) {
            ROS_ERROR("TargetSelector: Failed to deserialize OctoMap message (fullMsgToMap returned nullptr)!");
            return;
        }

        std::shared_ptr<octomap::AbstractOcTree> abstract_tree_ptr(abstract_tree_raw);

        std::shared_ptr<octomap::OcTree> new_tree = std::dynamic_pointer_cast<octomap::OcTree>(abstract_tree_ptr);

        if (!new_tree) {
            ROS_ERROR("TargetSelector: Deserialized map is not an OcTree! Type: %s",
                      abstract_tree_ptr->getTreeType().c_str());
            return;
        }

        ROS_DEBUG("TargetSelector: Successfully deserialized OctoMap (Resolution: %.3f m).", new_tree->getResolution());

        {
            std::lock_guard<std::mutex> lock(octree_mutex_);
            octree_ = new_tree;
            target_validator_.updateOctoMap(octree_);
            have_map_ = true;
        }
   }

    void centroidsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(centroids_mutex_);
        latest_centroids_.clear();
        latest_centroids_.reserve(msg->markers.size());
        for (const auto& marker : msg->markers) {
             if (marker.action == visualization_msgs::Marker::ADD &&
                 marker.type == visualization_msgs::Marker::SPHERE &&
                 marker.ns == "frontier_centroids")
             {
                FrontierCentroidInfo info;
                info.point = marker.pose.position;
                info.size = static_cast<int>(std::round(marker.scale.z));
                if (info.size < 0) info.size = 0;
                latest_centroids_.push_back(info);
            }
        }
        have_centroids_ = !latest_centroids_.empty();
        ROS_DEBUG("Received %zu valid frontier centroids.", latest_centroids_.size());
    }

    // --- Логика выбора цели ---
    void selectionTimerCallback(const ros::TimerEvent& event)
    {
        // 0. Проверка карты
        std::shared_ptr<const octomap::OcTree> current_tree;
        double octree_resolution = 0.0;
        {
            std::lock_guard<std::mutex> lock_o(octree_mutex_);
            if (!octree_ || !have_map_) {
                ROS_DEBUG_THROTTLE(5.0, "TargetSelector: Waiting for OctoMap...");
                clearVisualization();
                active_target_.reset();
                active_target_score_ = -1.0;
                return;
            }
            current_tree = octree_;
            octree_resolution = current_tree->getResolution();
        }
         if (octree_resolution <= 0) {
             ROS_ERROR_THROTTLE(5.0, "TargetSelector: Invalid OctoMap resolution (%.3f).", octree_resolution);
             return;
         }

        // 1. Получить текущую позу дрона
        geometry_msgs::PoseStamped current_pose_stamped;
        if (!getCurrentDronePose(current_pose_stamped)) {
            ROS_WARN_THROTTLE(2.0, "TargetSelector: Cannot get current drone pose.");
             clearVisualization();
             active_target_.reset();
             active_target_score_ = -1.0;
            return;
        }
        // Обновляем высоту в валидаторе для проверок
        target_validator_.setCurrentAltitude(current_pose_stamped.pose.position.z);

        // === Логика Выбора/Сохранения/Смены Цели ===
        bool publish_target = false; 

        // --- Проверка текущей активной цели ---
        if (active_target_.has_value()) {
            ROS_DEBUG("TargetSelector: Checking active target (%.2f, %.2f) with score %.4f",
                      active_target_->x, active_target_->y, active_target_score_);
            geometry_msgs::Point current_active_target = *active_target_;

            // Проверяем валидность местоположения цели с учетом текущей карты и радиуса дрона
            bool target_still_valid = target_validator_.isLocationValid(current_active_target);

            double dx = current_active_target.x - current_pose_stamped.pose.position.x;
            double dy = current_active_target.y - current_pose_stamped.pose.position.y;
            double dist_sq_to_target = dx * dx + dy * dy;
            bool target_reached = dist_sq_to_target < (target_reached_threshold_ * target_reached_threshold_);

            if (target_still_valid && !target_reached) {
                // Цель валидна и не достигнута. Она остается активной.
                publish_target = true; // Публикуем ее
                ROS_DEBUG("TargetSelector: Active target is still valid and not reached.");
            } else {
                // Цель стала невалидной или достигнута. Сбрасываем.
                ROS_INFO("TargetSelector: Active target (%.2f, %.2f) is now invalid (%d) or reached (%d). Resetting.",
                         current_active_target.x, current_active_target.y, !target_still_valid, target_reached);
                active_target_.reset();
                active_target_score_ = -1.0;
                publish_target = false; // Не публикуем сброшенную цель
            }
        } else {
            ROS_DEBUG("TargetSelector: No active target.");
            publish_target = false;
        }

        // --- Поиск и проверка новых кандидатов ---
        std::vector<FrontierCentroidInfo> local_centroids;
        {
            std::lock_guard<std::mutex> lock_c(centroids_mutex_);
             if (latest_centroids_.empty() && !active_target_.has_value()) { // Ждем только если совсем нечего делать
                ROS_DEBUG_THROTTLE(5.0, "TargetSelector: No active target and no new centroids.");
                 clearVisualization(); // Очищаем, если нет ни активной, ни кандидатов
                return;
            }
            local_centroids = latest_centroids_;
        }


        std::optional<std::pair<FrontierCentroidInfo, double>> best_new_candidate_opt;
        if (!local_centroids.empty()) {
            best_new_candidate_opt = findBestScoringCandidate(
                current_pose_stamped.pose.position, local_centroids, current_tree, octree_resolution);
        }

        if (best_new_candidate_opt) {
            const auto& best_new_info = best_new_candidate_opt->first;
            const double best_new_score = best_new_candidate_opt->second;
            ROS_DEBUG("TargetSelector: Best potential new candidate (%.2f, %.2f), score %.4f",
                     best_new_info.point.x, best_new_info.point.y, best_new_score);

            if (active_target_.has_value()) {
                // --- Сравнение с активной целью ---
                if (best_new_score > active_target_score_ * score_improvement_factor_) {
                    ROS_INFO("TargetSelector: Switching target! New score %.4f is > %.2f * current score %.4f",
                             best_new_score, score_improvement_factor_, active_target_score_);
                    active_target_ = best_new_info.point;
                    active_target_score_ = best_new_score;
                    publish_target = true; // Публикуем новую цель
                } else {
                    // Новый кандидат не достаточно хорош, остаемся на старой цели
                    ROS_DEBUG("TargetSelector: New candidate score %.4f not significantly better than current %.4f.",
                              best_new_score, active_target_score_);
                }
            } else {
                // --- Активной цели не было, выбираем лучшего нового ---
                ROS_INFO("TargetSelector: Selected *new* active target (%.2f, %.2f) with score %.4f",
                         best_new_info.point.x, best_new_info.point.y, best_new_score);
                active_target_ = best_new_info.point;
                active_target_score_ = best_new_score;
                publish_target = true; // Публикуем новую цель
            }
        } else if (!active_target_.has_value()) {
             // Активной цели не было и не нашли новых подходящих кандидатов
             ROS_INFO("TargetSelector: No active target and no suitable new candidates found.");
             clearVisualization(); // Очистить старую визуализацию
        }

        // --- Публикация выбранной цели ---
        if (publish_target && active_target_.has_value()) {
            geometry_msgs::PointStamped target_msg;
            target_msg.header.stamp = ros::Time::now();
            target_msg.header.frame_id = map_frame_id_;
            target_msg.point = *active_target_;
            selected_target_pub_.publish(target_msg);
            ROS_DEBUG("TargetSelector: Published target (%.2f, %.2f)", active_target_->x, active_target_->y);
            publishVisualization(local_centroids, active_target_); // Показываем кандидатов и выбранную цель
        } else if (!active_target_.has_value()) {
             clearVisualization();
        } else {
            publishVisualization(local_centroids, active_target_); // Показываем кандидатов и текущую цель
        }

    }

    // --- Вспомогательные функции ---
     double calculateUnknownVolumeRatio(
         const std::shared_ptr<const octomap::OcTree>& tree,
         const octomap::point3d& center,
         double resolution,
         double box_half_size) const
     {
        if (!tree || resolution <= 0 || box_half_size <= 0) return 0.0;
        octomap::point3d min_point = center - octomap::point3d(box_half_size, box_half_size, box_half_size);
        octomap::point3d max_point = center + octomap::point3d(box_half_size, box_half_size, box_half_size);
        unsigned long long unknown_count = 0;
        unsigned long long total_count = 0;
        for (double ix = min_point.x(); ix < max_point.x(); ix += resolution) {
            for (double iy = min_point.y(); iy < max_point.y(); iy += resolution) {
                for (double iz = min_point.z(); iz < max_point.z(); iz += resolution) {
                    total_count++;
                    if (tree->search(ix, iy, iz) == nullptr) {
                        unknown_count++;
                    }
                }
            }
        }
        if (total_count == 0) return 0.0;
        return static_cast<double>(unknown_count) / static_cast<double>(total_count);
     }

     std::optional<std::pair<FrontierCentroidInfo, double>> findBestScoringCandidate(
        const geometry_msgs::Point& current_pos,
        const std::vector<FrontierCentroidInfo>& candidates,
        const std::shared_ptr<const octomap::OcTree>& tree,
        double resolution) const
    {
        if (candidates.empty() || !tree || resolution <= 0) return std::nullopt;

        std::optional<FrontierCentroidInfo> best_candidate_info = std::nullopt;
        double max_score = -std::numeric_limits<double>::infinity(); // Инициализируем минимально возможным значением

        octomap::point3d current_pos_octomap(current_pos.x, current_pos.y, current_pos.z);
        double box_half_size = inf_gain_box_size_ / 2.0;

        ROS_DEBUG("Evaluating %zu candidates using weighted formula...", candidates.size());

        for (const auto& candidate_info : candidates) {
            const geometry_msgs::Point& centroid_msg = candidate_info.point;
            octomap::point3d centroid_octomap(centroid_msg.x, centroid_msg.y, centroid_msg.z);
            int current_size = candidate_info.size;

            // 1. Проверка валидности местоположения
            if (!target_validator_.isLocationValid(centroid_msg)) {
                 ROS_DEBUG("Candidate (%.2f, %.2f) rejected: location invalid.", centroid_msg.x, centroid_msg.y);
                continue;
            }

            // 2. Расчет расстояния и проверка минимального порога
            double current_distance = current_pos_octomap.distance(centroid_octomap);
            if (current_distance < min_target_distance_) {
                ROS_DEBUG("Candidate (%.2f, %.2f) rejected: too close (dist=%.2f < min=%.2f).", centroid_msg.x, centroid_msg.y, current_distance, min_target_distance_);
                continue;
            }

            // 3. Расчет коэффициента неизвестных вокселей
            double current_unknown_ratio = calculateUnknownVolumeRatio(
                tree, centroid_octomap, resolution, box_half_size);

            // --- 4. Расчет единого score по взвешенной формуле ---
            // Score = W_size * size + W_unknown * unknown_ratio - W_distance * distance
            double current_score = score_weight_size_ * static_cast<double>(current_size)
                                 + score_weight_unknown_ * current_unknown_ratio
                                 - score_weight_distance_ * current_distance;

            ROS_DEBUG("Candidate (%.2f, %.2f): size=%d, ratio=%.4f, dist=%.2f -> score=%.4f",
                      centroid_msg.x, centroid_msg.y, current_size, current_unknown_ratio, current_distance, current_score);

            // 5. Сравнение с максимальным найденным score
            if (current_score > max_score) {
                ROS_DEBUG("  New best score found (%.4f > %.4f)", current_score, max_score);
                max_score = current_score;
                best_candidate_info = candidate_info;
            }
        } // --- Конец цикла по кандидатам ---

        // 6. Возвращение результата
        if (best_candidate_info) {
             ROS_DEBUG("Best candidate selected: (%.2f, %.2f) with score %.4f",
                       best_candidate_info->point.x, best_candidate_info->point.y, max_score);
            return std::make_pair(*best_candidate_info, max_score);
        } else {
             ROS_DEBUG("No suitable candidate found after evaluation.");
            return std::nullopt;
        }
    }
    bool getCurrentDronePose(geometry_msgs::PoseStamped& pose_stamped) const
    {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(map_frame_id_, base_frame_id_, ros::Time(0), ros::Duration(0.1));
            pose_stamped.header.stamp = transform_stamped.header.stamp;
            pose_stamped.header.frame_id = map_frame_id_;
            pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
            pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
            pose_stamped.pose.position.z = transform_stamped.transform.translation.z;
            pose_stamped.pose.orientation = transform_stamped.transform.rotation;
            return true;
        } catch (const tf2::TransformException &ex) {
            ROS_WARN("TF lookup failed from '%s' to '%s': %s", map_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
            return false;
        }
    }

    void publishVisualization(const std::vector<FrontierCentroidInfo>& candidates,
                               const std::optional<geometry_msgs::Point>& active_target_opt)
    {
        if (visualization_pub_.getNumSubscribers() == 0) return;

        visualization_msgs::MarkerArray marker_array;
        ros::Time now = ros::Time::now();
        int id_counter = 0;

        // 1. Очистка предыдущих маркеров (кроме ID 0 - DELETEALL)
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = map_frame_id_;
        clear_marker.header.stamp = now;
        clear_marker.ns = "target_selector";
        clear_marker.id = id_counter++; // ID 0
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        // 2. Маркеры кандидатов (серые)
        for (const auto& candidate_info : candidates) {
            visualization_msgs::Marker candidate_marker;
            candidate_marker.header.frame_id = map_frame_id_;
            candidate_marker.header.stamp = now;
            candidate_marker.ns = "candidates";
            candidate_marker.id = id_counter++;
            candidate_marker.type = visualization_msgs::Marker::SPHERE;
            candidate_marker.action = visualization_msgs::Marker::ADD;
            candidate_marker.pose.position = candidate_info.point;
            candidate_marker.pose.position.z += 0.05;
            candidate_marker.pose.orientation.w = 1.0;
            candidate_marker.scale.x = 0.8;
            candidate_marker.scale.y = 0.8;
            candidate_marker.scale.z = 0.8;
            candidate_marker.color = createColor(0.5, 0.5, 0.5, 0.7); // Серый
            candidate_marker.lifetime = ros::Duration(1.0 / update_rate_ * 1.5);
            marker_array.markers.push_back(candidate_marker);
        }

        // 3. Маркер выбранной цели 
        if (active_target_opt.has_value()) {
            visualization_msgs::Marker target_marker;
            target_marker.header.frame_id = map_frame_id_;
            target_marker.header.stamp = now;
            target_marker.ns = "target_selector";
            target_marker.id = id_counter++; 
            target_marker.type = visualization_msgs::Marker::SPHERE;
            target_marker.action = visualization_msgs::Marker::ADD;
            target_marker.pose.position = *active_target_opt;
            geometry_msgs::PoseStamped current_pose;
            if (getCurrentDronePose(current_pose)) {
                 target_marker.pose.position.z = current_pose.pose.position.z;
            } else {
                 target_marker.pose.position.z = active_target_opt->z;
            }
            target_marker.pose.orientation.w = 1.0;
            target_marker.scale.x = 1;
            target_marker.scale.y = 1;
            target_marker.scale.z = 1;
            target_marker.color = createColor(1.0, 1.0, 0.0, 0.9); // Желтый
            target_marker.lifetime = ros::Duration(0); 
            marker_array.markers.push_back(target_marker);
        }

        visualization_pub_.publish(marker_array);
    }

     void clearVisualization() {
         if (visualization_pub_.getNumSubscribers() == 0) return;
         visualization_msgs::MarkerArray marker_array;
         visualization_msgs::Marker clear_marker;
         clear_marker.header.frame_id = map_frame_id_;
         clear_marker.header.stamp = ros::Time::now();
         clear_marker.ns = "target_selector"; 
         clear_marker.id = 0;
         clear_marker.action = visualization_msgs::Marker::DELETEALL;
         marker_array.markers.push_back(clear_marker);
         visualization_pub_.publish(marker_array);
    }

    std_msgs::ColorRGBA createColor(float r, float g, float b, float a) const {
        std_msgs::ColorRGBA color;
        color.r = r; color.g = g; color.b = b; color.a = a;
        return color;
    }

    // --- Переменные класса ---
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Коммуникации
    ros::Subscriber centroids_sub_;
    ros::Subscriber octomap_sub_;
    ros::Publisher selected_target_pub_; 
    ros::Publisher visualization_pub_;
    ros::Timer selection_timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Состояние карты и центроидов
    std::mutex octree_mutex_;
    std::shared_ptr<const octomap::OcTree> octree_; 
    bool have_map_;

    std::mutex centroids_mutex_;
    std::vector<FrontierCentroidInfo> latest_centroids_;
    bool have_centroids_;

    // Состояние выбора цели
    std::optional<geometry_msgs::Point> active_target_;
    double active_target_score_ = -1.0;
    TargetLocationValidator target_validator_;

    // Параметры
    std::string map_frame_id_;
    std::string base_frame_id_;
    std::string octomap_topic_;
    std::string centroids_topic_;
    std::string selected_target_topic_;

    double drone_radius_;
    double update_rate_;
    double min_target_distance_;
    double target_reached_threshold_;
    double score_improvement_factor_;
    double inf_gain_box_size_;

    // Новые параметры весов для единой формулы
    double score_weight_size_;
    double score_weight_unknown_;
    double score_weight_distance_;
};

// --- main ---
int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_selector_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        TargetSelector selector(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Unhandled exception in TargetSelector: %s", e.what());
        return 1;
    } catch (...) {
        ROS_FATAL("Unknown unhandled exception in TargetSelector");
        return 1;
    }
    return 0;
}