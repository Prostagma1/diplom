#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

// OctoMap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// OMPL
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/config.h>
#include <ompl/util/Console.h>

#include <vector>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// --- Класс OctoMapValidityChecker ---
class OctoMapValidityChecker : public ob::StateValidityChecker {
public:
    OctoMapValidityChecker(const ob::SpaceInformationPtr &si,
                           std::shared_ptr<const octomap::OcTree> tree,
                           double drone_radius,
                           double planning_altitude) :
        ob::StateValidityChecker(si),
        octree_(tree),
        drone_radius_(drone_radius),
        planning_altitude_(planning_altitude)
    {
        if (drone_radius_ <= 0) {
             ROS_WARN("[ValidityChecker] Drone safety radius is non-positive (%.2f).", drone_radius_);
        }
    }

    bool isValid(const ob::State *state) const override
    {
        if (!octree_) {
             ROS_ERROR_THROTTLE(5.0,"[ValidityChecker] OctoMap is null! Cannot check state validity.");
             return false;
        }
        const auto *state_se2 = state->as<ob::SE2StateSpace::StateType>();
        const double x = state_se2->getX();
        const double y = state_se2->getY();
        const double z = planning_altitude_;

        octomap::point3d center(x, y, z);
        double check_radius = drone_radius_;
        octomap::point3d min_bbx = center - octomap::point3d(check_radius, check_radius, check_radius);
        octomap::point3d max_bbx = center + octomap::point3d(check_radius, check_radius, check_radius);

        // --- ДОПУЩЕНИЕ: Разрешаем до N занятых вокселей ---
        int occupied_voxel_count = 0;
        // Максимальное количество занятых вокселей, при котором состояние всё ещё считается валидным
        const int max_allowed_occupied_voxels = 3;
        // --- Конец допущения ---

        for (octomap::OcTree::leaf_bbx_iterator it = octree_->begin_leafs_bbx(min_bbx, max_bbx),
                                              end = octree_->end_leafs_bbx(); it != end; ++it)
        {
            if (octree_->isNodeOccupied(*it))
            {
                occupied_voxel_count++;
                // Оптимизация: если количество занятых вокселей уже превысило порог,
                // то дальнейшая проверка не имеет смысла, состояние невалидно.
                if (occupied_voxel_count > max_allowed_occupied_voxels) {
                     ROS_DEBUG("[ValidityChecker] Collision threshold exceeded (%d > %d) at state (%.2f, %.2f, %.2f)",
                               occupied_voxel_count, max_allowed_occupied_voxels, x, y, z);
                    return false; // Состояние невалидно
                }
            }
        }

        // Если цикл завершился, проверяем итоговое количество найденных занятых вокселей.
        // Если оно не превышает порог, состояние считается валидным.
        if (occupied_voxel_count > max_allowed_occupied_voxels) {
             // Эта ветка сработает, только если оптимизация выше была убрана
             ROS_DEBUG("[ValidityChecker] Collision: %d occupied voxels found (max allowed: %d) at state (%.2f, %.2f, %.2f).",
                       occupied_voxel_count, max_allowed_occupied_voxels, x, y, z);
            return false;
        } else {
            // Состояние валидно, даже если нашли 1, 2 или 3 занятых вокселя
             ROS_DEBUG("[ValidityChecker] State (%.2f, %.2f, %.2f) is considered valid (%d occupied voxels found, max allowed: %d).",
                       x, y, z, occupied_voxel_count, max_allowed_occupied_voxels);
            return true;
        }
    }


    void updateOctoMap(std::shared_ptr<const octomap::OcTree> tree) {
        octree_ = tree;
        ROS_DEBUG("[ValidityChecker] Updated OctoMap pointer.");
    }

    void setPlanningAltitude(double altitude) {
        planning_altitude_ = altitude;
        ROS_DEBUG("[ValidityChecker] Updated planning altitude to %.2f", planning_altitude_);
    }

private:
    std::shared_ptr<const octomap::OcTree> octree_;
    double drone_radius_;
    double planning_altitude_;
};


// --- Основной класс планировщика пути ---
class PathPlanner
{
public:
    PathPlanner(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh),
        tf_listener_(tf_buffer_),
        octree_(nullptr),
        have_map_(false),
        has_active_target_(false),
        current_planning_altitude_(0.0)
    {
        loadParams();
        setupOMPL();
        setupCommunications();
        ROS_INFO("Path Planner node initialized.");
    }

private:
    void loadParams() {
        pnh_.param<std::string>("map_frame", map_frame_id_, "map");
        pnh_.param<std::string>("base_frame", base_frame_id_, "base_link");
        pnh_.param<std::string>("octomap_topic", octomap_topic_, "/octomap_full");
        pnh_.param<std::string>("selected_target_topic", selected_target_topic_, "/selected_target_point");
        pnh_.param<std::string>("planned_path_topic", planned_path_topic_, "/planned_path");

        pnh_.param<double>("drone_radius", drone_radius_, 0.3);
        pnh_.param<double>("planning_time_limit", planning_time_limit_, 1.0);
        pnh_.param<double>("goal_tolerance", goal_tolerance_, 0.1);
        pnh_.param<double>("ompl_range", ompl_range_, 0.5);
        pnh_.param<double>("rrt_goal_bias", rrt_goal_bias_, 0.05);
        pnh_.param<double>("update_rate", update_rate_, 2.0);
        pnh_.param<std::string>("ompl_planner_type", selected_planner_type_, "RRTstar");
        pnh_.param<double>("map_bound_x_min", map_bound_x_min_, -20.0);
        pnh_.param<double>("map_bound_x_max", map_bound_x_max_, 20.0);
        pnh_.param<double>("map_bound_y_min", map_bound_y_min_, -20.0);
        pnh_.param<double>("map_bound_y_max", map_bound_y_max_, 20.0);
        pnh_.param<double>("target_update_threshold", target_update_threshold_, 1);
        pnh_.param<bool>("simplify_path", simplify_path_, true);
        pnh_.param<double>("target_timeout", target_timeout_, 5.0);

        ROS_INFO("Path Planner Parameters:");
        ROS_INFO("  Frames: map='%s', base='%s'", map_frame_id_.c_str(), base_frame_id_.c_str());
        ROS_INFO("  Topics: octomap='%s', selected_target='%s', planned_path='%s'",
                 octomap_topic_.c_str(), selected_target_topic_.c_str(), planned_path_topic_.c_str());
        ROS_INFO("  Drone radius: %.2f m", drone_radius_);
        ROS_INFO("  Planning: time=%.2fs, tolerance=%.2fm, update_rate=%.2f Hz",
                 planning_time_limit_, goal_tolerance_, update_rate_);
        ROS_INFO("  OMPL: planner=%s, range=%.2f, goal_bias=%.2f, simplify=%d",
                 selected_planner_type_.c_str(), ompl_range_, rrt_goal_bias_, simplify_path_);
        ROS_INFO("  Map Bounds: X[%.1f, %.1f], Y[%.1f, %.1f]",
                 map_bound_x_min_, map_bound_x_max_, map_bound_y_min_, map_bound_y_max_);
        ROS_INFO("  Target timeout: %.2f s", target_timeout_);
        ROS_INFO("  Target update threshold: %.2f m", target_update_threshold_);
    }

    void setupCommunications() {
        octomap_sub_ = nh_.subscribe(octomap_topic_, 1, &PathPlanner::octomapCallback, this);
        target_sub_ = nh_.subscribe(selected_target_topic_, 1, &PathPlanner::targetCallback, this);
        planned_path_pub_ = nh_.advertise<nav_msgs::Path>(planned_path_topic_, 1);
        visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_planning_visualization", 5);
        planning_timer_ = nh_.createTimer(ros::Duration(1.0 / update_rate_), &PathPlanner::planningTimerCallback, this);
        ROS_INFO("Path Planner subscribers, publisher, and timer set up.");
    }

    void setupOMPL() {
         ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

         state_space_ = std::make_shared<ob::SE2StateSpace>();
         ob::RealVectorBounds bounds(2);
         bounds.setLow(0, map_bound_x_min_); bounds.setHigh(0, map_bound_x_max_);
         bounds.setLow(1, map_bound_y_min_); bounds.setHigh(1, map_bound_y_max_);
         state_space_->as<ob::SE2StateSpace>()->setBounds(bounds);
         space_info_ = std::make_shared<ob::SpaceInformation>(state_space_);

         // Инициализируем Validity Checker с nullptr картой и радиусом
         validity_checker_ = std::make_shared<OctoMapValidityChecker>(space_info_, nullptr, drone_radius_, 0.0);
         space_info_->setStateValidityChecker(validity_checker_);
         space_info_->setup();

         // Выбор планировщика
         if (selected_planner_type_ == "RRTstar") {
             auto specific_planner = std::make_shared<og::RRTstar>(space_info_);
             specific_planner->setRange(ompl_range_);
             specific_planner->setGoalBias(rrt_goal_bias_);
             planner_ = specific_planner;
         } else if (selected_planner_type_ == "InformedRRTstar") {
             auto specific_planner = std::make_shared<og::InformedRRTstar>(space_info_);
             specific_planner->setRange(ompl_range_);
             specific_planner->setGoalBias(rrt_goal_bias_);
             planner_ = specific_planner;
         } else if (selected_planner_type_ == "BITstar") {
             auto specific_planner = std::make_shared<og::BITstar>(space_info_);
             planner_ = specific_planner;
         } else {
             ROS_WARN("Invalid ompl_planner_type '%s'. Defaulting to RRT*.", selected_planner_type_.c_str());
             auto specific_planner = std::make_shared<og::RRTstar>(space_info_);
             specific_planner->setRange(ompl_range_);
             specific_planner->setGoalBias(rrt_goal_bias_);
             planner_ = specific_planner;
             selected_planner_type_ = "RRTstar";
         }

        if (!planner_) {
             ROS_FATAL("Failed to create OMPL planner!");
             ros::shutdown();
             return;
         }
         ROS_INFO("OMPL setup complete for planner type %s.", selected_planner_type_.c_str());
    }


    // --- Callbacks ---
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
         std::shared_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::fullMsgToMap(*msg));
         if (!abstract_tree) {
             ROS_ERROR("Failed to deserialize OctoMap message!");
             return;
         }
         std::shared_ptr<octomap::OcTree> new_tree = std::dynamic_pointer_cast<octomap::OcTree>(abstract_tree);
         if (!new_tree) {
             ROS_ERROR("Deserialized map is not an OcTree!");
             return;
         }

        {
             std::lock_guard<std::mutex> lock(octree_mutex_);
             octree_ = new_tree;
             validity_checker_->updateOctoMap(octree_); // Обновляем карту в checker'е
             have_map_ = true;
        }
        ROS_DEBUG("PathPlanner: Updated OctoMap.");
    }

    // Callback для цели с проверкой близости
    void targetCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        if (msg->header.frame_id != map_frame_id_) {
            ROS_WARN("Received target point in frame '%s', but expected '%s'. Ignoring.",
                    msg->header.frame_id.c_str(), map_frame_id_.c_str());
            return;
        }

        geometry_msgs::Point new_target_point = msg->point;
        ros::Time new_target_stamp = msg->header.stamp;

        std::lock_guard<std::mutex> lock(target_mutex_);

        // --- Проверка близости к текущей активной цели ---
        if (has_active_target_) {
            double current_target_x = latest_target_point_.point.x;
            double current_target_y = latest_target_point_.point.y;

            double dx = new_target_point.x - current_target_x;
            double dy = new_target_point.y - current_target_y;
            double distance_sq = dx * dx + dy * dy;
            double threshold_sq = target_update_threshold_ * target_update_threshold_;

            if (distance_sq <= threshold_sq) {
                ROS_DEBUG("PathPlanner: New target (%.2f, %.2f) is too close to the current active target (%.2f, %.2f) (dist=%.2fm <= thresh=%.2fm). Ignoring update.",
                        new_target_point.x, new_target_point.y,
                        current_target_x, current_target_y,
                        std::sqrt(distance_sq), target_update_threshold_);
                latest_target_point_.header.stamp = new_target_stamp; // Обновляем время последней цели, чтобы таймаут сдвинулся
                return;
            }
            ROS_INFO("PathPlanner: New target is sufficiently far from the previous one. Updating.");
        }
        // --- Конец проверки близости ---

        // Принимаем новую цель
        latest_target_point_ = *msg;
        has_active_target_ = true;
        ROS_INFO("PathPlanner: Accepted new target (%.2f, %.2f) at time %.2f",
                new_target_point.x, new_target_point.y, new_target_stamp.toSec());

    }

    // --- Основной цикл планирования ---
    void planningTimerCallback(const ros::TimerEvent& event)
    {
        // 0. Проверка наличия карты
        {
            std::lock_guard<std::mutex> lock(octree_mutex_);
            if (!octree_ || !have_map_) {
                ROS_DEBUG_THROTTLE(5.0, "PathPlanner: Waiting for OctoMap...");
                 clearVisualization();
                return;
            }
        }

        // 1. Проверка наличия и свежести цели
        geometry_msgs::PointStamped current_target;
        {
            std::lock_guard<std::mutex> lock(target_mutex_);
            if (!has_active_target_) {
                ROS_DEBUG_THROTTLE(5.0, "PathPlanner: Waiting for target point...");
                 clearVisualization();
                return;
            }
            // Проверка таймаута цели
            if ((ros::Time::now() - latest_target_point_.header.stamp).toSec() > target_timeout_) {
                 ROS_WARN("PathPlanner: Target timed out (older than %.2f s). Stopping planning.", target_timeout_);
                 has_active_target_ = false;
                 clearVisualization();

                 nav_msgs::Path empty_path;
                 empty_path.header.stamp = ros::Time::now();
                 empty_path.header.frame_id = map_frame_id_;
                 planned_path_pub_.publish(empty_path);
                 return;
            }
            current_target = latest_target_point_; // Копируем активную цель
        }

        // 2. Получить текущую позу дрона
        geometry_msgs::PoseStamped current_pose_stamped;
        if (!getCurrentDronePose(current_pose_stamped)) {
            ROS_WARN_THROTTLE(2.0, "PathPlanner: Cannot get current drone pose. Skipping planning cycle.");
            return;
        }
         // Обновляем высоту планирования в ValidityChecker
        current_planning_altitude_ = current_pose_stamped.pose.position.z;
        validity_checker_->setPlanningAltitude(current_planning_altitude_);

        // 3. Планирование пути
        ROS_DEBUG("PathPlanner: Planning path to target (%.2f, %.2f) received at %.2f",
                 current_target.point.x, current_target.point.y, current_target.header.stamp.toSec());

        nav_msgs::Path planned_path;
        bool success = planPath(current_pose_stamped.pose, current_target.point, planned_path, simplify_path_);

        if (success && !planned_path.poses.empty()) {
            planned_path.header.stamp = ros::Time::now();
            planned_path.header.frame_id = map_frame_id_;
            planned_path_pub_.publish(planned_path);
            ROS_INFO("PathPlanner: Planned path with %zu poses to target (%.2f, %.2f).",
                     planned_path.poses.size(), current_target.point.x, current_target.point.y);
            publishVisualization(current_target.point, planned_path); // Показать цель и путь
        } else {
            ROS_WARN("PathPlanner: Failed to plan path to target (%.2f, %.2f).",
                      current_target.point.x, current_target.point.y);
             nav_msgs::Path empty_path;
             empty_path.header.stamp = ros::Time::now();
             empty_path.header.frame_id = map_frame_id_;
             planned_path_pub_.publish(empty_path);
             publishVisualization(current_target.point, nav_msgs::Path()); // Показать цель, но без пути
        }
    }


    // --- Вспомогательные функции ---
    bool getCurrentDronePose(geometry_msgs::PoseStamped& pose_stamped) const
    {
        geometry_msgs::TransformStamped transform_stamped;
        try {
            // Использование ros::Time(0) для получения последней доступной трансформации
            transform_stamped = tf_buffer_.lookupTransform(map_frame_id_, base_frame_id_, ros::Time(0), ros::Duration(0.1));
            pose_stamped.header.stamp = transform_stamped.header.stamp; // Время полученной трансформации
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

    bool planPath(const geometry_msgs::Pose& start_pose,
                  const geometry_msgs::Point& goal_point,
                  nav_msgs::Path& result_path,
                  bool simplify)
    {
         ROS_DEBUG("Starting OMPL planning from (%.2f, %.2f) to (%.2f, %.2f) at Z=%.2f",
                   start_pose.position.x, start_pose.position.y,
                   goal_point.x, goal_point.y, current_planning_altitude_);

         auto pdef = std::make_shared<ob::ProblemDefinition>(space_info_);
         pdef->setOptimizationObjective(std::make_shared<ob::PathLengthOptimizationObjective>(space_info_));

         ob::ScopedState<ob::SE2StateSpace> start_state(space_info_);
         start_state->setXY(start_pose.position.x, start_pose.position.y);
         tf2::Quaternion q_start;
         tf2::fromMsg(start_pose.orientation, q_start);
         double roll_start, pitch_start, yaw_start;
         tf2::Matrix3x3(q_start).getRPY(roll_start, pitch_start, yaw_start);
         start_state->setYaw(yaw_start);
         pdef->addStartState(start_state);

         ob::ScopedState<ob::SE2StateSpace> goal_state(space_info_);
         goal_state->setXY(goal_point.x, goal_point.y);
         // Явно задаем ориентацию цели (например, 0), т.к. SE2 требует yaw
         // Если ориентация цели важна, её нужно будет передавать и устанавливать здесь
         goal_state->setYaw(0.0);
         pdef->setGoalState(goal_state, goal_tolerance_);

         planner_->setProblemDefinition(pdef);
         planner_->clear(); // Очистка предыдущих данных планировщика
         ob::PlannerStatus solved = planner_->solve(planning_time_limit_);

         result_path.poses.clear();

         if (solved == ob::PlannerStatus::EXACT_SOLUTION || solved == ob::PlannerStatus::APPROXIMATE_SOLUTION)
         {
             ROS_DEBUG("OMPL found solution (Status: %s)", solved.asString().c_str());
             ob::PathPtr path = pdef->getSolutionPath();
             if (!path) { ROS_ERROR("OMPL success but null path!"); return false; }
             auto& geo_path = static_cast<og::PathGeometric&>(*path);

             if (simplify && geo_path.getStateCount() > 2)
             {
                 og::PathSimplifier simplifier(space_info_);
                 ROS_DEBUG("Simplifying path with %zu states...", geo_path.getStateCount());
                 try {
                     // Ограничиваем время на упрощение, чтобы не блокировать основной цикл
                     double simplification_time = std::min(planning_time_limit_ * 0.2, 0.2); // Например, 20% времени планирования или макс 0.2с
                     if (!simplifier.simplify(geo_path, simplification_time)) {
                          ROS_WARN("Path simplification timed out or failed.");
                     }
                     ROS_DEBUG("Path simplified to %zu states.", geo_path.getStateCount());
                 } catch (const ompl::Exception& e) {
                     ROS_ERROR("OMPL Exception during path simplification: %s", e.what());
                     // Продолжаем использовать не упрощенный путь, если упрощение вызвало исключение
                 }
             }

             result_path.poses.reserve(geo_path.getStateCount());
             for (size_t i = 0; i < geo_path.getStateCount(); ++i)
             {
                 const auto *state = geo_path.getState(i)->as<ob::SE2StateSpace::StateType>();
                 geometry_msgs::PoseStamped pose_stamped;
                 pose_stamped.header.seq = i;
                 pose_stamped.header.stamp = ros::Time(0); // Время будет установлено при публикации
                 pose_stamped.header.frame_id = map_frame_id_;
                 pose_stamped.pose.position.x = state->getX();
                 pose_stamped.pose.position.y = state->getY();
                 pose_stamped.pose.position.z = current_planning_altitude_; // Используем текущую высоту планирования
                 tf2::Quaternion q_out;
                 q_out.setRPY(0, 0, state->getYaw()); // Устанавливаем только yaw
                 pose_stamped.pose.orientation = tf2::toMsg(q_out);
                 result_path.poses.push_back(pose_stamped);
             }
             return true;
         }
         else {
             ROS_WARN("OMPL failed to find a path (Status: %s)", solved.asString().c_str());
             return false;
         }
    }

    // Визуализация
    void publishVisualization(const geometry_msgs::Point& target, const nav_msgs::Path& path)
    {
        if (visualization_pub_.getNumSubscribers() == 0) return;

        visualization_msgs::MarkerArray marker_array;
        ros::Time now = ros::Time::now();
        int id_counter = 0;

        // 1. Очистка предыдущих маркеров этого неймспейса
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = map_frame_id_;
        clear_marker.header.stamp = now;
        clear_marker.ns = "path_planner";
        clear_marker.id = id_counter++; // ID 0 - используется для DELETEALL
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        // Публикуем DELETEALL немедленно, чтобы избежать мерцания старых маркеров
        visualization_pub_.publish(marker_array);
        marker_array.markers.clear(); // Очищаем массив для добавления новых маркеров


        // 2. Маркер цели планирования (синий)
        visualization_msgs::Marker target_marker;
        target_marker.header.frame_id = map_frame_id_;
        target_marker.header.stamp = now;
        target_marker.ns = "path_planner";
        target_marker.id = id_counter++; // ID 1
        target_marker.type = visualization_msgs::Marker::SPHERE;
        target_marker.action = visualization_msgs::Marker::ADD;
        target_marker.pose.position = target;
        target_marker.pose.position.z = current_planning_altitude_; // Отображаем на высоте планирования
        target_marker.pose.orientation.w = 1.0;
        target_marker.scale.x = 0.5; // Немного уменьшим размер маркера цели
        target_marker.scale.y = 0.5;
        target_marker.scale.z = 0.5;
        target_marker.color = createColor(0.0, 0.0, 1.0, 0.8); // Синий
        target_marker.lifetime = ros::Duration(); // 0 = Постоянный (до следующего DELETEALL)
        marker_array.markers.push_back(target_marker);

        // 3. Маркер пути (зеленый)
        if (!path.poses.empty()) {
            visualization_msgs::Marker path_marker;
            path_marker.header.frame_id = map_frame_id_;
            path_marker.header.stamp = now;
            path_marker.ns = "path_planner";
            path_marker.id = id_counter++; // ID 2
            path_marker.type = visualization_msgs::Marker::LINE_STRIP;
            path_marker.action = visualization_msgs::Marker::ADD;
            path_marker.pose.orientation.w = 1.0; // Ориентация для линии не важна
            path_marker.scale.x = 0.1; // Толщина линии
            path_marker.color = createColor(0.0, 1.0, 0.0, 0.8); // Зеленый
            path_marker.lifetime = ros::Duration(); // 0 = Постоянный (до следующего DELETEALL)

            path_marker.points.reserve(path.poses.size());
            for (const auto& pose_stamped : path.poses) {
                path_marker.points.push_back(pose_stamped.pose.position);
            }
             marker_array.markers.push_back(path_marker);
        }

        // Публикуем массив с новыми маркерами (цель и путь, если есть)
        if (!marker_array.markers.empty()) {
            visualization_pub_.publish(marker_array);
        }
    }

     void clearVisualization() {
         if (visualization_pub_.getNumSubscribers() == 0) return;
         visualization_msgs::MarkerArray marker_array;
         visualization_msgs::Marker clear_marker;
         clear_marker.header.frame_id = map_frame_id_;
         clear_marker.header.stamp = ros::Time::now();
         clear_marker.ns = "path_planner";
         clear_marker.id = 0; // ID для DELETEALL
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
    ros::Subscriber octomap_sub_;
    ros::Subscriber target_sub_;
    ros::Publisher planned_path_pub_;
    ros::Publisher visualization_pub_;
    ros::Timer planning_timer_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Состояние карты
    std::mutex octree_mutex_;
    std::shared_ptr<octomap::OcTree> octree_; // Используем shared_ptr для управления памятью
    bool have_map_;

    // Состояние цели
    std::mutex target_mutex_;
    geometry_msgs::PointStamped latest_target_point_;
    bool has_active_target_;

    double current_planning_altitude_; // Текущая высота, на которой происходит планирование

    // OMPL компоненты
    ob::StateSpacePtr state_space_;
    ob::SpaceInformationPtr space_info_;
    std::shared_ptr<OctoMapValidityChecker> validity_checker_; // shared_ptr
    ob::PlannerPtr planner_;

    // Параметры
    std::string map_frame_id_;
    std::string base_frame_id_;
    std::string octomap_topic_;
    std::string selected_target_topic_;
    std::string planned_path_topic_;

    double drone_radius_;
    double target_update_threshold_;
    double planning_time_limit_;
    double goal_tolerance_;
    double ompl_range_;
    double rrt_goal_bias_;
    double update_rate_;
    double map_bound_x_min_, map_bound_x_max_, map_bound_y_min_, map_bound_y_max_;
    double target_timeout_;
    std::string selected_planner_type_;
    bool simplify_path_;
};

// --- main ---
int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"); // Используем приватный NodeHandle для параметров

    try {
        PathPlanner planner(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Unhandled exception in PathPlanner: %s", e.what());
        return 1;
    } catch (...) {
        ROS_FATAL("Unknown unhandled exception in PathPlanner");
        return 1;
    }
    return 0;
}