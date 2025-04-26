cmake_minimum_required(VERSION 3.0.2)
project(diplom)

# --- Compiler Settings ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find ROS Dependencies (Catkin) ---
find_package(catkin REQUIRED COMPONENTS
  cv_bridge         # Для конвертации ROS Image <-> OpenCV Mat
  geometry_msgs     # Стандартные сообщения геометрии (Pose, Twist, Point, etc.)
  image_transport   # Для публикации/подписки на сжатые изображения
  nav_msgs          # Сообщения для навигации (Odometry, Path, MapMetaData)
  octomap_msgs      # Сообщения для OctoMap
  octomap_ros       # ROS обертка для OctoMap (конвертеры)
  roscpp            # Клиентская библиотека ROS C++
  rospy             # Клиентская библиотека ROS Python (для скриптов)
  sensor_msgs       # Стандартные сообщения сенсоров (Image, Imu, PointCloud2, etc.)
  std_msgs          # Базовые сообщения ROS (String, Int32, etc.)
  tf2_geometry_msgs # Конвертеры для сообщений геометрии и TF2
  tf2_ros           # Библиотека TF2 для ROS
  visualization_msgs # Сообщения для визуализации в RViz (Marker, MarkerArray)
  ompl
)


# --- Find Non-ROS Dependencies ---
find_package(PythonInterp REQUIRED)   # Для установки Python скриптов
find_package(OpenCV REQUIRED)         # Библиотека компьютерного зрения
find_package(Eigen3 REQUIRED)         # Библиотека линейной алгебры
find_package(octomap REQUIRED)        # Основная библиотека OctoMap

# --- Catkin Package Information ---
# Определяет зависимости для системы сборки и для rosdep
catkin_package(
  INCLUDE_DIRS include             # Публичные заголовки 
  CATKIN_DEPENDS                   # ROS зависимости
    cv_bridge geometry_msgs image_transport nav_msgs octomap_msgs octomap_ros
    roscpp rospy sensor_msgs std_msgs
    tf2_geometry_msgs tf2_ros visualization_msgs 
  DEPENDS                          # Системные зависимости
    OpenCV octomap
)

# --- Include Directories ---
# Добавляем пути к заголовкам для компиляции
include_directories(
  include                          # Локальная папка include/ 
  ${catkin_INCLUDE_DIRS}           # Заголовки из зависимостей catkin
  ${OpenCV_INCLUDE_DIRS}           # Заголовки OpenCV
  ${EIGEN3_INCLUDE_DIR}            # Заголовки Eigen3
  ${OCTOMAP_INCLUDE_DIRS}          # Заголовки OctoMap
  /opt/ros/noetic/include/ompl-1.6 # Заголовки ompl
)

# --- Build Targets (Executables & Scripts) ---

# 1. Camera Publisher (C++)
add_executable(camera_publisher src/camera_publisher.cpp)
target_link_libraries(camera_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
  rpc
  pthread
)
add_dependencies(camera_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 2. imu_publisher_node (C++)
add_executable(imu_publisher_node src/main.cpp
                                 src/msp_ekf_node.cpp
                                 src/imu_ekf.cpp
                                 src/msp_protocol.cpp
                                 src/tcp_transmitter.cpp)
target_link_libraries(imu_publisher_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
add_dependencies(imu_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 3. Frontier detector node (C++)
add_executable(frontier_detector_node src/frontier_detector_node.cpp)
target_link_libraries(frontier_detector_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
add_dependencies(frontier_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 4. Target selector node (C++)
add_executable(target_selector_node src/target_selector_node.cpp)
target_link_libraries(target_selector_node
 ${catkin_LIBRARIES}
 ${OCTOMAP_LIBRARIES}
)
add_dependencies(target_selector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 5. Path planner node (C++)
add_executable(path_planner_node src/path_planner_node.cpp)
target_link_libraries(path_planner_node
 ${catkin_LIBRARIES}
 ${OCTOMAP_LIBRARIES}
 /opt/ros/noetic/lib/x86_64-linux-gnu/libompl.so # ompl
) 
add_dependencies(path_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 6. Teleop (C++)
add_executable(teleop src/teleop.cpp src/msp_protocol.cpp src/tcp_transmitter.cpp) 
target_link_libraries(teleop
  ${catkin_LIBRARIES}
)
add_dependencies(teleop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 7. PLY ror filter (C++)
add_executable(ply_filter_node src/ply_filter_node.cpp
                               src/pointcloud_utils.cpp)
target_link_libraries(ply_filter_node
  ${catkin_LIBRARIES}
)
add_dependencies(ply_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 8. PLY mls (C++)
add_executable(ply_mls_node src/ply_mls_node.cpp
                            src/pointcloud_utils.cpp)
target_link_libraries(ply_mls_node
  ${catkin_LIBRARIES}
)
add_dependencies(ply_mls_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# --- Installation Rules ---

# Установка Launch файлов
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE # Исключаем служебные файлы SVN 
)

# Установка C++ исполняемых файлов
install(TARGETS
  camera_publisher
  imu_publisher_node
  frontier_detector_node
  path_planner_node
  target_selector_node
  ply_filter_node
  ply_mls_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
)