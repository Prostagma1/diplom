#include <diplom/msp_ekf_node.h>
#include <cmath>

MspEkfNode::MspEkfNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), is_connected_(false)
{
    ROS_INFO("Initializing MSP EKF Node...");

    // --- Загрузка параметров ---
    nh_private_.param<std::string>("msp_host", msp_host_, "127.0.0.1");
    nh_private_.param<int>("msp_port", msp_port_, 5760);
    nh_private_.param<std::string>("imu_frame_id", imu_frame_id_, "imu_link");
    nh_private_.param<double>("imu_update_rate", imu_update_rate_, 100.0);
    nh_private_.param<double>("low_freq_rate", low_freq_rate_, 50.0); 
    nh_private_.param<double>("timeshift_cam_imu", timeshift_cam_imu_, -0.023224507794321888);

    nh_private_.param<double>("accelerometer_noise_density", acc_noise_density_, 0.31);
    nh_private_.param<double>("accelerometer_random_walk", acc_random_walk_, 0.008);
    nh_private_.param<double>("gyroscope_noise_density", gyro_noise_density_, 0.0001);
    nh_private_.param<double>("gyroscope_random_walk", gyro_random_walk_, 0.000001);
    nh_private_.param<bool>("gyro_units_are_degrees", gyro_units_are_degrees_, false);

    nh_private_.param<double>("raw_acc_to_ms2_scale", raw_acc_to_ms2_scale_, 9.80665 / 512.0);
    nh_private_.param<double>("raw_gyro_to_rads_scale", raw_gyro_to_rads_scale_, M_PI / 180.0); 

    ROS_INFO("Parameters loaded:");
    ROS_INFO("  MSP Host: %s:%d", msp_host_.c_str(), msp_port_);
    ROS_INFO("  IMU Frame ID: %s", imu_frame_id_.c_str());
    ROS_INFO("  IMU Rate: %.2f Hz", imu_update_rate_);
    ROS_INFO("  Timeshift Cam-IMU: %.6f s", timeshift_cam_imu_);
    ROS_INFO("  Acc Noise Density: %.4f", acc_noise_density_);
    ROS_INFO("  Acc Random Walk: %.4f", acc_random_walk_);
    ROS_INFO("  Gyro Noise Density: %.6f (%s)", gyro_noise_density_, gyro_units_are_degrees_ ? "deg/s/sqrt(Hz)" : "rad/s/sqrt(Hz)");
    ROS_INFO("  Gyro Random Walk: %.8f (%s)", gyro_random_walk_, gyro_units_are_degrees_ ? "deg*sqrt(Hz)?" : "rad*sqrt(Hz)?");
    ROS_INFO("  Raw Acc Scale: %.8f", raw_acc_to_ms2_scale_);
    ROS_INFO("  Raw Gyro Scale: %.8f", raw_gyro_to_rads_scale_);


    // --- Конвертация единиц шума гироскопа ---
    if (gyro_units_are_degrees_) {
        ROS_WARN("Gyro noise parameters assumed to be in degrees. Converting to radians.");
        double deg_to_rad = M_PI / 180.0;
        gyro_noise_density_ *= deg_to_rad;
        gyro_random_walk_ *= deg_to_rad;
        ROS_INFO("  Converted Gyro Noise Density: %.6f rad/s/sqrt(Hz)", gyro_noise_density_);
        ROS_INFO("  Converted Gyro Random Walk: %.8f rad*sqrt(Hz)?", gyro_random_walk_);
    }

    // --- Проверка частоты ---
    if (imu_update_rate_ <= 0) {
        ROS_WARN("Invalid imu_update_rate (<= 0). Using default 100 Hz.");
        imu_update_rate_ = 100.0;
    }
    double dt_nominal = 1.0 / imu_update_rate_;

    // --- Создание объектов ---
    transmitter_ptr_ = std::make_unique<TCPTransmitter>(msp_host_, msp_port_);
    control_ptr_ = std::make_unique<MspProtocol>(*transmitter_ptr_);
    ekf_ptr_ = std::make_unique<ImuEKF>(
        dt_nominal,
        acc_noise_density_, acc_random_walk_,
        gyro_noise_density_, gyro_random_walk_
    );

    // --- Попытка соединения ---
    is_connected_ = connectToFlightController();
    if (!is_connected_) {
        ROS_ERROR("Failed to connect to flight controller on startup. Will retry in timer.");
    }

    // --- Создание издателя ---
    pub_imu_ = nh_.advertise<sensor_msgs::Imu>("imu_0", 10);

    // --- Создание таймера ---
    ros::Duration timer_period(dt_nominal);
    main_timer_ = nh_.createTimer(timer_period, &MspEkfNode::timerCallback, this);

    ROS_INFO("MSP EKF Node initialized and timer started.");
}

bool MspEkfNode::connectToFlightController() {
     ROS_INFO("Attempting to connect to flight controller at %s:%d...", msp_host_.c_str(), msp_port_);
     int trials = 5;
     for (int i = 0; i < trials; ++i) {
         if (transmitter_ptr_->connect()) {
             ROS_INFO("Successfully connected to flight controller.");
             return true;
         }
         if (i < trials - 1) {
              ROS_WARN("Connection attempt %d failed. Retrying in 1 second...", i + 1);
              ros::Duration(1.0).sleep();
         }
     }
     ROS_ERROR("Failed to connect to flight controller after %d attempts.", trials);
     return false;
}


void MspEkfNode::timerCallback(const ros::TimerEvent& event) {
    // Проверка соединения и переподключение при необходимости
    if (!transmitter_ptr_->isConnected()) {
        ROS_WARN_THROTTLE(5.0, "MSP connection lost. Attempting to reconnect...");
        is_connected_ = connectToFlightController();
        if (!is_connected_) {
            return; 
        }
    }

    bool imu_ok = control_ptr_->sendRawMsg(MspProtocol::MspCode::MSP_RAW_IMU);

    if (!imu_ok) {
         ROS_WARN_THROTTLE(1.0,"Failed to send MSP_RAW_IMU request.");
         return;
    }

    // Читаем ответы (с таймаутами)
    bool imu_read_ok = control_ptr_->fastReadImu(); // Внутри вызывается receiveRawMsg

    // Используем время сразу после попытки чтения как временную метку данных
    ros::Time data_stamp = ros::Time::now();

    if (!imu_read_ok) {
         ROS_WARN_THROTTLE(1.0,"Failed to read/parse MSP_RAW_IMU response.");
         return;
    }

    // --- Подготовка данных для EKF ---
    const SensorData& sensor_data = control_ptr_->getSensorData();

    // Акселерометр (m/s^2)
    Eigen::Vector3d accel_input(
        static_cast<double>(sensor_data.raw_accel[0]) * raw_acc_to_ms2_scale_,
        static_cast<double>(sensor_data.raw_accel[1]) * raw_acc_to_ms2_scale_,
        static_cast<double>(sensor_data.raw_accel[2]) * raw_acc_to_ms2_scale_
    );

    // Гироскоп (rad/s)
    Eigen::Vector3d gyro_input(
        static_cast<double>(sensor_data.raw_gyro[0]) * raw_gyro_to_rads_scale_,
        static_cast<double>(sensor_data.raw_gyro[1]) * raw_gyro_to_rads_scale_,
        static_cast<double>(sensor_data.raw_gyro[2]) * raw_gyro_to_rads_scale_
    );

    // --- Запуск EKF ---
    ekf_ptr_->predict(gyro_input, accel_input, data_stamp);

    // --- Публикация IMU сообщения ---
    sensor_msgs::Imu imu_msg = fillImuMsg(data_stamp, accel_input, gyro_input);
    pub_imu_.publish(imu_msg);

    last_msp_read_time_ = data_stamp; // Обновляем время последнего успешного чтения
}

sensor_msgs::Imu MspEkfNode::fillImuMsg(const ros::Time& stamp,
                                         const Eigen::Vector3d& accel_input, // m/s^2
                                         const Eigen::Vector3d& gyro_input) // rad/s
{
    sensor_msgs::Imu msg;
    msg.header.stamp = stamp - ros::Duration(timeshift_cam_imu_); // Применяем сдвиг времени
    msg.header.frame_id = imu_frame_id_;

    // --- Ориентация (из EKF) ---
    std::array<double, 4> ros_q = ekf_ptr_->getRosQuaternion(); // x, y, z, w
    msg.orientation.x = ros_q[0];
    msg.orientation.y = ros_q[1];
    msg.orientation.z = ros_q[2];
    msg.orientation.w = ros_q[3];
    msg.orientation_covariance = ekf_ptr_->getRosOrientationCovariance();

    // --- Угловая скорость (из EKF, скорректированная смещением) ---
    Eigen::Vector3d gyro_bias_est = ekf_ptr_->getGyroBias();
    Eigen::Vector3d gyro_corrected = gyro_input - gyro_bias_est;
    msg.angular_velocity.x = gyro_corrected.x();
    msg.angular_velocity.y = gyro_corrected.y();
    msg.angular_velocity.z = gyro_corrected.z();
    msg.angular_velocity_covariance = ekf_ptr_->getRosAngularVelocityCovariance();

    // --- Линейное ускорение (прямое измерение) ---
    msg.linear_acceleration.x = accel_input.x();
    msg.linear_acceleration.y = accel_input.y();
    msg.linear_acceleration.z = accel_input.z();
    // Ковариация линейного ускорения (основана на шуме измерения)
    double accel_meas_noise_var = 0.0;
     // Variance = density^2 / dt (или density^2 * rate)
    if (imu_update_rate_ > 1e-6) {
         accel_meas_noise_var = acc_noise_density_ * acc_noise_density_ * imu_update_rate_;
    } else {
         accel_meas_noise_var = acc_noise_density_ * acc_noise_density_ * 100.0; // Fallback
    }
    msg.linear_acceleration_covariance.assign(0.0); // Заполнить нулями (метод boost::array)
    msg.linear_acceleration_covariance[0] = accel_meas_noise_var; // Var(ax)
    msg.linear_acceleration_covariance[4] = accel_meas_noise_var; // Var(ay)
    msg.linear_acceleration_covariance[8] = accel_meas_noise_var; // Var(az)

    return msg;
}