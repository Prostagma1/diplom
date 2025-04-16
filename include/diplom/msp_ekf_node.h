#ifndef MSP_EKF_NODE_H
#define MSP_EKF_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

#include "tcp_transmitter.h"
#include "msp_protocol.h"
#include "imu_ekf.h"
#include <memory> 

class MspEkfNode {
public:
    MspEkfNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    ~MspEkfNode() = default;

    // Основной цикл обработки (вызывается таймером)
    void timerCallback(const ros::TimerEvent& event);

private:
    // Инициализация соединения с FC
    bool connectToFlightController();

    // Заполнение IMU сообщения
    sensor_msgs::Imu fillImuMsg(const ros::Time& stamp,
                                const Eigen::Vector3d& accel_input, // m/s^2
                                const Eigen::Vector3d& gyro_input); // rad/s

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ROS Интерфейсы
    ros::Publisher pub_imu_;
    ros::Timer main_timer_;

    // Параметры
    std::string msp_host_;
    int msp_port_;
    std::string imu_frame_id_;
    double imu_update_rate_;
    double low_freq_rate_;
    double timeshift_cam_imu_;
    double acc_noise_density_;
    double acc_random_walk_;
    double gyro_noise_density_;
    double gyro_random_walk_;
    bool gyro_units_are_degrees_;
    double raw_acc_to_ms2_scale_;
    double raw_gyro_to_rads_scale_;

    // Объекты для работы
    std::unique_ptr<TCPTransmitter> transmitter_ptr_;
    std::unique_ptr<MspProtocol> control_ptr_;
    std::unique_ptr<ImuEKF> ekf_ptr_;

    bool is_connected_;
    ros::Time last_msp_read_time_;
};

#endif // MSP_EKF_NODE_H