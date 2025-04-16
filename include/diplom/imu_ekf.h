#ifndef IMU_EKF_H
#define IMU_EKF_H

#include <Eigen/Dense>
#include <ros/ros.h> 
#include <cmath>
#include <boost/array.hpp>

class ImuEKF {
public:
    using StateVector = Eigen::Matrix<double, 7, 1>;       // [qw, qx, qy, qz, bgx, bgy, bgz]
    using StateCovariance = Eigen::Matrix<double, 7, 7>;
    using ProcessNoiseCovariance = Eigen::Matrix<double, 7, 7>;
    using Vector3d = Eigen::Vector3d;
    using Quaterniond = Eigen::Quaterniond; // Eigen использует [w, x, y, z] конструктор, но хранит [x, y, z, w]

    ImuEKF(double dt_nominal,
           double acc_noise_dens, double acc_rw,
           double gyro_noise_dens, double gyro_rw);

    // Выполняет шаг предсказания
    void predict(const Vector3d& gyro_meas, const Vector3d& acc_meas, const ros::Time& current_time);

    // Получение результатов
    Quaterniond getQuaternion() const; // Возвращает Eigen Quaternion (w, x, y, z)
    Vector3d getGyroBias() const;
    // Возвращает ковариацию ориентации 3x3 (аппроксимация для угловой ошибки)
    Eigen::Matrix3d getOrientationCovariance() const;
    // Возвращает ковариацию угловой скорости 3x3
    Eigen::Matrix3d getAngularVelocityCovariance() const;

    // --- Функции для ROS сообщения ---
    // Получение кватерниона в формате ROS (x, y, z, w)
    std::array<double, 4> getRosQuaternion() const;
    // Получение ковариации ориентации для ROS (9 элементов, row-major)
    boost::array<double, 9> getRosOrientationCovariance() const;
    // Получение ковариации угловой скорости для ROS (9 элементов, row-major)
    boost::array<double, 9> getRosAngularVelocityCovariance() const;


private:
    double dt_nominal_;

    // Параметры шума
    double gyro_noise_dens_;
    double gyro_rw_;
    double acc_noise_dens_;
    double acc_rw_;

    // Состояние и ковариация
    StateVector x_; // [qw, qx, qy, qz, bgx, bgy, bgz]
    StateCovariance P_;

    // Матрица шума процесса (рассчитывается один раз)
    ProcessNoiseCovariance Q_;

    ros::Time last_time_;
    bool initialized_;

    // Вспомогательные функции
    static Quaterniond normalize(const Quaterniond& q);
    static Eigen::Matrix<double, 4, 3> dq_dbg_jacobian(const Quaterniond& q, double dt);
};

#endif // IMU_EKF_H