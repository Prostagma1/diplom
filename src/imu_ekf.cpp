#include <diplom/imu_ekf.h>

#include <tf2/LinearMath/Quaternion.h> 

ImuEKF::ImuEKF(double dt_nominal,
               double acc_noise_dens, double acc_rw,
               double gyro_noise_dens, double gyro_rw)
    : dt_nominal_(dt_nominal),
      gyro_noise_dens_(gyro_noise_dens), gyro_rw_(gyro_rw),
      acc_noise_dens_(acc_noise_dens), acc_rw_(acc_rw),
      initialized_(false)
{
    // --- Начальное состояние ---
    x_.setZero();
    x_(0) = 1.0; // qw = 1 (начальная ориентация - единичный кватернион)

    // --- Начальная ковариация ---
    P_ = StateCovariance::Identity() * 0.1;
    P_.block<4, 4>(0, 0) *= 0.01;
    // --- Матрица шума процесса Q ---
    double gyro_meas_noise_var = gyro_noise_dens_ * gyro_noise_dens_ * dt_nominal_;
    // Дисперсия случайного блуждания смещений гироскопа
    double gyro_bias_noise_var = gyro_rw_ * gyro_rw_ * dt_nominal_;

    Q_.setZero();
    double quat_noise_factor = 0.25 * dt_nominal_ * dt_nominal_; 
    Q_(1, 1) = Q_(2, 2) = Q_(3, 3) = gyro_meas_noise_var * 0.5; 
    Q_(0, 0) = gyro_meas_noise_var * 0.1; 

    // Вклад случайного блуждания в неопределенность смещений
    Q_(4, 4) = Q_(5, 5) = Q_(6, 6) = gyro_bias_noise_var;

    ROS_INFO("EKF Initialized. Q diagonal elements for gyro bias: %.4e", gyro_bias_noise_var);
    ROS_INFO("EKF Initialized. Q diagonal elements for quat x,y,z: %.4e", Q_(1, 1));
}

ImuEKF::Quaterniond ImuEKF::normalize(const Quaterniond& q) {
    if (q.coeffs().norm() < 1e-9) {
        ROS_WARN_THROTTLE(5.0, "Normalizing near-zero quaternion!");
        return Quaterniond::Identity();
    }
    return q.normalized();
}

// Вспомогательная функция для Якобиана d(q_pred) / d(b_g)
Eigen::Matrix<double, 4, 3> ImuEKF::dq_dbg_jacobian(const Quaterniond& q, double dt) {

    Eigen::Matrix<double, 4, 3> jac;
    double half_dt = 0.5 * dt;

    // Коэффициенты кватерниона q (w, x, y, z)
    double qw = q.w();
    double qx = q.x();
    double qy = q.y();
    double qz = q.z();

    // Столбец для d/dbgx
    jac(0, 0) =  half_dt * qx;
    jac(1, 0) = -half_dt * qw;
    jac(2, 0) = -half_dt * qz;
    jac(3, 0) =  half_dt * qy;

    // Столбец для d/dbgy
    jac(0, 1) =  half_dt * qy;
    jac(1, 1) =  half_dt * qz;
    jac(2, 1) = -half_dt * qw;
    jac(3, 1) = -half_dt * qx;

    // Столбец для d/dbgz
    jac(0, 2) =  half_dt * qz;
    jac(1, 2) = -half_dt * qy;
    jac(2, 2) =  half_dt * qx;
    jac(3, 2) = -half_dt * qw;

    return jac;
}


void ImuEKF::predict(const Vector3d& gyro_meas, const Vector3d& acc_meas, const ros::Time& current_time) {
    if (!initialized_) {
        last_time_ = current_time;
        initialized_ = true;
        ROS_INFO("EKF first prediction step skipped, initializing time.");
        return;
    }

    double dt_actual = (current_time - last_time_).toSec();
    if (dt_actual <= 1e-6) {
        ROS_WARN_THROTTLE(2.0, "EKF: dt_actual (%.6f) too small or negative. Skipping prediction.", dt_actual);
        return;
    }
    // Ограничим dt сверху, если вдруг был большой перерыв
    dt_actual = std::min(dt_actual, dt_nominal_ * 5.0);


    // --- 1. Предсказание Состояния ---
    // Текущая ориентация и смещения
    Quaterniond q_current(x_(0), x_(1), x_(2), x_(3));
    Vector3d b_g_current = x_.segment<3>(4);

    // Коррекция гироскопа
    Vector3d gyro_corrected = gyro_meas - b_g_current;

    // Интегрирование ориентации (Экспоненциальное отображение)
    Vector3d delta_angle = gyro_corrected * dt_actual;
    double angle_norm = delta_angle.norm();
    Quaterniond delta_q;
    if (angle_norm > 1e-9) {
        Vector3d axis = delta_angle / angle_norm;
        delta_q = Quaterniond(Eigen::AngleAxisd(angle_norm, axis));
    } else {
        delta_q = Quaterniond::Identity();
    }

    // Обновление кватерниона: q_pred = q_current * delta_q
    Quaterniond q_pred = normalize(q_current * delta_q);

    // Предсказание смещения (модель случайного блуждания)
    Vector3d b_g_pred = b_g_current;

    // Обновление вектора состояния x_
    x_(0) = q_pred.w(); x_(1) = q_pred.x(); x_(2) = q_pred.y(); x_(3) = q_pred.z();
    x_.segment<3>(4) = b_g_pred;

    // --- 2. Вычисление Якобиана Модели Процесса (F) ---
    Eigen::Matrix<double, 7, 7> F = Eigen::Matrix<double, 7, 7>::Identity();

    Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
    // Используем приближение Якобиана умножения: I + 0.5 * Omega(gyro_corrected * dt)
    double gx_dt = gyro_corrected.x() * dt_actual;
    double gy_dt = gyro_corrected.y() * dt_actual;
    double gz_dt = gyro_corrected.z() * dt_actual;
    Eigen::Matrix4d Omega_dt;
    Omega_dt << 0.0,    -gx_dt, -gy_dt, -gz_dt,
                gx_dt,  0.0,     gz_dt, -gy_dt,
                gy_dt, -gz_dt,   0.0,    gx_dt,
                gz_dt,  gy_dt,  -gx_dt,  0.0;
    F.block<4, 4>(0, 0) = I4 + 0.5 * Omega_dt; 

    F.block<4, 3>(0, 4) = dq_dbg_jacobian(q_current, dt_actual);

    // --- 3. Предсказание Ковариации ---
    // Масштабируем Q пропорционально dt_actual / dt_nominal
    double dt_ratio = (dt_nominal_ > 1e-6) ? (dt_actual / dt_nominal_) : 1.0;
    ProcessNoiseCovariance Q_scaled = Q_ * dt_ratio;

    // P_pred = F * P * F^T + Q_scaled
    P_ = F * P_ * F.transpose() + Q_scaled;

    P_ = 0.5 * (P_ + P_.transpose());

    last_time_ = current_time;
}

// --- Методы доступа ---

ImuEKF::Quaterniond ImuEKF::getQuaternion() const {
    // Возвращаем кватернион из вектора состояния
    return Quaterniond(x_(0), x_(1), x_(2), x_(3)); // w, x, y, z
}

ImuEKF::Vector3d ImuEKF::getGyroBias() const {
    return x_.segment<3>(4); // bgx, bgy, bgz
}

// Приближенная ковариация ориентации (3x3)
Eigen::Matrix3d ImuEKF::getOrientationCovariance() const {
 
    Eigen::Matrix3d cov_approx = Eigen::Matrix3d::Zero();
    cov_approx(0, 0) = 4.0 * P_(1, 1); // Var(theta_x) ~ 4 * Var(qx)
    cov_approx(1, 1) = 4.0 * P_(2, 2); // Var(theta_y) ~ 4 * Var(qy)
    cov_approx(2, 2) = 4.0 * P_(3, 3); // Var(theta_z) ~ 4 * Var(qz)
    return cov_approx;
}

// Ковариация угловой скорости (3x3)
Eigen::Matrix3d ImuEKF::getAngularVelocityCovariance() const {
    // Неопределенность = шум измерения + неопределенность смещения
    double gyro_meas_noise_var = gyro_noise_dens_ * gyro_noise_dens_ / dt_nominal_; 
    if (dt_nominal_ > 1e-6) {
         gyro_meas_noise_var = gyro_noise_dens_ * gyro_noise_dens_ / dt_nominal_;
    } else {
         gyro_meas_noise_var = gyro_noise_dens_ * gyro_noise_dens_ * 100.0; 
         ROS_WARN_ONCE("dt_nominal is zero, using fallback rate 100Hz for gyro noise variance.");
    }


    Eigen::Matrix3d measurement_cov = Eigen::Matrix3d::Identity() * gyro_meas_noise_var;
    Eigen::Matrix3d bias_cov = P_.block<3, 3>(4, 4); // Ковариация смещений из P

    return measurement_cov + bias_cov;
}

// --- Функции для ROS ---

std::array<double, 4> ImuEKF::getRosQuaternion() const {
    Quaterniond q = getQuaternion(); // w, x, y, z
    return {q.x(), q.y(), q.z(), q.w()}; // ROS: x, y, z, w
}

boost::array<double, 9> ImuEKF::getRosOrientationCovariance() const {
    Eigen::Matrix3d cov = getOrientationCovariance();
    boost::array<double, 9> ros_cov; 
    // Row-major order
    ros_cov[0] = cov(0, 0); ros_cov[1] = cov(0, 1); ros_cov[2] = cov(0, 2);
    ros_cov[3] = cov(1, 0); ros_cov[4] = cov(1, 1); ros_cov[5] = cov(1, 2);
    ros_cov[6] = cov(2, 0); ros_cov[7] = cov(2, 1); ros_cov[8] = cov(2, 2);
    return ros_cov;
}

boost::array<double, 9> ImuEKF::getRosAngularVelocityCovariance() const {
    Eigen::Matrix3d cov = getAngularVelocityCovariance();
    boost::array<double, 9> ros_cov; 
    // Row-major order
    ros_cov[0] = cov(0, 0); ros_cov[1] = cov(0, 1); ros_cov[2] = cov(0, 2);
    ros_cov[3] = cov(1, 0); ros_cov[4] = cov(1, 1); ros_cov[5] = cov(1, 2);
    ros_cov[6] = cov(2, 0); ros_cov[7] = cov(2, 1); ros_cov[8] = cov(2, 2);
    return ros_cov;
}