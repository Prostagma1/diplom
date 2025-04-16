#ifndef MSP_PROTOCOL_H
#define MSP_PROTOCOL_H

#include "tcp_transmitter.h"
#include <vector>
#include <cstdint>
#include <chrono>
#include <thread>
#include <map>
#include <string>
#include <ros/ros.h>

// Структура для хранения данных сенсоров
struct SensorData {
    std::array<double, 3> gyroscope = {0.0, 0.0, 0.0}; // rad/s
    std::array<double, 3> accelerometer = {0.0, 0.0, 0.0}; // m/s^2
    std::array<double, 3> magnetometer = {0.0, 0.0, 0.0}; // Raw units
    double altitude = 0.0; // meters
    double sonar = 0.0; // meters
    std::array<double, 3> kinematics = {0.0, 0.0, 0.0}; // roll, pitch, yaw degrees
    std::array<int16_t, 8> debug = {0}; // Raw debug values

    // Добавим сырые значения для удобства
    std::array<int16_t, 3> raw_gyro = {0};
    std::array<int16_t, 3> raw_accel = {0};
    std::array<int16_t, 3> raw_mag = {0};
    int32_t raw_altitude = 0;
    std::array<int16_t, 3> raw_kinematics = {0};
};

class MspProtocol {
public:
    // MSP коды как enum class для безопасности типов
    enum class MspCode : uint8_t {
        MSP_API_VERSION                = 1,
        MSP_FC_VARIANT                 = 2,
        MSP_FC_VERSION                 = 3,
        MSP_BOARD_INFO                 = 4,
        MSP_BUILD_INFO                 = 5,
        
        MSP_NAME                       = 10,
        MSP_SET_NAME                   = 11,
        
        MSP_BATTERY_CONFIG             = 32,
        MSP_SET_BATTERY_CONFIG         = 33,
        MSP_MODE_RANGES                = 34,
        MSP_SET_MODE_RANGE             = 35,
        MSP_FEATURE_CONFIG             = 36,
        MSP_SET_FEATURE_CONFIG         = 37,
        MSP_BOARD_ALIGNMENT_CONFIG     = 38,
        MSP_SET_BOARD_ALIGNMENT_CONFIG = 39,
        MSP_CURRENT_METER_CONFIG       = 40,
        MSP_SET_CURRENT_METER_CONFIG   = 41,
        MSP_MIXER_CONFIG               = 42,
        MSP_SET_MIXER_CONFIG           = 43,
        MSP_RX_CONFIG                  = 44,
        MSP_SET_RX_CONFIG              = 45,
        MSP_LED_COLORS                 = 46,
        MSP_SET_LED_COLORS             = 47,
        MSP_LED_STRIP_CONFIG           = 48,
        MSP_SET_LED_STRIP_CONFIG       = 49,
        MSP_RSSI_CONFIG                = 50,
        MSP_SET_RSSI_CONFIG            = 51,
        MSP_ADJUSTMENT_RANGES          = 52,
        MSP_SET_ADJUSTMENT_RANGE       = 53,
        MSP_CF_SERIAL_CONFIG           = 54,
        MSP_SET_CF_SERIAL_CONFIG       = 55,
        MSP_VOLTAGE_METER_CONFIG       = 56,
        MSP_SET_VOLTAGE_METER_CONFIG   = 57,
        MSP_SONAR                      = 58,
        MSP_PID_CONTROLLER             = 59,
        MSP_SET_PID_CONTROLLER         = 60,
        MSP_ARMING_CONFIG              = 61,
        MSP_SET_ARMING_CONFIG          = 62,
        MSP_RX_MAP                     = 64,
        MSP_SET_RX_MAP                 = 65,
        MSP_SET_REBOOT                 = 68,
        MSP_DATAFLASH_SUMMARY          = 70,
        MSP_DATAFLASH_READ             = 71,
        MSP_DATAFLASH_ERASE            = 72,
        MSP_LOOP_TIME                  = 73,
        MSP_SET_LOOP_TIME              = 74,
        MSP_FAILSAFE_CONFIG            = 75,
        MSP_SET_FAILSAFE_CONFIG        = 76,
        MSP_RXFAIL_CONFIG              = 77,
        MSP_SET_RXFAIL_CONFIG          = 78,
        MSP_SDCARD_SUMMARY             = 79,
        MSP_BLACKBOX_CONFIG            = 80,
        MSP_SET_BLACKBOX_CONFIG        = 81,
        MSP_TRANSPONDER_CONFIG         = 82,
        MSP_SET_TRANSPONDER_CONFIG     = 83,
        MSP_OSD_CONFIG                 = 84,
        MSP_SET_OSD_CONFIG             = 85,
        MSP_OSD_CHAR_READ              = 86,
        MSP_OSD_CHAR_WRITE             = 87,
        MSP_VTX_CONFIG                 = 88,
        MSP_SET_VTX_CONFIG             = 89,
        MSP_ADVANCED_CONFIG            = 90,
        MSP_SET_ADVANCED_CONFIG        = 91,
        MSP_FILTER_CONFIG              = 92,
        MSP_SET_FILTER_CONFIG          = 93,
        MSP_PID_ADVANCED               = 94,
        MSP_SET_PID_ADVANCED           = 95,
        MSP_SENSOR_CONFIG              = 96,
        MSP_SET_SENSOR_CONFIG          = 97,
        MSP_ARMING_DISABLE             = 99,
        MSP_STATUS                     = 101,
        MSP_RAW_IMU                    = 102,
        MSP_SERVO                      = 103,
        MSP_MOTOR                      = 104,
        MSP_RC                         = 105,
        MSP_RAW_GPS                    = 106,
        MSP_COMP_GPS                   = 107,
        MSP_ATTITUDE                   = 108,
        MSP_ALTITUDE                   = 109,
        MSP_ANALOG                     = 110,
        MSP_RC_TUNING                  = 111,
        MSP_PID                        = 112,
        MSP_BOXNAMES                   = 116,
        MSP_PIDNAMES                   = 117,
        MSP_BOXIDS                     = 119,
        MSP_SERVO_CONFIGURATIONS       = 120,
        MSP_MOTOR_3D_CONFIG            = 124,
        MSP_RC_DEADBAND                = 125,
        MSP_SENSOR_ALIGNMENT           = 126,
        MSP_LED_STRIP_MODECOLOR        = 127,
        
        MSP_VOLTAGE_METERS             = 128,
        MSP_CURRENT_METERS             = 129,
        MSP_BATTERY_STATE              = 130,
        MSP_MOTOR_CONFIG               = 131,
        MSP_GPS_CONFIG                 = 132,
        MSP_COMPASS_CONFIG             = 133,
        MSP_GPS_RESCUE                 = 135,
        
        MSP_STATUS_EX                  = 150,
        
        MSP_UID                        = 160,
        MSP_GPS_SV_INFO                = 164,
        
        MSP_GPSSTATISTICS              = 166,
        
        MSP_DISPLAYPORT                = 182,
        
        MSP_COPY_PROFILE               = 183,
        
        MSP_BEEPER_CONFIG              = 184,
        MSP_SET_BEEPER_CONFIG          = 185,
        
        MSP_SET_RAW_RC                 = 200,
        MSP_SET_PID                    = 202,
        MSP_SET_RC_TUNING              = 204,
        MSP_ACC_CALIBRATION            = 205,
        MSP_MAG_CALIBRATION            = 206,
        MSP_RESET_CONF                 = 208,
        MSP_SELECT_SETTING             = 210,
        MSP_SET_SERVO_CONFIGURATION    = 212,
        MSP_SET_MOTOR                  = 214,
        MSP_SET_MOTOR_3D_CONFIG        = 217,
        MSP_SET_RC_DEADBAND            = 218,
        MSP_SET_RESET_CURR_PID         = 219,
        MSP_SET_SENSOR_ALIGNMENT       = 220,
        MSP_SET_LED_STRIP_MODECOLOR    = 221,
        MSP_SET_MOTOR_CONFIG           = 222,
        MSP_SET_GPS_CONFIG             = 223,
        MSP_SET_COMPASS_CONFIG         = 224,
        MSP_SET_GPS_RESCUE             = 225,
        
        MSP_MODE_RANGES_EXTRA          = 238,
        MSP_SET_ACC_TRIM               = 239,
        MSP_ACC_TRIM                   = 240,
        MSP_SERVO_MIX_RULES            = 241,
        MSP_SET_RTC                    = 246,
        
        MSP_EEPROM_WRITE               = 250,
        MSP_DEBUG                      = 254        
    };


    explicit MspProtocol(TCPTransmitter& transmitter); 

    bool fastReadImu();

    // Методы для отправки команд
    bool reboot();
    bool sendRawMotors(const std::vector<uint16_t>& motor_values); // 8 моторов
    bool sendRawRC(const std::vector<uint16_t>& rc_channels);
    
    // Получение данных сенсоров
    const SensorData& getSensorData() const;

    // Отправка сырого MSP сообщения
    bool sendRawMsg(MspCode code, const std::vector<uint8_t>& data = {}, bool blocking = true, int timeout_ms = -1);


private:
    TCPTransmitter& transmitter_; // Ссылка на передатчик
    SensorData sensor_data_;

    
    // Прием сырого MSP сообщения (ожидает конкретный размер)
    std::vector<uint8_t> receiveRawMsg(size_t expected_size, int timeout_ms = 3000);

    // Вспомогательная функция для упаковки данных (Little Endian)
    static std::vector<uint8_t> packData(const std::vector<uint16_t>& values);
    static std::vector<uint8_t> packData(const std::vector<uint8_t>& values); // Для единообразия

    // Вспомогательные функции для распаковки (Little Endian)
    // Возвращает true при успехе
    template<typename T>
    static bool unpackValue(const std::vector<uint8_t>& buffer, size_t offset, T& value);

    // Распаковка массива значений
    template<typename T>
    static bool unpackArray(const std::vector<uint8_t>& buffer, size_t offset, std::vector<T>& values);

    template<typename T, size_t N>
    static bool unpackStdArray(const std::vector<uint8_t>& buffer, size_t offset, std::array<T, N>& values);

    // Расчет контрольной суммы MSP
    static uint8_t calculateChecksum(MspCode code, const std::vector<uint8_t>& data);
    static uint8_t calculateChecksum(uint8_t data_len, MspCode code, const std::vector<uint8_t>& data);

    // Константы заголовка MSP
    static constexpr uint8_t MSP_HEADER_CHAR1 = '$';
    static constexpr uint8_t MSP_HEADER_CHAR2 = 'M';
    static constexpr uint8_t MSP_DIRECTION_TO_FC = '<';
    static constexpr uint8_t MSP_DIRECTION_FROM_FC = '>';
    static constexpr size_t MSP_HEADER_SIZE = 3; // $, M, < or >
    static constexpr size_t MSP_OVERHEAD = 6; // Header(3) + Size(1) + Code(1) + CRC(1)
};


// --- Реализация шаблонных функций в заголовочном файле ---

template<typename T>
bool MspProtocol::unpackValue(const std::vector<uint8_t>& buffer, size_t offset, T& value) {
    if (offset + sizeof(T) > buffer.size()) {
        ROS_ERROR("unpackValue: Buffer too small (offset=%zu, size=%zu, need %zu)", offset, buffer.size(), sizeof(T));
        return false;
    }
    // Прямое копирование байт (работает для little-endian архитектур, как x86/ARM)
    std::memcpy(&value, buffer.data() + offset, sizeof(T));
    // Для big-endian нужна будет конвертация байт
    return true;
}

template<typename T>
bool MspProtocol::unpackArray(const std::vector<uint8_t>& buffer, size_t offset, std::vector<T>& values) {
     size_t bytes_needed = values.size() * sizeof(T);
    if (offset + bytes_needed > buffer.size()) {
        ROS_ERROR("unpackArray: Buffer too small (offset=%zu, size=%zu, need %zu)", offset, buffer.size(), bytes_needed);
        return false;
    }
    // Прямое копирование (для little-endian)
     std::memcpy(values.data(), buffer.data() + offset, bytes_needed);
     return true;
}

template<typename T, size_t N>
bool MspProtocol::unpackStdArray(const std::vector<uint8_t>& buffer, size_t offset, std::array<T, N>& values) {
    size_t bytes_needed = N * sizeof(T);
    if (offset + bytes_needed > buffer.size()) {
        ROS_ERROR("unpackStdArray: Buffer too small (offset=%zu, size=%zu, need %zu)", offset, buffer.size(), bytes_needed);
        return false;
    }
    // Прямое копирование (для little-endian)
    std::memcpy(values.data(), buffer.data() + offset, bytes_needed);
    return true;
}


#endif // MSP_PROTOCOL_H