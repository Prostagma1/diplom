#include <diplom/msp_protocol.h>
#include <cstring> // memcpy


// --- Определения для статических констант-членов ---
constexpr uint8_t MspProtocol::MSP_HEADER_CHAR1;
constexpr uint8_t MspProtocol::MSP_HEADER_CHAR2;
constexpr uint8_t MspProtocol::MSP_DIRECTION_TO_FC;
constexpr uint8_t MspProtocol::MSP_DIRECTION_FROM_FC; 
constexpr size_t MspProtocol::MSP_HEADER_SIZE;       
constexpr size_t MspProtocol::MSP_OVERHEAD;          

MspProtocol::MspProtocol(TCPTransmitter& transmitter)
    : transmitter_(transmitter) {
}

const SensorData& MspProtocol::getSensorData() const {
    return sensor_data_;
}


uint8_t MspProtocol::calculateChecksum(MspCode code, const std::vector<uint8_t>& data) {
    uint8_t crc = 0;
    crc ^= static_cast<uint8_t>(data.size()); // Data Length
    crc ^= static_cast<uint8_t>(code);        // Code
    for (uint8_t byte : data) {
        crc ^= byte;
    }
    return crc;
}

uint8_t MspProtocol::calculateChecksum(uint8_t data_len, MspCode code, const std::vector<uint8_t>& data) {
     uint8_t crc = 0;
     crc ^= data_len;
     crc ^= static_cast<uint8_t>(code);
     for(size_t i = 0; i < data_len; ++i) { 
         if (i < data.size()) {
             crc ^= data[i];
         } else {
             ROS_ERROR("Checksum calculation inconsistency!");
         }
     }
     return crc;
}


// Пакер для uint16_t (Little Endian)
std::vector<uint8_t> MspProtocol::packData(const std::vector<uint16_t>& values) {
    std::vector<uint8_t> buffer;
    buffer.reserve(values.size() * sizeof(uint16_t));
    for (uint16_t val : values) {
        buffer.push_back(static_cast<uint8_t>(val & 0xFF));         // Low byte
        buffer.push_back(static_cast<uint8_t>((val >> 8) & 0xFF));  // High byte
    }
    return buffer;
}

std::vector<uint8_t> MspProtocol::packData(const std::vector<uint8_t>& values) {
    return values; 
}


// --- Public Methods ---

bool MspProtocol::sendRawMsg(MspCode code, const std::vector<uint8_t>& data, bool blocking, int timeout_ms) {
    if (!transmitter_.isConnected()) {
        ROS_ERROR("MSP: Cannot send, transmitter not connected.");
        return false;
    }

    uint8_t data_len = static_cast<uint8_t>(data.size());
    if (data.size() > 255) {
        ROS_ERROR("MSP: Data payload too large (%zu bytes, max 255)", data.size());
        return false;
    }

    std::vector<uint8_t> message;
    message.reserve(MSP_OVERHEAD + data_len);

    // Header
    message.push_back(MSP_HEADER_CHAR1);      // $
    message.push_back(MSP_HEADER_CHAR2);      // M
    message.push_back(MSP_DIRECTION_TO_FC);   // <

    // Size & Code
    message.push_back(data_len);
    message.push_back(static_cast<uint8_t>(code));

    // Data
    message.insert(message.end(), data.begin(), data.end());

    // Checksum
    uint8_t crc = calculateChecksum(data_len, code, data);
    message.push_back(crc);

    // Отправка
    int bytes_sent = transmitter_.send(message, blocking, timeout_ms);

    if (bytes_sent < 0) {
        ROS_ERROR("MSP: Failed to send message for code %d", static_cast<int>(code));
        return false;
    } else if (static_cast<size_t>(bytes_sent) != message.size()) {
        ROS_WARN("MSP: Incomplete send for code %d (sent %d / %zu bytes)", static_cast<int>(code), bytes_sent, message.size());
        return false;
    }

    return true;
}

std::vector<uint8_t> MspProtocol::receiveRawMsg(size_t expected_payload_size, int timeout_ms) {
    if (!transmitter_.isConnected()) {
        ROS_ERROR("MSP: Cannot receive, transmitter not connected.");
        return {};
    }

    size_t total_expected_size = MSP_OVERHEAD + expected_payload_size;
    std::vector<uint8_t> raw_response = transmitter_.receive(total_expected_size, timeout_ms);

    if (raw_response.empty() || raw_response.size() < MSP_OVERHEAD) {
        ROS_DEBUG("MSP: Receive failed or got incomplete header (got %zu bytes, expected %zu)", raw_response.size(), total_expected_size);
        return {}; // Ошибка приема или недостаточный размер
    }

    // Проверка заголовка
    if (raw_response[0] != MSP_HEADER_CHAR1 ||
        raw_response[1] != MSP_HEADER_CHAR2 ||
        raw_response[2] != MSP_DIRECTION_FROM_FC) { // Ожидаем ответ от FC
        ROS_ERROR("MSP: Received invalid header: %c%c%c", raw_response[0], raw_response[1], raw_response[2]);
        return {};
    }

    uint8_t received_len = raw_response[3];
    MspCode received_code = static_cast<MspCode>(raw_response[4]);
    uint8_t received_crc = raw_response.back();

    // Проверка размера полезной нагрузки
    if (received_len != expected_payload_size) {
         ROS_WARN("MSP: Received unexpected payload size (got %d, expected %zu) for code %d",
                  received_len, expected_payload_size, static_cast<int>(received_code));
    }
    if (raw_response.size() != MSP_OVERHEAD + received_len) {
         ROS_ERROR("MSP: Received inconsistent message size (total %zu bytes, header says payload is %d) for code %d",
                   raw_response.size(), received_len, static_cast<int>(received_code));
         return {};
    }

    // Извлечение данных (без заголовка и CRC)
    std::vector<uint8_t> payload_data;
    if (received_len > 0) {
        payload_data.assign(raw_response.begin() + 5, raw_response.end() - 1);
    }

    // Проверка контрольной суммы
    uint8_t calculated_crc = calculateChecksum(received_len, received_code, payload_data);
    if (calculated_crc != received_crc) {
        ROS_ERROR("MSP: Checksum error for code %d (received 0x%02X, calculated 0x%02X)",
                  static_cast<int>(received_code), received_crc, calculated_crc);
        return {};
    }

    return payload_data; 
}


bool MspProtocol::fastReadImu() {
    // Формат ответа MSP_RAW_IMU: <9h (ax,ay,az, gx,gy,gz, mx,my,mz) - 9 * int16_t
    // Ожидаемый размер полезной нагрузки = 9 * 2 = 18 байт
    const size_t expected_payload_size = 18;
    std::vector<uint8_t> payload = receiveRawMsg(expected_payload_size);

    if (payload.empty() || payload.size() < expected_payload_size) {
        ROS_WARN("MSP: Failed to receive valid MSP_RAW_IMU response.");
        return false;
    }

    std::array<int16_t, 9> raw_values;
    if (unpackStdArray(payload, 0, raw_values)) {
        sensor_data_.raw_accel[0] = raw_values[0];
        sensor_data_.raw_accel[1] = raw_values[1];
        sensor_data_.raw_accel[2] = raw_values[2];

        sensor_data_.raw_gyro[0] = raw_values[3];
        sensor_data_.raw_gyro[1] = raw_values[4];
        sensor_data_.raw_gyro[2] = raw_values[5];

        sensor_data_.raw_mag[0] = raw_values[6];
        sensor_data_.raw_mag[1] = raw_values[7];
        sensor_data_.raw_mag[2] = raw_values[8];

    
        return true;
    } else {
        ROS_ERROR("MSP: Failed to unpack IMU data.");
        return false;
    }
}

// --- Методы отправки команд ---

bool MspProtocol::reboot() {
    ROS_INFO("MSP: Reboot requested");
    // MSP_SET_REBOOT не имеет данных
    return sendRawMsg(MspCode::MSP_SET_REBOOT, {});
}

bool MspProtocol::sendRawMotors(const std::vector<uint16_t>& motor_values) {
    if (motor_values.size() != 8) {
        ROS_ERROR("MSP: sendRawMotors expects exactly 8 motor values, got %zu", motor_values.size());
        return false;
    }
    // MSP_SET_MOTOR ожидает 8 * uint16_t
    std::vector<uint8_t> payload = packData(motor_values);
    return sendRawMsg(MspCode::MSP_SET_MOTOR, payload);
}

bool MspProtocol::sendRawRC(const std::vector<uint16_t>& rc_channels) {
    // MSP_SET_RAW_RC ожидает N * uint16_t
    if (rc_channels.empty()) {
        ROS_ERROR("MSP: sendRawRC requires at least one channel value.");
        return false;
    }
    std::vector<uint8_t> payload = packData(rc_channels);
    return sendRawMsg(MspCode::MSP_SET_RAW_RC, payload);
}