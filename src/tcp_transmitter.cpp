#include <diplom/tcp_transmitter.h>
#include <cstring> 
#include <poll.h>

TCPTransmitter::TCPTransmitter(const std::string& host, int port)
    : host_(host), port_(port), sockfd_(-1), is_connected_(false) {
    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port_);
    if (inet_pton(AF_INET, host_.c_str(), &server_addr_.sin_addr) <= 0) {
        ROS_ERROR("Invalid address/ Address not supported: %s", host_.c_str());
    }
}

TCPTransmitter::~TCPTransmitter() {
    disconnect();
}

bool TCPTransmitter::setSocketTimeout(int sockfd, int timeout_ms, bool is_send_timeout) {
    if (sockfd < 0) return false;

    struct timeval tv;
    if (timeout_ms < 0) {
        tv.tv_sec = 3600; // 1 час
        tv.tv_usec = 0;
    } else {
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
    }

    int optname = is_send_timeout ? SO_SNDTIMEO : SO_RCVTIMEO;
    if (setsockopt(sockfd, SOL_SOCKET, optname, (const char*)&tv, sizeof(tv)) < 0) {
        ROS_ERROR("Failed to set socket %s timeout: %s", is_send_timeout ? "send" : "receive", strerror(errno));
        return false;
    }
    return true;
}


bool TCPTransmitter::connect() {
    if (is_connected_) {
        ROS_DEBUG("TCP client already connected to %s:%d", host_.c_str(), port_);
        return true;
    }

    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0) {
        ROS_ERROR("Cannot create socket: %s", strerror(errno));
        return false;
    }

    // Установка опций сокета
    int flag = 1;
    if (setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int)) < 0) {
        ROS_WARN("Cannot set TCP_NODELAY: %s", strerror(errno));
    }

    int recvbuf_size = 8192 * 2;
    if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVBUF, &recvbuf_size, sizeof(recvbuf_size)) < 0) {
         ROS_WARN("Cannot set SO_RCVBUF: %s", strerror(errno));
    }

    // Пытаемся соединиться
    if (::connect(sockfd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
        ROS_ERROR("Cannot connect to TCP %s:%d : %s", host_.c_str(), port_, strerror(errno));
        close(sockfd_);
        sockfd_ = -1;
        return false;
    }

    is_connected_ = true;
    ROS_INFO("TCP connected to %s:%d", host_.c_str(), port_);
    return true;
}

void TCPTransmitter::disconnect() {
    if (is_connected_ && sockfd_ >= 0) {
        if (close(sockfd_) == 0) {
             ROS_INFO("TCP connection closed");
        } else {
             ROS_ERROR("Error closing TCP socket: %s", strerror(errno));
        }
        sockfd_ = -1;
        is_connected_ = false;
    } else {
        ROS_DEBUG("TCP client already disconnected");
    }
}

bool TCPTransmitter::isConnected() const {
    return is_connected_;
}

int TCPTransmitter::send(const std::vector<uint8_t>& buffer, bool blocking, int timeout_ms) {
    if (!is_connected_ || sockfd_ < 0) {
        ROS_ERROR("Cannot send data: not connected");
        return -1;
    }

    // Установка таймаута для отправки
    if (!setSocketTimeout(sockfd_, timeout_ms, true)) {
         ROS_WARN("Could not set send timeout, proceeding with default.");
    }

    ssize_t total_sent = 0;
    size_t bytes_left = buffer.size();
    const uint8_t* ptr = buffer.data();

    while (bytes_left > 0) {
        ssize_t bytes_sent = ::send(sockfd_, ptr, bytes_left, MSG_NOSIGNAL);

        if (bytes_sent < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                 ROS_WARN("Send would block or timed out: %s", strerror(errno));
                 return -1; // Ошибка - не удалось отправить все
            } else {
                // Другая ошибка сокета
                ROS_ERROR("Cannot send data: %s", strerror(errno));
                disconnect(); // Соединение разорвано
                return -1;
            }
        } else if (bytes_sent == 0) {
            ROS_ERROR("Cannot send data: connection closed by peer");
            disconnect();
            return -1;
        }

        total_sent += bytes_sent;
        bytes_left -= bytes_sent;
        ptr += bytes_sent;
    }

    return total_sent; // Возвращаем количество отправленных байт
}

std::vector<uint8_t> TCPTransmitter::receive(size_t size, int timeout_ms) {
    if (!is_connected_ || sockfd_ < 0) {
        ROS_ERROR("Cannot receive data: not connected");
        return {}; // Пустой вектор
    }
    if (size == 0) {
        return {}; // Ничего не запрашивали
    }

    // Установка таймаута для приема
    if (!setSocketTimeout(sockfd_, timeout_ms, false)) {
         ROS_WARN("Could not set receive timeout, proceeding with default.");
    }


    std::vector<uint8_t> buffer;
    buffer.reserve(size); // Предварительно выделяем память
    size_t total_received = 0;
    uint8_t temp_buf[4096]; // Временный буфер для чтения

    while (total_received < size) {
        size_t bytes_to_read = std::min(sizeof(temp_buf), size - total_received);
        ssize_t bytes_received = ::recv(sockfd_, temp_buf, bytes_to_read, 0);

        if (bytes_received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Таймаут приема
                ROS_WARN("Receive timed out after %d ms (received %zu/%zu bytes)", timeout_ms, total_received, size);
                return {}; 
            } else {
                // Другая ошибка сокета
                ROS_ERROR("Receive error: %s", strerror(errno));
                disconnect(); 
                return {}; 
            }
        } else if (bytes_received == 0) {
            // Соединение закрыто удаленной стороной
            ROS_ERROR("Receive error: connection closed by peer (received %zu/%zu bytes)", total_received, size);
            disconnect();
            return {}; 
        }

        // Добавляем полученные данные в основной буфер
        buffer.insert(buffer.end(), temp_buf, temp_buf + bytes_received);
        total_received += bytes_received;
    }

    return buffer;
}