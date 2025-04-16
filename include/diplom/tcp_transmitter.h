#ifndef TCP_TRANSMITTER_H
#define TCP_TRANSMITTER_H

#include <string>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h> // For TCP_NODELAY
#include <arpa/inet.h>
#include <unistd.h> // For close()
#include <errno.h>
#include <ros/ros.h> // For logging

class TCPTransmitter {
public:
    TCPTransmitter(const std::string& host, int port);
    ~TCPTransmitter();

    // Запрещаем копирование и присваивание
    TCPTransmitter(const TCPTransmitter&) = delete;
    TCPTransmitter& operator=(const TCPTransmitter&) = delete;

    bool connect();
    void disconnect();
    // Возвращает количество отправленных байт, или -1 при ошибке
    int send(const std::vector<uint8_t>& buffer, bool blocking = true, int timeout_ms = -1);
    // Возвращает полученные данные, или пустой вектор при ошибке/таймауте/закрытии
    std::vector<uint8_t> receive(size_t size, int timeout_ms = 10000); // Таймаут в мс

    bool isConnected() const;

private:
    std::string host_;
    int port_;
    int sockfd_;
    struct sockaddr_in server_addr_;
    bool is_connected_;

    bool setSocketTimeout(int sockfd, int timeout_ms, bool is_send_timeout);
};

#endif // TCP_TRANSMITTER_H