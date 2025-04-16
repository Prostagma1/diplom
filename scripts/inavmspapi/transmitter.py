import socket
import logging
from abc import ABC, abstractmethod

class Transmitter(ABC):
    @abstractmethod
    def __init__(self):
        self.is_connect = False

    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def disconnect(self):
        pass

    @abstractmethod
    def send(self, bufView, blocking=True, timeout=-1):
        pass

    @abstractmethod
    def receive(self, size, timeout=10):
        pass

class TCPTransmitter(Transmitter):
    def __init__(self, address):
        super().__init__()
        self.address = address
        self.tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.tcp_client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        self.tcp_client.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192 * 2)

    def connect(self):
        if not self.is_connect:
            try:
                self.tcp_client.connect(self.address)
                self.is_connect = True
                logging.info("TCP connected to %s", self.address)
            except socket.error as e:
                logging.error("Cannot connect to TCP: %s", e)
        else:
            logging.debug("TCP client already connected")

    def disconnect(self):
        if self.is_connect:
            try:
                self.tcp_client.close()
                self.is_connect = False
                logging.info("TCP connection closed")
            except socket.error as e:
                logging.error("Cannot close TCP: %s", e)
        else:
            logging.debug("TCP client already disconnected")

    def send(self, bufView: bytearray, blocking: bool = True, timeout: int = -1):
        if timeout > 0:
            self.tcp_client.settimeout(timeout)
        else:
            self.tcp_client.settimeout(None)
        try:
            self.tcp_client.sendall(bufView)
            # logging.debug("Sent message: %s", bufView)
            return 1
        except socket.error as e:
            # logging.error("Cannot send data: %s", e)
            return 0

    def receive(self, size: int, timeout: int = 10):
        self.tcp_client.settimeout(timeout)
        data = b""
        try:
            while len(data) < size:
                packet = self.tcp_client.recv(size - len(data))
                if not packet:
                    break  # Соединение закрыто
                data += packet
            # Разделяем заголовок и основное сообщение
            return data[:1], data[1:]
        except socket.error as e:
            logging.error("Receive error: %s", e)
            return b'', b''

