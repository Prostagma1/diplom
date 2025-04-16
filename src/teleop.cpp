#include <ros/ros.h>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>
#include <algorithm>
#include <sstream>
#include <cctype>
#include <stdexcept> // Для std::runtime_error

#include "diplom/msp_protocol.h" 
#include "diplom/tcp_transmitter.h" 

// --- Конфигурация ---
namespace Config {
    const std::string HOST = "127.0.0.1";
    const int PORT = 5762;
    const double SEND_RATE_HZ = 50.0;
    const int NUM_RC_CHANNELS = 7;

    // Индексы каналов
    enum RcChannel : int {
        ROLL = 0,
        PITCH = 1,
        THROTTLE = 2,
        YAW = 3,
        AUX1 = 4, // ARM
        AUX2 = 5,
        AUX3 = 6
    };

    // Значения RC
    const uint16_t RC_VAL_MIN = 1000;
    const uint16_t RC_VAL_MAX = 2000;
    const uint16_t RC_VAL_CENTER = 1500;
    const uint16_t RC_VAL_THROTTLE_MIN = 1000;
    const uint16_t RC_VAL_THROTTLE_MAX = 2000;
    const uint16_t RC_VAL_ARM_DISARMED = 1000;
    const uint16_t RC_VAL_ARM_ARMED = 2000;

    // Параметры управления
    const uint16_t TARGET_PITCH_HIGH = 1700;
    const uint16_t TARGET_PITCH_LOW = 1300;
    const uint16_t TARGET_ROLL_HIGH = 1700;
    const uint16_t TARGET_ROLL_LOW = 1300;
    const uint16_t TARGET_YAW_HIGH = 1700;
    const uint16_t TARGET_YAW_LOW = 1300;

    const int THROTTLE_COARSE_STEP = 10;
    const int THROTTLE_FINE_STEP = 1;
    const int RETURN_STEP = 10; // Шаг возврата к центру

    // Начальные значения
    const uint16_t INITIAL_THROTTLE = 1390;
} // namespace Config

class MspTeleop {
public:
    MspTeleop(ros::NodeHandle& nh) :
        nh_(nh),
        transmitter_(Config::HOST, Config::PORT), // Инициализация передатчика
        msp_protocol_(transmitter_),             // Инициализация протокола
        loop_rate_(Config::SEND_RATE_HZ),
        running_(true),
        current_roll_(Config::RC_VAL_CENTER),
        current_pitch_(Config::RC_VAL_CENTER),
        current_yaw_(Config::RC_VAL_CENTER),
        current_throttle_(Config::INITIAL_THROTTLE)
    {
        if (!transmitter_.connect()) {
            throw std::runtime_error("Failed to connect to MSP endpoint at " + Config::HOST + ":" + std::to_string(Config::PORT));
        }
        ROS_INFO("Connected to MSP endpoint at %s:%d", Config::HOST.c_str(), Config::PORT);

        initKeyboard();
        printControls();
        initializeDroneSequence();
    }

    ~MspTeleop() {
        ROS_INFO("Exiting teleop loop.");
        restoreKeyboard();
        ROS_INFO("Keyboard restored.");
        sendFinalDisarmCommand(); // Отправляем команду Disarm
    }

    // Основной цикл работы узла
    void run() {
        while (running_ && ros::ok()) {
            processInput();
            updateRcState();
            sendRcCommands();

            ros::spinOnce();
            loop_rate_.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    TCPTransmitter transmitter_;
    MspProtocol msp_protocol_;
    ros::Rate loop_rate_;
    bool running_;

    // Состояние RC
    uint16_t current_roll_;
    uint16_t current_pitch_;
    uint16_t current_yaw_;
    uint16_t current_throttle_;

    // Настройки терминала
    struct termios old_tio_, new_tio_;
    char last_key_pressed_ = 0; // Храним последнюю нажатую клавишу

    // Инициализация клавиатуры
    void initKeyboard() {
        tcgetattr(STDIN_FILENO, &old_tio_);
        new_tio_ = old_tio_;
        new_tio_.c_lflag &= (~ICANON & ~ECHO);
        new_tio_.c_cc[VMIN] = 0;
        new_tio_.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
        ROS_INFO("Keyboard control initialized.");
    }

    // Восстановление клавиатуры
    void restoreKeyboard() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

    // Чтение символа с клавиатуры
    char readChar() {
        char buf = 0;
        ssize_t n = read(STDIN_FILENO, &buf, 1);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0; // Нет данных
            else { perror("read keyboard error"); return -1; }     // Ошибка
        }
        return buf; // Символ или 0
    }

     // Вывод информации об управлении
    void printControls() {
        ROS_INFO("--- Controls (Smooth, Always Armed) ---");
        ROS_INFO(" W/S: Pitch Target High/Low     A/D: Roll Target Left/Right");
        ROS_INFO(" Q/E: Yaw Target Left/Right");
        ROS_INFO(" M/N: Throttle Up/Down (Coarse) Z/X: Throttle Up/Down (Fine)");
        ROS_INFO(" Ctrl+C: Exit");
        ROS_WARN(" ARM state is always ON after initial sequence!");
        ROS_INFO(" Release movement keys to return Roll/Pitch/Yaw to center.");
        ROS_INFO("------------------------------------");
    }

    // Последовательность инициализации дрона
    void initializeDroneSequence() {
        ROS_INFO("Sending initial neutral command (throttle low, disarmed)...");
        std::vector<uint16_t> init_cmd1 = {Config::RC_VAL_CENTER, Config::RC_VAL_CENTER, Config::RC_VAL_THROTTLE_MIN, Config::RC_VAL_CENTER,
                                           Config::RC_VAL_ARM_DISARMED, 1000, 1000};
        msp_protocol_.sendRawRC(init_cmd1);
        ros::Duration(0.5).sleep();

        ROS_INFO("Sending arming command (throttle low, armed)...");
        std::vector<uint16_t> init_cmd2 = {Config::RC_VAL_CENTER, Config::RC_VAL_CENTER, Config::RC_VAL_THROTTLE_MIN, Config::RC_VAL_CENTER,
                                           Config::RC_VAL_ARM_ARMED, 1000, 1000};
        msp_protocol_.sendRawRC(init_cmd2);
        ros::Duration(0.5).sleep();
        ROS_INFO("Drone initialization sequence complete. Control loop starting.");
    }

    // Обработка ввода с клавиатуры
    void processInput() {
        last_key_pressed_ = readChar();
        if (last_key_pressed_ == -1) {
            ROS_ERROR("Error reading keyboard input. Exiting.");
            running_ = false;
        } else if (last_key_pressed_ == 3) { // Ctrl+C
            ROS_INFO("Ctrl+C detected, exiting...");
            running_ = false;
        }
    }

    // Обновление состояния RC на основе последнего ввода
    void updateRcState() {
        bool control_key_pressed = false;
        char key = last_key_pressed_;

        if (key != 0) {
            char lower_key = std::tolower(key);

            // Обработка клавиш управления движением (прямое присваивание цели)
            if (lower_key == 'w')      { current_pitch_ = Config::TARGET_PITCH_HIGH; control_key_pressed = true; }
            else if (lower_key == 's') { current_pitch_ = Config::TARGET_PITCH_LOW;  control_key_pressed = true; }
            else if (lower_key == 'd') { current_roll_  = Config::TARGET_ROLL_HIGH;  control_key_pressed = true; }
            else if (lower_key == 'a') { current_roll_  = Config::TARGET_ROLL_LOW;   control_key_pressed = true; }
            else if (lower_key == 'e') { current_yaw_   = Config::TARGET_YAW_HIGH;   control_key_pressed = true; }
            else if (lower_key == 'q') { current_yaw_   = Config::TARGET_YAW_LOW;    control_key_pressed = true; }

            // Обработка клавиш газа (инкрементально)
            else if (lower_key == 'm') { current_throttle_ = std::min(static_cast<uint16_t>(current_throttle_ + Config::THROTTLE_COARSE_STEP), Config::RC_VAL_THROTTLE_MAX); }
            else if (lower_key == 'n') { current_throttle_ = std::max(static_cast<uint16_t>(current_throttle_ - Config::THROTTLE_COARSE_STEP), Config::RC_VAL_THROTTLE_MIN); }
            else if (lower_key == 'x') { current_throttle_ = std::min(static_cast<uint16_t>(current_throttle_ + Config::THROTTLE_FINE_STEP),   Config::RC_VAL_THROTTLE_MAX); }
            else if (lower_key == 'z') { current_throttle_ = std::max(static_cast<uint16_t>(current_throttle_ - Config::THROTTLE_FINE_STEP),   Config::RC_VAL_THROTTLE_MIN); }
        }

        // Логика возврата к центру, если не нажата клавиша управления движением
        if (!control_key_pressed && key == 0) { // Возвращаем только если ничего не нажато
             returnAxisToCenter(current_roll_);
             returnAxisToCenter(current_pitch_);
             returnAxisToCenter(current_yaw_);
        }
    }

    // Вспомогательная функция для возврата оси к центру
    void returnAxisToCenter(uint16_t& axis_value) {
        if (axis_value > Config::RC_VAL_CENTER) {
            axis_value = std::max(Config::RC_VAL_CENTER, static_cast<uint16_t>(axis_value - Config::RETURN_STEP));
        } else if (axis_value < Config::RC_VAL_CENTER) {
            axis_value = std::min(Config::RC_VAL_CENTER, static_cast<uint16_t>(axis_value + Config::RETURN_STEP));
        }
    }

    // Отправка команд RC
    void sendRcCommands() {
        std::vector<uint16_t> rc_channels(Config::NUM_RC_CHANNELS);
        rc_channels[Config::RcChannel::ROLL]     = current_roll_;
        rc_channels[Config::RcChannel::PITCH]    = current_pitch_;
        rc_channels[Config::RcChannel::THROTTLE] = current_throttle_;
        rc_channels[Config::RcChannel::YAW]      = current_yaw_;
        rc_channels[Config::RcChannel::AUX1]     = Config::RC_VAL_ARM_ARMED; // Всегда ARM
        rc_channels[Config::RcChannel::AUX2]     = 1000; // Default
        rc_channels[Config::RcChannel::AUX3]     = 1000; // Default

        if (running_ && transmitter_.isConnected()) {
            if (!msp_protocol_.sendRawRC(rc_channels)) {
                ROS_WARN_THROTTLE(2.0, "Failed to send RC command via MSP.");
            }
            std::stringstream ss; for(size_t i=0; i<rc_channels.size(); ++i) ss << rc_channels[i] << (i==rc_channels.size()-1?"":", ");
            ROS_INFO_THROTTLE(0.5, "RC: [%s]", ss.str().c_str());
        } else if (!transmitter_.isConnected()) {
             ROS_WARN_THROTTLE(5.0, "Transmitter disconnected.");
        }
    }

     // Отправка финальной команды Disarm
    void sendFinalDisarmCommand() {
         if (transmitter_.isConnected()) {
            ROS_INFO("Sending final Disarm command...");
            std::vector<uint16_t> final_cmd(Config::NUM_RC_CHANNELS);
            final_cmd[Config::RcChannel::ROLL]     = Config::RC_VAL_CENTER;
            final_cmd[Config::RcChannel::PITCH]    = Config::RC_VAL_CENTER;
            final_cmd[Config::RcChannel::THROTTLE] = Config::RC_VAL_THROTTLE_MIN;
            final_cmd[Config::RcChannel::YAW]      = Config::RC_VAL_CENTER;
            final_cmd[Config::RcChannel::AUX1]     = Config::RC_VAL_ARM_DISARMED; // DISARM
            final_cmd[Config::RcChannel::AUX2]     = 1000;
            final_cmd[Config::RcChannel::AUX3]     = 1000;

            msp_protocol_.sendRawRC(final_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Дать время на отправку
        } else {
            ROS_WARN("Transmitter was not connected. Final command not sent.");
        }
    }

}; // class MspTeleop

int main(int argc, char **argv) {
    ros::init(argc, argv, "msp_teleop_refactored", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    try {
        MspTeleop teleop_node(nh);
        teleop_node.run();
    } catch (const std::runtime_error& e) {
        ROS_FATAL("Initialization failed: %s", e.what());
        return 1;
    } catch (...) {
        ROS_FATAL("An unexpected error occurred during initialization or execution.");
        return 1;
    }

    return 0;
}