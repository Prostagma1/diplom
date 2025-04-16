#include <ros/ros.h>
#include <diplom/msp_ekf_node.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_publisher"); // Имя узла

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~"); // Для приватных параметров

    try {
        MspEkfNode node(nh, nh_private);
        ROS_INFO("MSP EKF Node spinning.");
        ros::spin(); 
    } catch (const std::exception& e) {
        ROS_FATAL("Unhandled exception in main: %s", e.what());
        return 1;
    } catch (...) {
        ROS_FATAL("Unknown unhandled exception in main.");
        return 1;
    }

    ROS_INFO("MSP EKF Node shutting down.");
    return 0;
}