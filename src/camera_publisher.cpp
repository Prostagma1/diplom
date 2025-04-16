#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rpc/client.h>
#include <rpc/rpc_error.h>
#include <vector>
#include <cmath>
#include <ros/rate.h> 

// Глобальные параметры постобработки 
struct PostProcessParams {
    double gamma = 1.0;
    double saturation = 1.0;
    double contrast = 1.0;
};
PostProcessParams g_postProcessParams;

// Глобальные параметры камеры
struct CameraResolution {
    int width = 640;
    int height = 480;
};
CameraResolution g_cameraResolution;

// Глобальные настройки 
int g_fps = 20;
std::string g_imageMode = "resized";

// Функция загрузки всех настроек из ROS Parameter Server
void loadAllSettings(ros::NodeHandle& pnh) { // Принимаем private NodeHandle

    // --- Загрузка из ROS Параметров ---

    // Загрузка режима изображения
    pnh.param<std::string>("image_mode", g_imageMode, "resized"); 
    ROS_INFO("Loaded ROS param 'image_mode': %s", g_imageMode.c_str());

    // Загрузка разрешения камеры
    pnh.param<int>("camera_resolution/width", g_cameraResolution.width, 640);
    pnh.param<int>("camera_resolution/height", g_cameraResolution.height, 480); 
    ROS_INFO("Loaded ROS param 'camera_resolution': %dx%d", g_cameraResolution.width, g_cameraResolution.height);

    // Загрузка FPS
    pnh.param<int>("fps", g_fps, 20); 
    ROS_INFO("Loaded ROS param 'fps': %d", g_fps);

    // Загрузка параметров постобработки
    pnh.param<double>("post_processing/gamma", g_postProcessParams.gamma, 1.0); 
    pnh.param<double>("post_processing/saturation", g_postProcessParams.saturation, 1.0); 
    pnh.param<double>("post_processing/contrast", g_postProcessParams.contrast, 1.0); 
    ROS_INFO("Loaded ROS param 'post_processing': gamma=%.2f, saturation=%.2f, contrast=%.2f",
             g_postProcessParams.gamma, g_postProcessParams.saturation, g_postProcessParams.contrast);
}

// Функция для постобработки изображения с использованием глобальных параметров
cv::Mat postProcess(cv::Mat image) {
    // Гамма-коррекция
    if (std::abs(g_postProcessParams.gamma - 1.0) > 1e-6) { // Применяем только если гамма не 1.0
       if (g_postProcessParams.gamma <= 0) {
           ROS_WARN_ONCE("Gamma correction skipped: gamma value (%.2f) must be positive.", g_postProcessParams.gamma);
       } else {
           cv::Mat lut(1, 256, CV_8U);
           uchar* p = lut.ptr();
           double invGamma = 1.0 / g_postProcessParams.gamma;
           for (int i = 0; i < 256; ++i) {
               p[i] = cv::saturate_cast<uchar>(std::pow(i / 255.0, invGamma) * 255.0);
           }
           cv::LUT(image, lut, image);
       }
    }

    // Контраст
    if (std::abs(g_postProcessParams.contrast - 1.0) > 1e-6) { // Применяем только если контраст не 1.0
        if (g_postProcessParams.contrast < 0) {
             ROS_WARN_ONCE("Contrast adjustment skipped: contrast value (%.2f) cannot be negative.", g_postProcessParams.contrast);
        } else {
             image.convertTo(image, -1, g_postProcessParams.contrast, 0);
        }
    }

    // Насыщенность
    if (std::abs(g_postProcessParams.saturation - 1.0) > 1e-6) { // Применяем только если насыщенность не 1.0
        if (g_postProcessParams.saturation < 0) {
             ROS_WARN_ONCE("Saturation adjustment skipped: saturation value (%.2f) cannot be negative.", g_postProcessParams.saturation);
        } else if (image.channels() == 3) {
            cv::Mat hsv;
            cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
            std::vector<cv::Mat> channels;
            cv::split(hsv, channels);
            channels[1] *= g_postProcessParams.saturation;
            cv::threshold(channels[1], channels[1], 255, 255, cv::THRESH_TRUNC);
            cv::merge(channels, hsv);
            cv::cvtColor(hsv, image, cv::COLOR_HSV2BGR);
        } else if (image.channels() != 3) {
            ROS_WARN_ONCE("Saturation adjustment skipped: Image does not have 3 channels (BGR).");
        }
    }
    return image;
}

// Функция получения изображения с камеры через RPC вызов
cv::Mat getCameraCapture(rpc::client& client, int camera_id = 0, bool depth = false) {
    if (!ros::ok()) return cv::Mat();
    try {
        auto result = client.call("getCameraCapture", camera_id, false, depth);
        std::vector<uint8_t> raw_image = result.as<std::vector<uint8_t>>();
        if (!raw_image.empty()) {
             const size_t expected_size = 360 * 480 * 4; // BGRA
             if (raw_image.size() != expected_size) {
                 ROS_ERROR_THROTTLE(5.0, "Received unexpected data size from RPC: %zu bytes. Expected 480x360xBGRA (%zu bytes). Check RPC server output format.",
                           raw_image.size(), expected_size);
                 return cv::Mat();
             }

            cv::Mat cv_image_bgra(360, 480, CV_8UC4, raw_image.data());
            cv::Mat cv_image_bgr;
            cv::cvtColor(cv_image_bgra, cv_image_bgr, cv::COLOR_BGRA2BGR);

            cv::Mat final_image;
            // Масштабируем до разрешения, заданного параметрами ROS
            if (cv_image_bgr.cols != g_cameraResolution.width || cv_image_bgr.rows != g_cameraResolution.height) {
                 cv::resize(cv_image_bgr, final_image, cv::Size(g_cameraResolution.width, g_cameraResolution.height), 0, 0, cv::INTER_LINEAR);
            } else {
                final_image = cv_image_bgr; // Масштабирование не требуется
            }

            if (g_imageMode == "processed") {
                return postProcess(final_image);
            } else if (g_imageMode == "resized") {
                return final_image;
            } else {
                 ROS_WARN_ONCE("Unknown image_mode '%s'. Returning resized image.", g_imageMode.c_str());
                 return final_image;
            }
        } else {
            ROS_WARN_THROTTLE(5.0, "Received empty image data from getCameraCapture RPC call.");
        }
    } catch (const rpc::rpc_error &e) {
        ROS_ERROR_THROTTLE(5.0, "RPC error in getCameraCapture: %s. Check if RPC server is running and accessible.", e.what());
    } catch (const cv::Exception &e) {
         ROS_ERROR("OpenCV exception in getCameraCapture: %s", e.what());
    }
    catch (const std::exception &e) {
        ROS_ERROR("Standard exception in getCameraCapture: %s", e.what());
    }
    return cv::Mat(); // Возвращаем пустой Mat в случае ошибки
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh; // Обычный NodeHandle
    ros::NodeHandle pnh("~"); // Private NodeHandle для параметров

    // Загружаем все настройки из ROS параметров
    loadAllSettings(pnh);

    // --- Валидация загруженных параметров ---
    if (g_imageMode != "resized" && g_imageMode != "processed") {
        ROS_WARN("Invalid 'image_mode' parameter: '%s'. Valid options: 'resized', 'processed'. Defaulting to 'resized'.", g_imageMode.c_str());
        g_imageMode = "resized";
    }
    if (g_cameraResolution.width <= 0 || g_cameraResolution.height <= 0) {
         ROS_ERROR("Invalid 'camera_resolution' parameters (%dx%d). Width and height must be positive. Exiting.", g_cameraResolution.width, g_cameraResolution.height);
         return 1;
    }
     if (g_fps <= 0) {
         ROS_WARN("Invalid 'fps' parameter: %d. FPS must be positive. Defaulting to 1.", g_fps);
         g_fps = 1;
    }
    if (g_postProcessParams.gamma <= 0) {
         ROS_WARN("Invalid 'post_processing/gamma' parameter: %.2f. Gamma must be positive. Gamma correction will be disabled if used.", g_postProcessParams.gamma);
    }
    if (g_postProcessParams.saturation < 0) {
         ROS_WARN("Invalid 'post_processing/saturation' parameter: %.2f. Saturation cannot be negative. Saturation adjustment will be disabled if used.", g_postProcessParams.saturation);
    }
     if (g_postProcessParams.contrast < 0) {
         ROS_WARN("Invalid 'post_processing/contrast' parameter: %.2f. Contrast cannot be negative. Contrast adjustment will be disabled if used.", g_postProcessParams.contrast);
    }
    // --- Конец валидации ---

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera_0", 1);

    std::string rpc_host = "127.0.0.1";
    int rpc_port = 8080;
    pnh.param<std::string>("rpc_host", rpc_host, "127.0.0.1");
    pnh.param<int>("rpc_port", rpc_port, 8080);
    ROS_INFO("Connecting to RPC server at %s:%d", rpc_host.c_str(), rpc_port);

    rpc::client client(rpc_host, rpc_port);

    ros::Rate loop_rate(g_fps);
    int error_count = 0;

    while (ros::ok()) {
        cv::Mat frame = getCameraCapture(client); 
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "camera_link"; 

        if (!frame.empty()) {
            if (error_count > 0) {
                 ROS_INFO("Image stream recovered after %d consecutive errors.", error_count);
                 error_count = 0; // Сброс счетчика ошибок при успехе
            }
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            pub.publish(msg);
        } else {
            // Ошибка получения кадра
            error_count++;
            ROS_WARN("Failed to get image from camera (Error count: %d). Check RPC server connection and camera status.", error_count);


             ros::Duration(0.5).sleep(); // Пауза перед повторной попыткой
        }

        ros::spinOnce();
        loop_rate.sleep();
        
    }

    ROS_INFO("Camera publisher node shutting down.");
    return 0;
}