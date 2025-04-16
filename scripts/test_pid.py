#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_geometry_msgs # Нужен для do_transform_pose
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist # Twist может не понадобиться, но полезно для логгирования
from nav_msgs.msg import Path
from std_msgs.msg import Float64 # Можно использовать для отладки PID
import math
import time
import numpy as np

# --- Импортируйте ваши классы управления ---
# Предполагается, что они находятся в доступном месте
try:
    from inavmspapi import MultirotorControl
    from inavmspapi.transmitter import TCPTransmitter
except ImportError:
    rospy.logfatal("Не удалось импортировать TCPTransmitter или MultirotorControl. Убедитесь, что файлы доступны.")
    exit(1)

# --- Простой PID контроллер ---
class SimplePID:
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, setpoint=0.0, sample_time=0.01, output_limits=(-100, 100), integral_limits=(-10, 10)):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.setpoint = setpoint
        self.sample_time = sample_time # Примерное время между вызовами update

        self._min_output, self._max_output = output_limits
        self._min_integral, self._max_integral = integral_limits

        self._last_time = time.time()
        self._last_error = 0.0
        self._proportional = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        self._last_output = 0.0

    def update(self, process_variable):
        current_time = time.time()
        dt = current_time - self._last_time

        # --- Пропуск итерации, если dt слишком мал ---
        if dt < self.sample_time and self._last_output is not None:
             return self._last_output # Возвращаем предыдущее значение
        # Если первая итерация, делаем dt равным sample_time
        if self._last_output is None:
             dt = self.sample_time

        error = self.setpoint - process_variable

        # --- Proportional term ---
        self._proportional = self.Kp * error

        # --- Integral term ---
        self._integral += self.Ki * error * dt
        # Ограничение интегральной суммы
        self._integral = max(min(self._integral, self._max_integral), self._min_integral)

        # --- Derivative term ---
        delta_error = error - self._last_error
        # Небольшая защита от деления на ноль и выбросов при больших dt
        if dt > 1e-6:
             self._derivative = self.Kd * delta_error / dt
        else:
             self._derivative = 0.0

        # --- Compute output ---
        output = self._proportional + self._integral + self._derivative
        output = max(min(output, self._max_output), self._min_output)

        # --- Update state ---
        self._last_error = error
        self._last_time = current_time
        self._last_output = output

        #rospy.logdebug(f"PID Update: PV={process_variable:.2f}, Setpoint={self.setpoint:.2f}, Error={error:.2f}, P={self._proportional:.2f}, I={self._integral:.2f}, D={self._derivative:.2f}, Output={output:.2f}, dt={dt:.4f}")

        return output

    def reset(self):
        self._proportional = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        self._last_error = 0.0
        self._last_output = 0.0
        self._last_time = time.time()

    def set_limits(self, output_limits, integral_limits):
        self._min_output, self._max_output = output_limits
        self._min_integral, self._max_integral = integral_limits
        # Сбросить интеграл при смене лимитов
        self._integral = max(min(self._integral, self._max_integral), self._min_integral)

# --- Основной класс узла ---
class PathFollowerNode:
    def __init__(self):
        rospy.init_node('path_follower_node')
        rospy.loginfo("Инициализация Path Follower Node...")

        # --- Параметры ---
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.control_rate = rospy.get_param("~control_rate", 20.0) # Гц
        self.target_altitude = rospy.get_param("~target_altitude", 1.5) # Желаемая высота полета (метры)
        self.waypoint_tolerance = rospy.get_param("~waypoint_tolerance", 0.3) # Метры. Как близко к точке считать ее достигнутой
        # Важно: lookahead > waypoint_tolerance
        self.lookahead_distance = rospy.get_param("~lookahead_distance", 0.8) # Метры. На какое расстояние вперед по пути смотреть

        # Параметры подключения к симулятору/дрону
        tcp_host = rospy.get_param("~tcp_host", "127.0.0.1")
        tcp_port = rospy.get_param("~tcp_port", 5762)

        # Параметры RC команд (НУЖНО НАСТРОИТЬ!)
        self.rc_roll_center = rospy.get_param("~rc_roll_center", 1500)
        self.rc_pitch_center = rospy.get_param("~rc_pitch_center", 1500)
        self.rc_yaw_center = rospy.get_param("~rc_yaw_center", 1500)
        self.rc_throttle_hover = rospy.get_param("~rc_throttle_hover", 1500) # ОЧЕНЬ ВАЖНО НАСТРОИТЬ!
        self.rc_min = rospy.get_param("~rc_min", 1000)
        self.rc_max = rospy.get_param("~rc_max", 2000)
        # Диапазон отклонения от центра/ховера (НАСТРОИТЬ!)
        self.rc_roll_range = rospy.get_param("~rc_roll_range", 300) # +/- от центра
        self.rc_pitch_range = rospy.get_param("~rc_pitch_range", 300) # +/- от центра
        self.rc_yaw_range = rospy.get_param("~rc_yaw_range", 400)    # +/- от центра
        self.rc_throttle_range = rospy.get_param("~rc_throttle_range", 400) # +/- от ховера

        self.arm_value = rospy.get_param("~arm_value", 2000) # Значение для ARM канала
        self.disarm_value = rospy.get_param("~disarm_value", 1000)
        self.mode_value = rospy.get_param("~mode_value", 1500) # Значение для канала MODE (если нужно)

        # Коэффициенты PID (КРИТИЧЕСКИ ВАЖНО НАСТРОИТЬ!)
        # Высота (Z)
        kp_z = rospy.get_param("~pid/z/kp", 150.0) # Примерные значения!
        ki_z = rospy.get_param("~pid/z/ki", 5.0)
        kd_z = rospy.get_param("~pid/z/kd", 20.0)
        # Горизонтальное движение (скорость к цели)
        kp_xy = rospy.get_param("~pid/xy/kp", 1.0) # Этот PID выдает желаемую скорость
        ki_xy = rospy.get_param("~pid/xy/ki", 0.1)
        kd_xy = rospy.get_param("~pid/xy/kd", 0.0)
        self.max_xy_speed = rospy.get_param("~max_xy_speed", 1.0) # Макс. горизонтальная скорость, м/с
        # Рыскание (Yaw)
        kp_yaw = rospy.get_param("~pid/yaw/kp", 150.0)
        ki_yaw = rospy.get_param("~pid/yaw/ki", 10.0)
        kd_yaw = rospy.get_param("~pid/yaw/kd", 5.0)

        rospy.loginfo(f"Target Altitude: {self.target_altitude:.2f} m")
        rospy.loginfo(f"Control Rate: {self.control_rate:.1f} Hz")
        rospy.loginfo(f"RC Hover Throttle: {self.rc_throttle_hover}")
        rospy.loginfo(f"PID Z: Kp={kp_z}, Ki={ki_z}, Kd={kd_z}")
        rospy.loginfo(f"PID XY->Speed: Kp={kp_xy}, Ki={ki_xy}, Kd={kd_xy}")
        rospy.loginfo(f"PID Yaw: Kp={kp_yaw}, Ki={ki_yaw}, Kd={kd_yaw}")


        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- Состояние ---
        self.current_path = None
        self.current_pose_map = None # PoseStamped в map фрейме
        self.target_waypoint_index = 0
        self.armed = False # Начнем с disarmed состояния в логике

        # --- Инициализация управления ---
        try:
            transmitter_address = (tcp_host, tcp_port)
            self.tcp_transmitter = TCPTransmitter(transmitter_address)
            # Пытаемся подключиться сразу
            if not self.connect_transmitter():
                 # Ошибка при подключении handled в connect_transmitter
                 rospy.signal_shutdown("Не удалось подключиться к передатчику.")
                 return

            self.control = MultirotorControl(self.tcp_transmitter)
            rospy.loginfo("MultirotorControl инициализирован.")

        except Exception as e:
            rospy.logfatal(f"Ошибка инициализации управления: {e}")
            rospy.signal_shutdown("Ошибка инициализации управления.")
            return

        # --- Инициализация PID контроллеров ---
        pid_dt = 1.0 / self.control_rate
        # PID Высоты: выход -> добавка к тяге висения
        self.pid_z = SimplePID(kp_z, ki_z, kd_z, setpoint=self.target_altitude, sample_time=pid_dt,
                               output_limits=(-self.rc_throttle_range, self.rc_throttle_range))

        # PID Горизонтального движения: ошибка -> желаемая скорость в направлении ошибки
        # Мы не будем напрямую использовать этот PID для генерации Roll/Pitch,
        # а будем использовать его выход (желаемую скорость) для расчета Roll/Pitch
        # (Более простой подход - сразу PID положения в Roll/Pitch, но этот гибче)
        # Этот PID не используется в текущей реализации ниже, оставлен для возможного расширения.

        # PID Рыскания: выход -> добавка к RC каналу рыскания
        self.pid_yaw = SimplePID(kp_yaw, ki_yaw, kd_yaw, setpoint=0.0, sample_time=pid_dt,
                                 output_limits=(-self.rc_yaw_range, self.rc_yaw_range))

        # --- Паблишеры и подписчики ---
        self.path_sub = rospy.Subscriber("/planned_path", Path, self.path_callback, queue_size=1)
        # Дополнительные паблишеры для отладки
        self.debug_target_pose_pub = rospy.Publisher("~debug/target_pose", PoseStamped, queue_size=1)
        self.debug_lookahead_pub = rospy.Publisher("~debug/lookahead_pose", PoseStamped, queue_size=1)

        # --- Таймер основного цикла управления ---
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_loop_callback)

        # --- Обработчик завершения ---
        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("Path Follower Node инициализирован и готов.")


    def connect_transmitter(self, retries=5, delay=1):
         """Пытается подключиться к TCP передатчику."""
         for i in range(retries):
             try:
                 self.tcp_transmitter.connect()
                 if self.tcp_transmitter.is_connect:
                     rospy.loginfo("Успешное подключение к TCP передатчику.")
                     return True
             except Exception as e:
                 rospy.logwarn(f"Попытка подключения {i+1}/{retries} не удалась: {e}")
                 time.sleep(delay)
         rospy.logerr("Не удалось подключиться к TCP передатчику после нескольких попыток.")
         return False

    def path_callback(self, msg):
        """Обработка нового пути."""
        if not msg.poses:
            rospy.logwarn("Получен пустой путь.")
            self.current_path = None
            self.target_waypoint_index = 0
            # Возможно, стоит отправить команду зависания, если путь исчез
            # self.send_hover_command() # Нужна реализация
            return

        if self.current_path and msg.header.seq == self.current_path.header.seq:
             rospy.logdebug("Получен тот же путь, игнорируем.")
             return

        rospy.loginfo(f"Получен новый путь с {len(msg.poses)} точками.")
        self.current_path = msg
        self.target_waypoint_index = 0 # Начинаем с первой точки нового пути
        # Сбрасываем PID при получении нового пути, чтобы избежать старых интегральных ошибок
        self.pid_z.reset()
        self.pid_yaw.reset()
        # Сбросить setpoint для PID yaw (будет установлен в control_loop)
        self.pid_yaw.setpoint = 0.0


    def get_current_pose(self):
        """Получает текущую позу дрона в map фрейме."""
        try:
            # Используем ros::Time(0), чтобы получить последнюю доступную трансформацию
            trans = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.1))

            pose_stamped = PoseStamped()
            pose_stamped.header = trans.header # Время и фрейм (map_frame)
            pose_stamped.pose.position.x = trans.transform.translation.x
            pose_stamped.pose.position.y = trans.transform.translation.y
            pose_stamped.pose.position.z = trans.transform.translation.z
            pose_stamped.pose.orientation = trans.transform.rotation
            self.current_pose_map = pose_stamped
            return True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, f"Не удалось получить TF трансформацию {self.map_frame} -> {self.base_frame}: {e}")
            self.current_pose_map = None
            return False

    def get_yaw_from_quat(self, quaternion):
        """Извлекает Yaw из кватерниона ROS."""
        # Используем numpy для конвертации
        q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        # Простое преобразование для Yaw (z-axis rotation)
        # Внимание: это упрощенное преобразование, полное RPY требует больше вычислений
        # или использования tf.transformations
        yaw = math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                         1.0 - 2.0 * (q[1]**2 + q[2]**2))
        return yaw

    def normalize_angle(self, angle):
        """Нормализует угол в диапазон [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def calculate_lookahead_point(self):
        """ Находит точку на пути на расстоянии lookahead_distance """
        if not self.current_path or not self.current_pose_map:
            return None

        current_x = self.current_pose_map.pose.position.x
        current_y = self.current_pose_map.pose.position.y

        # Находим ближайший сегмент пути
        closest_dist_sq = float('inf')
        closest_segment_start_idx = -1

        poses = self.current_path.poses
        num_poses = len(poses)

        # --- Пройдем по всем сегментам пути ---
        for i in range(max(0, self.target_waypoint_index - 1), num_poses - 1): # Начинаем проверку немного раньше текущей цели
             x1, y1 = poses[i].pose.position.x, poses[i].pose.position.y
             x2, y2 = poses[i+1].pose.position.x, poses[i+1].pose.position.y

             # Простая проверка расстояния до начальной точки сегмента (можно улучшить до расстояния до отрезка)
             dist_sq = (current_x - x1)**2 + (current_y - y1)**2

             if dist_sq < closest_dist_sq:
                 # --- Проверка, что эта точка не "позади" lookahead ---
                 # Вектор от дрона к началу сегмента
                 vec_drone_to_p1_x = x1 - current_x
                 vec_drone_to_p1_y = y1 - current_y
                 # Вектор сегмента
                 vec_segment_x = x2 - x1
                 vec_segment_y = y2 - y1
                 # Скалярное произведение
                 dot_product = vec_drone_to_p1_x * vec_segment_x + vec_drone_to_p1_y * vec_segment_y

                 # Если точка примерно на перпендикуляре или "впереди" по направлению сегмента
                 # Это очень грубая проверка, чтобы избежать выбора сегментов далеко позади
                 # if dot_product > -0.5 * ((vec_segment_x**2 + vec_segment_y**2)**0.5): # Порог -0.5 длины сегмента
                 closest_dist_sq = dist_sq
                 closest_segment_start_idx = i


        if closest_segment_start_idx == -1:
            # Если не нашли подходящий сегмент, возможно, используем последнюю точку?
            if num_poses > 0:
                 lookahead_pose = poses[-1].pose # Целимся в конец пути
                 self.publish_debug_lookahead(lookahead_pose)
                 return lookahead_pose
            else:
                 return None # Нет пути


        # --- Ищем точку на расстоянии lookahead_distance от дрона вдоль пути ---
        dist_travelled_from_closest_start = 0.0
        current_lookahead_pose = None

        for i in range(closest_segment_start_idx, num_poses - 1):
             x1, y1 = poses[i].pose.position.x, poses[i].pose.position.y
             x2, y2 = poses[i+1].pose.position.x, poses[i+1].pose.position.y

             seg_dx, seg_dy = x2 - x1, y2 - y1
             segment_len = math.sqrt(seg_dx**2 + seg_dy**2)

             # Расстояние от дрона до начала текущего рассматриваемого сегмента
             # Если это первый рассматриваемый сегмент (i == closest_segment_start_idx),
             # нужно найти проекцию дрона на этот сегмент (для точности),
             # но для упрощения пока будем считать от начала сегмента.
             dist_from_drone_to_seg_start = 0.0
             if i == closest_segment_start_idx:
                  # Приблизительно: расстояние до начала ближайшего сегмента
                 dist_from_drone_to_seg_start = math.sqrt(closest_dist_sq)
             else:
                 # Для последующих сегментов, начало - это конец предыдущего
                 dist_from_drone_to_seg_start = dist_travelled_from_closest_start


             # Сколько еще нужно пройти до lookahead distance
             dist_needed = self.lookahead_distance - dist_from_drone_to_seg_start

             if segment_len >= dist_needed and dist_needed > 0:
                 # Точка взгляда находится на этом сегменте
                 ratio = dist_needed / segment_len
                 lookahead_x = x1 + ratio * seg_dx
                 lookahead_y = y1 + ratio * seg_dy

                 current_lookahead_pose = PoseStamped()
                 current_lookahead_pose.header.frame_id = self.map_frame
                 current_lookahead_pose.header.stamp = rospy.Time.now()
                 current_lookahead_pose.pose.position.x = lookahead_x
                 current_lookahead_pose.pose.position.y = lookahead_y
                 current_lookahead_pose.pose.position.z = self.target_altitude # Держим высоту
                 # Ориентацию можно взять как направление сегмента
                 yaw = math.atan2(seg_dy, seg_dx)
                 q = tf.transformations.quaternion_from_euler(0, 0, yaw)
                 current_lookahead_pose.pose.orientation.x = q[0]
                 current_lookahead_pose.pose.orientation.y = q[1]
                 current_lookahead_pose.pose.orientation.z = q[2]
                 current_lookahead_pose.pose.orientation.w = q[3]
                 break # Нашли точку

             else:
                 # Точка взгляда дальше этого сегмента, переходим к следующему
                 dist_travelled_from_closest_start += segment_len


        # Если цикл закончился, а точка не найдена (lookahead дальше конца пути),
        # используем последнюю точку пути.
        if current_lookahead_pose is None and num_poses > 0:
             current_lookahead_pose = poses[-1].pose
             # Установим нужную высоту
             current_lookahead_pose.pose.position.z = self.target_altitude


        if current_lookahead_pose:
            self.publish_debug_lookahead(current_lookahead_pose)
            return current_lookahead_pose
        else:
            return None


    def check_waypoint_reached(self, target_pose):
        """Проверяет, достигнута ли целевая точка."""
        if not self.current_pose_map:
            return False
        dx = target_pose.pose.position.x - self.current_pose_map.pose.position.x
        dy = target_pose.pose.position.y - self.current_pose_map.pose.position.y
        # Игнорируем Z для достижения точки в 2D
        dist_sq = dx**2 + dy**2
        return dist_sq < self.waypoint_tolerance**2

    def calculate_control_commands(self):
        """Основная логика расчета управляющих команд."""
        if not self.current_path or not self.current_pose_map:
            rospy.logdebug_throttle(5.0, "Ожидание пути или позы...")
            # Если нет пути или позы, отправляем команду висения/disarm
            # (Нужно определить, что безопаснее)
            return self.get_hover_rc_commands(armed_state=self.armed) # Отправляем текущее состояние armed

        num_waypoints = len(self.current_path.poses)

        # --- Проверка завершения пути ---
        if self.target_waypoint_index >= num_waypoints:
            rospy.loginfo("Путь завершен.")
            self.current_path = None # Сбрасываем путь
            # Отправляем команду висения (или посадки?)
            return self.get_hover_rc_commands(armed_state=True) # Зависаем после пути

        # --- Получаем текущую целевую точку (не lookahead, а именно waypoint) ---
        current_target_wp = self.current_path.poses[self.target_waypoint_index]
        self.publish_debug_target(current_target_wp)

        # --- Проверяем, достигли ли текущей целевой точки ---
        if self.check_waypoint_reached(current_target_wp):
             rospy.loginfo(f"Достигнута точка {self.target_waypoint_index + 1}/{num_waypoints}")
             self.target_waypoint_index += 1
             # Проверяем снова, не вышли ли за пределы пути
             if self.target_waypoint_index >= num_waypoints:
                 rospy.loginfo("Путь завершен (после проверки достижения последней точки).")
                 self.current_path = None
                 return self.get_hover_rc_commands(armed_state=True)
             # Обновляем целевую точку для этого цикла
             current_target_wp = self.current_path.poses[self.target_waypoint_index]
             self.publish_debug_target(current_target_wp)
             # Сбрасываем PID yaw, т.к. цель изменилась
             self.pid_yaw.reset()


        # --- Используем точку взгляда (lookahead) для расчета направления ---
        lookahead_point = self.calculate_lookahead_point()
        if lookahead_point is None:
             rospy.logwarn("Не удалось рассчитать точку взгляда (lookahead). Используем текущую целевую точку.")
             # Если нет lookahead, используем текущий waypoint как цель для управления
             # Это может привести к менее плавному движению
             target_x = current_target_wp.pose.position.x
             target_y = current_target_wp.pose.position.y
             target_z = self.target_altitude # Всегда держим высоту
        else:
             target_x = lookahead_point.pose.position.x
             target_y = lookahead_point.pose.position.y
             target_z = self.target_altitude # Используем целевую высоту

        # --- Текущее состояние ---
        current_x = self.current_pose_map.pose.position.x
        current_y = self.current_pose_map.pose.position.y
        current_z = self.current_pose_map.pose.position.z
        current_yaw = self.get_yaw_from_quat(self.current_pose_map.pose.orientation)

        # --- Расчет ошибок ---
        error_x_map = target_x - current_x
        error_y_map = target_y - current_y
        error_z = target_z - current_z # target_altitude - current_z

        # --- Расчет желаемого курса (Yaw) ---
        # Направляем дрон на точку взгляда (lookahead)
        target_yaw = math.atan2(error_y_map, error_x_map)
        error_yaw = self.normalize_angle(target_yaw - current_yaw)

        # --- Преобразование ошибки XY в систему координат дрона ---
        # Это нужно для расчета Roll/Pitch
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        error_forward = error_x_map * cos_yaw + error_y_map * sin_yaw # Ошибка вдоль оси дрона
        error_strafe = -error_x_map * sin_yaw + error_y_map * cos_yaw # Ошибка поперек оси дрона

        # --- Вызов PID ---
        # Высота -> добавка к тяге
        throttle_adjustment = self.pid_z.update(current_z) # PID работает по ошибке error_z = target - current

        # Рыскание -> добавка к RC каналу Yaw
        # Устанавливаем setpoint для PID Yaw (желаемый курс)
        self.pid_yaw.setpoint = target_yaw
        # Т.к. pid работает по ошибке = setpoint - pv, а нам нужна ошибка = target - current,
        # можно либо передать -error_yaw, либо изменить логику PID.
        # Передадим текущий yaw, PID сам рассчитает ошибку = target_yaw - current_yaw
        # Важно: нужно обработать переходы через +/- PI внутри PID или перед передачей.
        # Простой PID класс может не справиться с этим сам. Передадим нормализованную ошибку.
        # self.pid_yaw.setpoint = 0 # Ставим setpoint в 0
        # yaw_adjustment = self.pid_yaw.update(-error_yaw) # Обновляем по отрицательной ошибке
        # Альтернатива: передаем текущий yaw, но нужно убедиться, что PID внутри корректно работает с углами
        # Давайте попробуем простой вариант: PID управляет ошибкой к нулю
        self.pid_yaw.setpoint = 0.0 # Хотим, чтобы error_yaw стала 0
        yaw_adjustment = self.pid_yaw.update(-error_yaw) # Подаем -ошибку, чтобы PID стремился ее убрать


        # --- Расчет Roll/Pitch напрямую из ошибок (простой P-контроллер) ---
        # Вместо PID скорости, используем пропорциональное управление по ошибкам в СК дрона
        # Нужны коэффициенты Kp_roll, Kp_pitch
        Kp_roll = rospy.get_param("~gain/roll", 100.0) # Пример! Нужна настройка
        Kp_pitch = rospy.get_param("~gain/pitch", 100.0)# Пример! Нужна настройка

        # Желаемый Roll пропорционален ошибке бокового смещения (strafe)
        # Знак зависит от конвенции дрона (+roll -> движение вправо?)
        roll_adjustment = Kp_roll * error_strafe
        # Желаемый Pitch пропорционален ошибке движения вперед (forward)
        # Знак зависит от конвенции дрона (-pitch -> движение вперед?)
        pitch_adjustment = -Kp_pitch * error_forward # Ставим минус, предполагая -pitch = вперед

        # --- Ограничение управляющих воздействий (добавок) ---
        roll_adjustment = max(min(roll_adjustment, self.rc_roll_range), -self.rc_roll_range)
        pitch_adjustment = max(min(pitch_adjustment, self.rc_pitch_range), -self.rc_pitch_range)
        # throttle_adjustment уже ограничен PID
        # yaw_adjustment уже ограничен PID


        # --- Формирование итоговых RC команд ---
        roll_cmd = int(self.rc_roll_center + roll_adjustment)
        pitch_cmd = int(self.rc_pitch_center + pitch_adjustment)
        throttle_cmd = int(self.rc_throttle_hover + throttle_adjustment)
        yaw_cmd = int(self.rc_yaw_center + yaw_adjustment)

        # Финальное ограничение команд диапазоном 1000-2000
        roll_cmd = max(min(roll_cmd, self.rc_max), self.rc_min)
        pitch_cmd = max(min(pitch_cmd, self.rc_max), self.rc_min)
        throttle_cmd = max(min(throttle_cmd, self.rc_max), self.rc_min)
        yaw_cmd = max(min(yaw_cmd, self.rc_max), self.rc_min)

        # Включаем ARM канал (пример)
        # Здесь нужна логика: когда армить? Только если есть путь и поза?
        # Пока будем считать, что если мы в этом методе, то должны быть заармлены
        current_arm_value = self.arm_value
        self.armed = True # Устанавливаем флаг

        # Формируем список для send_RAW_RC
        # Порядок каналов должен соответствовать ожиданиям MultirotorControl/INAV!
        # Пример: [Roll, Pitch, Throttle, Yaw, AUX1(ARM), AUX2(MODE), ...]
        # Добавьте нули или 1000 для неиспользуемых каналов, если нужно 8+ каналов
        rc_command = [
            roll_cmd,
            pitch_cmd,
            throttle_cmd,
            yaw_cmd,
            current_arm_value,
            self.mode_value, # Пример для Mode
            self.rc_min,       # Пример для AUX3
            self.rc_min        # Пример для AUX4
            # ... добавьте еще, если нужно
        ]

        # Убедимся, что команда содержит достаточное количество каналов (например, 8)
        while len(rc_command) < 8:
             rc_command.append(self.rc_min)

        rospy.logdebug(f"RC Command: R={roll_cmd} P={pitch_cmd} T={throttle_cmd} Y={yaw_cmd} | Target Yaw: {math.degrees(target_yaw):.1f}, Current Yaw: {math.degrees(current_yaw):.1f}, Error Yaw: {math.degrees(error_yaw):.1f}")
        rospy.logdebug(f"Errors: Fwd={error_forward:.2f}, Str={error_strafe:.2f}, Z={error_z:.2f}")
        return rc_command[:8] # Отправляем первые 8 каналов


    def get_hover_rc_commands(self, armed_state=False):
        """Возвращает RC команду для зависания или disarmed."""
        arm_cmd = self.arm_value if armed_state else self.disarm_value
        if not armed_state:
             self.armed = False # Обновляем флаг

        # Центрируем Roll, Pitch, Yaw. Throttle - висение или минимум.
        throttle_cmd = self.rc_throttle_hover if armed_state else self.rc_min
        throttle_cmd = max(min(throttle_cmd, self.rc_max), self.rc_min) # Ограничение

        rc_command = [
            self.rc_roll_center,
            self.rc_pitch_center,
            throttle_cmd,
            self.rc_yaw_center,
            arm_cmd,
            self.mode_value,
            self.rc_min,
            self.rc_min
        ]
        rospy.logdebug(f"Hover/Disarm Command: T={throttle_cmd}, Armed={armed_state}")
        return rc_command[:8]

    def control_loop_callback(self, event):
        """Основной цикл управления."""
        if not self.tcp_transmitter.is_connect:
             rospy.logwarn_throttle(5.0, "Нет подключения к передатчику. Пытаюсь переподключиться...")
             if not self.connect_transmitter(retries=1, delay=0): # Быстрая попытка
                  rospy.logerr_throttle(5.0,"Переподключение не удалось. Пропуск цикла управления.")
                  # Не отправляем команды, если нет связи
                  return
             else:
                  # Если подключились, продолжаем
                  pass

        if not self.get_current_pose():
            # Если не можем получить позу, отправляем команду disarm/hover для безопасности
            rospy.logwarn_throttle(1.0, "Не удалось получить позу. Отправка команды зависания/disarm.")
            rc_cmd = self.get_hover_rc_commands(armed_state=False) # Disarm для безопасности
            self.send_rc_command(rc_cmd)
            return

        # Рассчитываем управляющие команды
        rc_command = self.calculate_control_commands()

        # Отправляем команду
        if rc_command:
            self.send_rc_command(rc_command)
        else:
             # Если calculate_control_commands вернул None (например, ошибка),
             # отправляем безопасную команду
             rospy.logwarn_throttle(1.0, "Не удалось рассчитать команду. Отправка команды зависания/disarm.")
             rc_cmd = self.get_hover_rc_commands(armed_state=False) # Disarm
             self.send_rc_command(rc_cmd)


    def send_rc_command(self, rc_values):
        """Отправляет RC команду через MultirotorControl."""
        if not self.tcp_transmitter.is_connect:
             rospy.logerr("Попытка отправки команды при отсутствии подключения!")
             return False
        try:
            # Убедимся, что все значения - целые числа
            rc_values_int = [int(round(v)) for v in rc_values]
            # Ваш метод send_RAW_RC ожидает список
            res = self.control.send_RAW_RC(rc_values_int)
            if res == 0:
                 rospy.logwarn_throttle(1.0, "Ошибка отправки RC команды (send_RAW_RC вернул 0)")
                 # Потеря соединения?
                 if not self.tcp_transmitter.is_connect: # Проверка статуса после отправки
                      rospy.logerr("Соединение с TCP передатчиком потеряно!")
                 return False
            # rospy.logdebug(f"Отправлена RC команда: {rc_values_int}")
            return True
        except Exception as e:
            rospy.logerr(f"Исключение при отправке RC команды: {e}")
            # Возможно, соединение разорвано
            if self.tcp_transmitter.is_connect:
                try:
                    self.tcp_transmitter.disconnect() # Попробуем закрыть
                except: pass
            return False

    def shutdown_hook(self):
        """Выполняется при завершении работы узла."""
        rospy.logwarn("Path Follower Node завершает работу...")
        # Отправить команду Disarm перед отключением
        if self.tcp_transmitter and self.tcp_transmitter.is_connect and self.control:
            rospy.loginfo("Отправка команды Disarm...")
            disarm_cmd = self.get_hover_rc_commands(armed_state=False)
            self.send_rc_command(disarm_cmd)
            time.sleep(0.1) # Небольшая пауза
            self.send_rc_command(disarm_cmd) # Повторно на всякий случай
            time.sleep(0.1)

        # Закрыть соединение
        if self.tcp_transmitter and self.tcp_transmitter.is_connect:
            try:
                self.tcp_transmitter.disconnect()
                rospy.loginfo("Соединение с TCP передатчиком закрыто.")
            except Exception as e:
                rospy.logerr(f"Ошибка при закрытии TCP соединения: {e}")

    # --- Функции отладки ---
    def publish_debug_target(self, target_pose):
         """Публикует текущую целевую точку для RViz."""
         if self.debug_target_pose_pub.get_num_connections() > 0:
              debug_pose = PoseStamped()
              debug_pose.header = target_pose.header # Используем тот же stamp и frame
              if not debug_pose.header.frame_id: # Если у пути нет frame_id
                   debug_pose.header.frame_id = self.map_frame
              debug_pose.header.stamp = rospy.Time.now() # Текущее время
              debug_pose.pose = target_pose.pose
              # Установим правильную высоту для визуализации
              debug_pose.pose.position.z = self.target_altitude
              self.debug_target_pose_pub.publish(debug_pose)

    def publish_debug_lookahead(self, lookahead_pose):
         """Публикует точку взгляда для RViz."""
         if self.debug_lookahead_pub.get_num_connections() > 0:
              # lookahead_pose уже PoseStamped, просто публикуем
              self.debug_lookahead_pub.publish(lookahead_pose)


# --- Точка входа ---
if __name__ == '__main__':
    try:
        # --- Импорт tf.transformations после инициализации rospy, если он нужен ---
        # (В данной версии он не используется явно, но может понадобиться)
        import tf.transformations

        node = PathFollowerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path Follower Node прерван.")
    except ImportError as e:
         # Ловим ошибку импорта tf.transformations, если она возникнет
         rospy.logfatal(f"Ошибка импорта: {e}. Убедитесь, что пакет 'tf' (ROS Geometry) установлен.")
    except Exception as e:
        rospy.logfatal(f"Необработанное исключение в Path Follower: {e}", exc_info=True)