from inavmspapi import MultirotorControl
from inavmspapi.transmitter import TCPTransmitter
import keyboard
import time

HOST = '127.0.0.1'
PORT = 5762
ADDRESS = (HOST, PORT)

# Подключаемся к симулятору/полетному контроллеру
tcp_transmitter = TCPTransmitter(ADDRESS)
tcp_transmitter.connect()
control = MultirotorControl(tcp_transmitter)

trottle = 1390  # Инициализация сохранённого значения тяги

def get_rc_control():
    """Получает значения RC управления на основе ввода с клавиатуры."""
    global trottle
    # Стандартные значения RC: [Крен, Тангаж, Газ, Рыскание, Aux1, Aux2, Aux3, ...]
    # 1500 - центр для Крена, Тангажа, Рыскания. 1000 - минимальный газ.
    rc_control = [1500, 1500, trottle, 1500, 2000, 1000, 1000] # Предполагаем Aux1 - ARM, Aux2/3 по умолчанию
    speed = 200  # Скорость изменения значений для стандартного крена/тангажа
    fast_speed_multiplier = 3 # Множитель для быстрого крена/тангажа
    yaw_speed = 100 # Скорость изменения рыскания
    throttle_coarse_step = 10 # Шаг грубой настройки газа
    throttle_fine_step = 1   # Шаг точной настройки газа

    # --- Управление Тангажом ---
    if keyboard.is_pressed('w'):
        rc_control[1] = 1500 + speed  # Тангаж вперед
    elif keyboard.is_pressed('s'):
        rc_control[1] = 1500 - speed  # Тангаж назад

    # --- Управление Креном ---
    if keyboard.is_pressed('d'):
        rc_control[0] = 1500 + speed  # Крен вправо
    elif keyboard.is_pressed('a'):
        rc_control[0] = 1500 - speed  # Крен влево

    # --- Быстрое Управление Тангажом ---
    if keyboard.is_pressed('u'):
        rc_control[1] = 1500 + speed * fast_speed_multiplier # Быстрый Тангаж вперед
    elif keyboard.is_pressed('j'):
        rc_control[1] = 1500 - speed * fast_speed_multiplier # Быстрый Тангаж назад

    # --- Быстрое Управление Креном ---
    if keyboard.is_pressed('k'):
        rc_control[0] = 1500 + speed * fast_speed_multiplier # Быстрый Крен вправо
    elif keyboard.is_pressed('h'):
        rc_control[0] = 1500 - speed * fast_speed_multiplier # Быстрый Крен влево

    # --- Управление Газом (Грубо) ---
    if keyboard.is_pressed('m'):
        trottle = min(trottle + throttle_coarse_step, 2000)  # Увеличение тяги (грубо)
    elif keyboard.is_pressed('n'):
        trottle = max(trottle - throttle_coarse_step, 1000)  # Уменьшение тяги (грубо)

    # --- Управление Рысканием ---
    if keyboard.is_pressed('e'):
        rc_control[3] = 1500 + yaw_speed  # Рыскание по часовой
    elif keyboard.is_pressed('q'):
        rc_control[3] = 1500 - yaw_speed  # Рыскание против часовой

    # --- Управление Газом (Точно) ---
    if keyboard.is_pressed('x'):
        trottle = min(trottle + throttle_fine_step, 2000)  # Увеличение тяги (точно)
    elif keyboard.is_pressed('z'):
        trottle = max(trottle - throttle_fine_step, 1000)  # Уменьшение тяги (точно)

    # Применяем потенциально обновленное значение газа
    rc_control[2] = trottle  # Применение сохранённого значения тяги

    return rc_control

def main():
    """Основной цикл для отправки RC команд."""
    print("Скрипт RC Управления Запущен")
    print("------------------------------------")
    print("Управление:")
    print(" W/S: Тангаж Вперед/Назад")
    print(" A/D: Крен Влево/Вправо")
    print(" Q/E: Рыскание Влево/Вправо")
    print(" M/N: Газ Вверх/Вниз (Грубо)")
    print(" Z/X: Газ Вверх/Вниз (Точно)")
    print(" U/J: Быстрый Тангаж Вперед/Назад")
    print(" H/K: Быстрый Крен Влево/Вправо")
    print("------------------------------------")
    print("Отправка начальной нейтральной команды (газ низкий)...")
    control.send_RAW_RC([1500, 1500, 1000, 1500, 1000, 1000, 1000])
    time.sleep(0.5)

    control.send_RAW_RC([1500, 1500, 1000, 1500, 2000, 1000, 1000])
    time.sleep(0.5)
    print("Запуск цикла управления...")

    try:
        while True:
            rc_control_values = get_rc_control()
            control.send_RAW_RC(rc_control_values)
            print(rc_control_values)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nОстановка скрипта и отправка нейтральной команды...")
        # Отправляем нейтральную команду с низким газом перед выходом
        control.send_RAW_RC([1500, 1500, 1000, 1500, 1000, 1000, 1000]) 
        time.sleep(0.1)
        tcp_transmitter.disconnect()
        print("Отключено.")
    except Exception as e:
        print(f"\nПроизошла ошибка: {e}")
        print("Попытка отправки нейтральной команды...")
        try:
            control.send_RAW_RC([1500, 1500, 1000, 1500, 1000, 1000, 1000]) 
            time.sleep(0.1)
            tcp_transmitter.disconnect()
            print("Отключено.")
        except:
            print("Не удалось отправить нейтральную команду или отключиться.")


if __name__ == "__main__":
    print("Запрос повышенных прав, если необходимо для мониторинга клавиатуры...")
    main()