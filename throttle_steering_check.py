import socket
import time
import threading

# --- НАСТРОЙКА UDP ПОРТОВ ДЛЯ SERIAL4 И SERIAL5 ---
# Скрипт слушает эти порты на Mac на всех интерфейсах (0.0.0.0)
FARDRIVER_UDP_PORT = 14554  # Сюда ArduPilot шлет данные из SERIAL4
MODBUS_UDP_PORT = 14555     # Сюда ArduPilot шлет данные из SERIAL5
HOST = '0.0.0.0'

# --- ИМИТАЦИЯ СОСТОЯНИЯ ТРАНСПОРТА ---
vehicle_state = {
    "target_power": 0,
    "direction": 0,
    "rpm": 0,
    "voltage": 52.4,
    "current": 0.0,
    "temperature": 35,
    "stepper_pos": 0,
    "stepper_error": 0
}

# --- ПОТОК 1: СИМУЛЯТОР FARDRIVER (UDP) ---
def fardriver_udp_simulator():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, FARDRIVER_UDP_PORT))
    print(f"[Fardriver UDP Sim] Успешно запущен. Слушает порт {FARDRIVER_UDP_PORT}...")
    
    buffer = bytearray()
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            # Отладочный вывод сырых данных для проверки связи с ArduPilot
            print(f"[Fardriver] ПОЛУЧЕНЫ БАЙТЫ ОТ ARDUPILOT: {data.hex().upper()} от {addr}")
            
            for byte in data:
                if len(buffer) == 0 and byte != 0xAA:
                    continue
                buffer.append(byte)
                
                if len(buffer) == 16:
                    calculated_sum = sum(buffer[:14])
                    received_sum = (buffer[14] << 8) | buffer[15]
                    
                    if calculated_sum == received_sum:
                        vehicle_state["target_power"] = buffer[2]
                        vehicle_state["direction"] = buffer[3]
                        
                        # Моделируем простую физику ходового мотора
                        power_factor = vehicle_state["target_power"] / 100.0
                        vehicle_state["rpm"] = int(power_factor * 3000)
                        vehicle_state["current"] = power_factor * 25.5
                        vehicle_state["temperature"] = 35 + int(power_factor * 15) if power_factor > 0 else 35
                        
                        # Собираем ответный пакет телеметрии (17 байт)
                        tx_buf = bytearray(17)
                        tx_buf[0] = 0xAA  # Маркер начала
                        
                        # Обороты RPM (Байты 4 и 5)
                        tx_buf[4] = (vehicle_state["rpm"] >> 8) & 0xFF
                        tx_buf[5] = vehicle_state["rpm"] & 0xFF
                        
                        # Напряжение * 10 (Байты 6 и 7)
                        raw_volt = int(vehicle_state["voltage"] * 10)
                        tx_buf[6] = (raw_volt >> 8) & 0xFF
                        tx_buf[7] = raw_volt & 0xFF
                        
                        # Ток * 10 (Байты 8 и 9)
                        raw_curr = int(vehicle_state["current"] * 10)
                        tx_buf[8] = (raw_curr >> 8) & 0xFF
                        tx_buf[9] = raw_curr & 0xFF
                        
                        # Температура (Байт 10)
                        tx_buf[10] = vehicle_state["temperature"]
                        
                        # Считаем CRC-16 (простая сумма первых 14 байт)
                        crc = sum(tx_buf[:14])
                        tx_buf[14] = (crc >> 8) & 0xFF
                        tx_buf[15] = crc & 0xFF
                        tx_buf[16] = 0x0D  # Стоп-байт
                        
                        sock.sendto(tx_buf, addr)
                    buffer.clear()
        except Exception as e:
            print(f"[Fardriver UDP Error] Ошибка: {e}")
            time.sleep(0.1)

# --- ВСПОМОГАТЕЛЬНАЯ ФУНКЦИЯ ДЛЯ MODBUS CRC-16 ---
def modbus_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

# --- ПОТОК 2: СИМУЛЯТОР ШАГОВИКА MODBUS (UDP) ---
def modbus_udp_simulator():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, MODBUS_UDP_PORT))
    print(f"[Modbus UDP Sim] Успешно запущен. Слушает порт {MODBUS_UDP_PORT}...")
    
    # Буфер для накопления склеенного трафика от SITL
    raw_buffer = bytearray()
    
    while True:
        try:
            # Читаем данные из сокета и добавляем их в общий буфер
            data, addr = sock.recvfrom(1024)
            raw_buffer.extend(data)
            
            # Разбираем склеенный буфер, пока в нём есть данные
            while len(raw_buffer) >= 4:
                slave_id = raw_buffer[0]
                func_code = raw_buffer[1]
                
                # Определяем ожидаемую длину фрейма на основе кода функции Modbus RTU
                if func_code == 0x06:
                    frame_len = 8   # ID(1) + Func(1) + Addr(2) + Val(2) + CRC(2)
                elif func_code == 0x10:
                    if len(raw_buffer) < 7:
                        break # Ждем досылки заголовка многорегистровой записи
                    bytes_count = raw_buffer[6]
                    frame_len = 7 + bytes_count + 2 # Header(7) + Data(N) + CRC(2)
                elif func_code == 0x03:
                    frame_len = 8   # ID(1) + Func(1) + Addr(2) + Count(2) + CRC(2)
                else:
                    # Если поймали мусорный байт, сдвигаем буфер на 1 и ищем маркер заново
                    raw_buffer.pop(0)
                    continue
                
                # Если фрейм прилетел еще не полностью, выходим из внутреннего цикла и ждем recvfrom
                if len(raw_buffer) < frame_len:
                    break
                
                # Вырезаем текущий фрейм из общего буфера для обработки
                req = raw_buffer[:frame_len]
                del raw_buffer[:frame_len]
                
                # Валидируем CRC конкретно этого вырезанного фрейма
                received_crc = (req[-1] << 8) | req[-2]
                if modbus_crc(req[:-2]) != received_crc:
                    continue
                    
                print(f"[Modbus] РАЗОБРАН ФРЕЙМ: {req.hex().upper()} от {addr}")
                
                reg_addr = (req[2] << 8) | req[3]
                response = bytearray()
                
                if func_code == 0x06:  # Блокировка обмоток шаговика
                    val = (req[4] << 8) | req[5]
                    response.extend(req[:6])
                    
                elif func_code == 0x10:  # Запись целевой позиции руля
                    if reg_addr == 0x0034:
                        # ИСПРАВЛЕНИЕ: Извлекаем 32-битное число из пакета ArduPilot.
                        # В Modbus RTU (функция 0x10) данные 32-битного регистра идут в байтах 7, 8, 9, 10.
                        # Проверяем формат Low-Word First (стандарт Leadshine):
                        low_word = (req[7] << 8) | req[8]
                        high_word = (req[9] << 8) | req[10]
                        
                        # Если колеса в симуляторе выкрутятся не в ту сторону, 
                        # просто поменяйте их местами:
                        # low_word = (req[9] << 8) | req[10]
                        # high_word = (req[7] << 8) | req[8]

                        pulses = (high_word << 16) | low_word
                        
                        # Восстанавливаем знак для отрицательных шагов (поворот влево)
                        if pulses & (1 << 31):
                            pulses -= 1 << 32
                            
                        vehicle_state["stepper_pos"] = pulses
                    response.extend(req[:6])

                    
                elif func_code == 0x03:  # Чтение позиции CL57R
                    if reg_addr == 0x001C:
                        # Принудительно маскируем позицию под 32-битное беззнаковое число
                        current_pos = int(vehicle_state["stepper_pos"]) & 0xFFFFFFFF
                        
                        # Безопасно выделяем младшее и старшее слово для Modbus
                        low_word = current_pos & 0xFFFF
                        high_word = (current_pos >> 16) & 0xFFFF
                        
                        response.append(slave_id)
                        response.append(0x03)
                        response.append(0x04) # Byte count
                        response.append((low_word >> 8) & 0xFF)
                        response.append(low_word & 0xFF)
                        response.append((high_word >> 8) & 0xFF)
                        response.append(high_word & 0xFF)
                        # Итого 7 байт. Дальше скрипт считает modbus_crc(response) и добавляет еще 2 байта CRC. Всего в сокет улетает 9 байт.


                # Отправляем ответ на этот конкретный фрейм НЕМЕДЛЕННО
                if len(response) > 0:
                    crc = modbus_crc(response)
                    response.append(crc & 0xFF)
                    response.append((crc >> 8) & 0xFF)
                    sock.sendto(response, addr)
                    
        except Exception as e:
            print(f"[Modbus UDP Error] Ошибка: {e}")
            raw_buffer.clear()
            time.sleep(0.1)

# --- ГЛАВНАЯ ТОЧКА ВХОДА (ОСНОВНОЙ ЦИКЛ ОЖИДАНИЯ) ---
if __name__ == "__main__":
    print("\n=== ИНИЦИАЛИЗАЦИЯ UDP СИМУЛЯТОРА ВЕЗДЕХОДА ===")
    
    # Инициализируем и запускаем фоновые потоки драйверов
    t1 = threading.Thread(target=fardriver_udp_simulator, daemon=True)
    t2 = threading.Thread(target=modbus_udp_simulator, daemon=True)
    
    t1.start()
    t2.start()
    
    print("=== ВСЕ СЛУЖБЫ ЗАПУЩЕНЫ. ОЖИДАНИЕ ТРАФИКА ОТ SITL ===\n")
    
    # Бесконечный цикл с выгрузкой статуса (не дает скрипту закрыться)
    try:
        while True:
            print(f"[STATUS] Газ: {vehicle_state['target_power']}% | "
                  f"Обороты: {vehicle_state['rpm']} RPM | "
                  f"Руль (Шаги): {vehicle_state['stepper_pos']}")
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nСимулятор остановлен пользователем.")