import socket
import time

# --- КОНФИГУРАЦИЯ СЕТЕВЫХ ПОРТОВ ---
HOST = "127.0.0.1"
MODBUS_UDP_PORT = 14555     # Соответствует --serial5=udpclient:127.0.0.1:14555

# --- СОСТОЯНИЕ ШАГОВИКА ---
stepper_state = {
    "stepper_pos": 0,
    "stepper_error": 0,
    "last_rx_hex": "WAITING...",
    "position_updated": False  # Предохранитель: проверяет, приходила ли позиция 0x10 в этом цикле
}

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

def run_modbus_simulator():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, MODBUS_UDP_PORT))
    print(f"[CL57R Modbus Sim] Успешно запущен. Слушает UDP порт {MODBUS_UDP_PORT}...")
    
    raw_buffer = bytearray()
    last_log_time = time.time()
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            raw_buffer.extend(data)
            
            while len(raw_buffer) >= 4:
                slave_id = raw_buffer[0]
                func_code = raw_buffer[1]
                
                # Определение длины фрейма Modbus RTU
                if func_code == 0x06:
                    frame_len = 8   
                elif func_code == 0x10:
                    if len(raw_buffer) < 7:
                        break 
                    bytes_count = raw_buffer[6]
                    frame_len = 7 + bytes_count + 2 
                elif func_code == 0x03:
                    frame_len = 8   
                else:
                    raw_buffer.pop(0)
                    continue
                
                if len(raw_buffer) < frame_len:
                    break
                
                req = raw_buffer[:frame_len]
                del raw_buffer[:frame_len]
                
                # Проверка Modbus CRC
                received_crc = (req[-1] << 8) | req[-2]
                if modbus_crc(req[:-2]) != received_crc:
                    continue
                
                # Сохраняем успешно валидированный сырой Modbus-запрос в HEX
                stepper_state["last_rx_hex"] = req.hex().upper()
                    
                reg_addr = (req[2] << 8) | req[3]
                response = bytearray()
                
                # --- ФУНКЦИЯ 0x06: ОДИНОЧНАЯ ЗАПИСЬ ---
                if func_code == 0x06:
                    val = (req[4] << 8) | req[5]
                    
                    if reg_addr == 0x0038:
                        stepper_state["stepper_error"] = 0 if val == 1 else 1
                        
                    # АППАРАТНАЯ ПРОВЕРКА ПОСЛЕДОВАТЕЛЬНОСТИ КОМАНД
                    elif reg_addr == 0x0036:  # Прилетел триггер старта шаговика
                        if not stepper_state["position_updated"]:
                            # КРИТИЧЕСКИЙ СБОЙ: Автопилот запрашивает старт, не обновив координаты руля!
                            print(f"\n[CRITICAL ERROR] Trigger 0x0036 received, but Position 0x0034 was NOT sent! (Last RX HEX: {stepper_state['last_rx_hex']})")
                            stepper_state["stepper_error"] = 0xAA  # Выставляем системный код ошибки
                        else:
                            # Пакеты пришли в правильном порядке. Сбрасываем флаг для следующего цикла
                            stepper_state["position_updated"] = False
                            if stepper_state["stepper_error"] == 0xAA:
                                stepper_state["stepper_error"] = 0
                                
                    response.extend(req[:6])
                    
                # --- ФУНКЦИЯ 0x10: ЗАПИСЬ ЦЕЛЕВОЙ ПОЗИЦИИ (Синхронизация Endianness с ArduPilot) ---
                elif func_code == 0x10:
                    regs_count_high = req[4]
                    regs_count_low = req[5]
                    
                    if reg_addr == 0x0034:
                        # ИСПРАВЛЕНИЕ: Из-за специфики паковщика ArduPilot, старший байт 32-битного значения
                        # прилетает в начале массива данных.
                        low_word = (req[7] << 8) | req[8]
                        high_word = (req[9] << 8) | req[10]
                        
                        pulses = (high_word << 16) | low_word
                        if pulses & (1 << 31):
                            pulses -= 1 << 32
                            
                        stepper_state["stepper_pos"] = pulses
                        stepper_state["position_updated"] = True  # Фиксируем успешное обновление позиции
                    
                    response.append(slave_id)
                    response.append(0x10)
                    response.append(req[2])   
                    response.append(req[3])   
                    response.append(regs_count_high)
                    response.append(regs_count_low)
                    
                # --- ФУНКЦИЯ 0x03: ЧТЕНИЕ ЭНКОДЕРА (Регистры 0x0007-0x0008, High-Word First) ---
                elif func_code == 0x03:
                    if reg_addr == 0x0007:
                        current_pos = int(stepper_state["stepper_pos"]) & 0xFFFFFFFF
                        
                        # Выделяем слова строго по паспорту CL57R (High-Word First)
                        high_word = (current_pos >> 16) & 0xFFFF
                        low_word = current_pos & 0xFFFF
                        
                        response.append(slave_id)
                        response.append(0x03)
                        response.append(0x04) 
                        
                        # ИСПРАВЛЕНИЕ ПОРЯДКА: Сначала отправляем старшее слово (0x0007), затем младшее (0x0008)
                        response.append((high_word >> 8) & 0xFF)
                        response.append(high_word & 0xFF)
                        response.append((low_word >> 8) & 0xFF)
                        response.append(low_word & 0xFF)

                if len(response) > 0:
                    crc = modbus_crc(response)
                    response.append(crc & 0xFF)         
                    response.append((crc >> 8) & 0xFF)   
                    sock.sendto(response, addr)
            
            # Вывод объединенного лога раз в 1.5 секунды строго в одну строку
            if time.time() - last_log_time >= 1.5:
                approx_angle = (stepper_state["stepper_pos"] / 1000.0) * 360.0
                
                # Маркер состояния ошибки для наглядности в терминале
                err_status = "0xAA (MISSING_POS)" if stepper_state["stepper_error"] == 0xAA else str(stepper_state["stepper_error"])
                
                print(f"[STEER] Steps: {stepper_state['stepper_pos']:6d} | "
                      f"Angle: {approx_angle:+.1f} deg | "
                      f"Err: {err_status} | "
                      f"Raw RX: {stepper_state['last_rx_hex']}")
                last_log_time = time.time()
                    
        except Exception as e:
            print(f"[Modbus Error] Error: {e}")
            raw_buffer.clear()
            time.sleep(0.1)

if __name__ == "__main__":
    run_modbus_simulator()
