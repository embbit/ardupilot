import socket
import time

# --- КОНФИГУРАЦИЯ СЕТЕВЫХ ПОРТОВ ---
HOST = "127.0.0.1"
FARDRIVER_UDP_PORT = 14554  # Соответствует --serial4=udpclient:127.0.0.1:14554

# --- СОСТОЯНИЕ МОТОРА ---
motor_state = {
    "target_power": 0,
    "direction": 0,
    "rpm": 0,
    "voltage": 84.0,
    "current": 0.0,
    "line_current": 0.0,
    "phase_current": 0.0,
    "temperature_mot": 35,
    "temperature_ecu": 35,
    "charge_state": 98,
    "last_rx_hex": "WAITING..."  # Хранилище для последней сырой команды управления
}

def run_fardriver_simulator():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, FARDRIVER_UDP_PORT))
    print(f"[FarDriver Sim] Успешно запущен. Слушает UDP порт {FARDRIVER_UDP_PORT}...")
    
    buffer = bytearray()
    last_log_time = time.time()

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            
            for byte in data:
                if len(buffer) == 0 and byte != 0xAA:
                    continue
                buffer.append(byte)
                
                # Прием пакета управления от ArduPilot (16 байт)
                if len(buffer) == 16:
                    calculated_sum = sum(buffer[:14])
                    received_sum = (buffer[14] << 8) | buffer[15]
                    
                    if calculated_sum == received_sum:
                        # Сохраняем сырой кадр от полетного контроллера в HEX-формате
                        motor_state["last_rx_hex"] = buffer.hex().upper()
                        
                        motor_state["target_power"] = buffer[2]
                        motor_state["direction"] = buffer[3]
                        
                        # Моделируем физику параметров под нагрузкой
                        power_factor = motor_state["target_power"] / 100.0
                        motor_state["rpm"] = int(power_factor * 2800)
                        motor_state["current"] = power_factor * 35.5
                        motor_state["line_current"] = power_factor * 35.5  
                        motor_state["phase_current"] = power_factor * 75.0 
                        
                        if power_factor > 0:
                            motor_state["temperature_mot"] = 35 + int(power_factor * 12)
                            motor_state["temperature_ecu"] = motor_state["temperature_mot"] + 8
                        else:
                            motor_state["temperature_mot"] = 35
                            motor_state["temperature_ecu"] = 35
                        
                        # Сборка телеметрии по ТЗ АНК (строго 27 байт)
                        tx_buf = bytearray(27)
                        tx_buf[0] = 0xAA  
                        tx_buf[1] = 0x00  
                        
                        tx_buf[2] = (motor_state["rpm"] >> 8) & 0xFF
                        tx_buf[3] = motor_state["rpm"] & 0xFF

                        raw_curr = int(motor_state["current"] * 4)
                        tx_buf[4] = (raw_curr >> 8) & 0xFF
                        tx_buf[5] = raw_curr & 0xFF
                        
                        raw_volt = int(motor_state["voltage"] * 10)
                        tx_buf[6] = (raw_volt >> 8) & 0xFF
                        tx_buf[7] = raw_volt & 0xFF
                        
                        tx_buf[8] = motor_state["temperature_mot"] & 0xFF
                        tx_buf[9] = motor_state["temperature_ecu"] & 0xFF
                        tx_buf[10] = 0x02  # Статус: Связь с контроллером ОК
                        tx_buf[11] = motor_state["charge_state"]
                        tx_buf[12] = 0x00 
                        
                        # Байты 16-17: Линейный ток (дискреты 1/4 А)
                        raw_line_curr = int(motor_state["line_current"] * 4)
                        tx_buf[16] = (raw_line_curr >> 8) & 0xFF
                        tx_buf[17] = raw_line_curr & 0xFF

                        # Байты 18-19: Фазный ток (дискреты 1/4 А)
                        raw_phase_curr = int(motor_state["phase_current"] * 4)
                        tx_buf[18] = (raw_phase_curr >> 8) & 0xFF
                        tx_buf[19] = raw_phase_curr & 0xFF
                        
                        # Расчет CRC (суммируем первые 25 байт, пишем в 25 и 26)
                        crc = sum(tx_buf[:25])
                        tx_buf[25] = (crc >> 8) & 0xFF
                        tx_buf[26] = crc & 0xFF
                        
                        sock.sendto(tx_buf, addr)
                        
                    buffer.clear()

            # Вывод лога раз в 1.5 секунды в одну компактную строку
            if time.time() - last_log_time >= 1.5:
                print(f"[MOTOR] Gas: {motor_state['target_power']:3d}% | "
                      f"Mode: {'R' if motor_state['direction'] == 1 else 'D'} | "
                      f"RPM: {motor_state['rpm']:4d} | "
                      f"Line/Phase Cur: {motor_state['line_current']:.1f}A / {motor_state['phase_current']:.1f}A | "
                      f"Raw RX: {motor_state['last_rx_hex']}")
                last_log_time = time.time()

        except Exception as e:
            print(f"[FarDriver Error] Ошибка: {e}")
            buffer.clear()
            time.sleep(0.1)

if __name__ == "__main__":
    run_fardriver_simulator()
