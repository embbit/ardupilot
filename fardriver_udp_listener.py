import socket
import time
import threading

# --- НАСТРОЙКА UDP ПОРТОВ ДЛЯ SERIAL4 И SERIAL5 ---
FARDRIVER_UDP_PORT = 14554  # Будет привязан к SERIAL4 (-D)
MODBUS_UDP_PORT = 14555     # Будет привязан к SERIAL5 (-E)
HOST = '127.0.0.1'

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
    print(f"[Fardriver UDP Sim] Слушает порт {FARDRIVER_UDP_PORT} (для SERIAL4)")
    
    buffer = bytearray()
    while True:
        try:
            data, addr = sock.recvfrom(1024)
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
                        
                        # Моделируем физику
                        power_factor = vehicle_state["target_power"] / 100.0
                        vehicle_state["rpm"] = int(power_factor * 3000)
                        vehicle_state["current"] = power_factor * 25.5
                        vehicle_state["temperature"] = 35 + int(power_factor * 15) if power_factor > 0 else 35
                        
                        # Ответная телеметрия
                        tx_buf = bytearray(17)
                        tx_buf[0] = 0xAA
                        tx_buf[4] = (vehicle_state["rpm"] >> 8) & 0xFF
                        tx_buf[5] = vehicle_state["rpm"] & 0xFF
                        
                        raw_volt = int(vehicle_state["voltage"] * 10)
                        tx_buf[6] = (raw_volt >> 8) & 0xFF
                        tx_buf[7] = raw_volt & 0xFF
                        
                        raw_curr = int(vehicle_state["current"] * 10)
                        tx_buf[8] = (raw_curr >> 8) & 0xFF
                        tx_buf[9] = raw_curr & 0xFF
                        
                        tx_buf[10] = vehicle_state["temperature"]
                        
                        crc = sum(tx_buf[:14])
                        tx_buf[14] = (crc >> 8) & 0xFF
                        tx_buf[15] = crc & 0xFF
                        tx_buf[16] = 0x0D
                        
                        sock.sendto(tx_buf, addr)
                    buffer.clear()
        except Exception as e:
            print(f"[Fardriver UDP] Ошибка: {e}")
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
    print(f"[Modbus UDP Sim] Слушает порт {MODBUS_UDP_PORT} (для SERIAL5)")
    
    while True:
        try:
            req, addr = sock.recvfrom(1024)
            if len(req) < 4:
                continue
                
            slave_id = req[0]
            func_code = req[1]
            
            received_crc = (req[-1] << 8) | req[-2]
            if modbus_crc(req[:-2]) != received_crc:
                continue
                
            reg_addr = (req[2] << 8) | req[3]
            response = bytearray()
            
            if func_code == 0x06:
                val = (req[4] << 8) | req[5]
                print("[Modbus UDP] Обмотки рулевого привода заблокированы!")
                response.extend(req[:6])
                
            elif func_code == 0x10:
                if reg_addr == 0x0034:
                    pulses = (req[7] << 24) | (req[8] << 16) | (req[9] << 8) | req[10]
                    if pulses & (1 << 31):
                        pulses -= 1 << 32
                    vehicle_state["stepper_pos"] = pulses
                response.extend(req[:6])
                
            elif func_code == 0x03:
                if reg_addr == 0x0004:
                    response.append(slave_id)
                    response.append(0x03)
                    response.append(0x02)
                    response.append((vehicle_state["stepper_error"] >> 8) & 0xFF)
                    response.append(vehicle_state["stepper_error"] & 0xFF)
            
            if len(response) > 0:
                crc = modbus_crc(response)
                response.append(crc & 0xFF)
                response.append((crc >> 8) & 0xFF)
                sock.sendto(response, addr)
                
        except Exception as e:
            print(f"[Modbus UDP] Ошибка: {e}")
            time.sleep(0.1)

if __name__ == "__main__":
    print("=== ЗАПУСК UDP СИМУЛЯТОРА ДЛЯ SERIAL4 И SERIAL5 ===")
    t1 = threading.Thread(target=fardriver_udp_simulator, daemon=True)
    t2 = threading.Thread(target=modbus_udp_simulator, daemon=True)
    t1.start()
    t2.start()
    
    try:
        while True:
            print(f"[STATUS] Газ: {vehicle_state['target_power']}% | "
                  f"Обороты: {vehicle_state['rpm']} RPM | "
                  f"Руль (Шаги): {vehicle_state['stepper_pos']}")
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nUDP Симулятор остановлен.")