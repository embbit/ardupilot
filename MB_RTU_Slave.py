import socket
import sys

HOST = "127.0.0.1"
MODBUS_UDP_PORT = 14555

def calculate_crc(data):
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

def parse_modbus_request(raw_bytes):
    if len(raw_bytes) < 4:
        return "Пакет слишком короткий"
        
    slave_id = raw_bytes[0]
    func_code = raw_bytes[1]
    reg_addr = (raw_bytes[2] << 8) | raw_bytes[3]
    
    info = f"[Slave ID: {slave_id:02X}] | [Функция: 0x{func_code:02X}] | "
    
    if func_code == 0x03:
        count = (raw_bytes[4] << 8) | raw_bytes[5]
        info += f"ЧТЕНИЕ регистров с 0x{reg_addr:04X} (Кол-во: {count})"
    elif func_code == 0x06:
        val = (raw_bytes[4] << 8) | raw_bytes[5]
        info += f"ЗАПИСЬ (Single) в 0x{reg_addr:04X} -> Значение: {val} (0x{val:04X})"
    elif func_code == 0x10:
        reg_count = (raw_bytes[4] << 8) | raw_bytes[5]
        # Извлекаем байты данных (минус ID, Func, Addr(2), Count(2), ByteCount(1) и CRC(2))
        data_payload = raw_bytes[7:-2].hex().upper()
        info += f"ЗАПИСЬ (Multiple) в 0x{reg_addr:04X} (Регистров: {reg_count}, Данные: {data_payload})"
    else:
        info += f"Неизвестная функция на регистр 0x{reg_addr:04X}"
        
    return info

def run_sniffer():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass
        
    try:
        sock.bind((HOST, MODBUS_UDP_PORT))
        print(f"[MONITOR] Пассивный сниффер потока запущен на порту {MODBUS_UDP_PORT}...\n")
    except Exception as e:
        print(f"[ERROR] Не удалось занять порт: {e}")
        sys.exit(1)
        
    while True:
        try:
            data, addr = sock.recvfrom(2048)
            packet_buffer = bytearray(data)
            
            print(f"=== [Новая UDP датаграмма от {addr[0]}:{addr[1]} | Размер: {len(data)} байт] ===")
            
            while len(packet_buffer) >= 4:
                slave_id = packet_buffer[0]
                func_code = packet_buffer[1]
                
                if func_code in (0x03, 0x06):
                    frame_len = 8   
                elif func_code == 0x10:
                    if len(packet_buffer) < 7:
                        break 
                    bytes_count = packet_buffer[6]
                    frame_len = 7 + bytes_count + 2 
                else:
                    # Если поймали мусор, удаляем один байт и ищем заново
                    packet_buffer.pop(0)
                    continue
                
                if len(packet_buffer) < frame_len:
                    print(f" -> [!] Неполный кадр функции 0x{func_code:02X} (осталось {len(packet_buffer)} из {frame_len} байт)")
                    break
                
                # Извлекаем один Modbus-кадр
                modbus_frame = packet_buffer[:frame_len]
                del packet_buffer[:frame_len]
                
                # Считаем CRC
                expected_crc = calculate_crc(modbus_frame[:-2])
                received_crc = (modbus_frame[-1] << 8) | modbus_frame[-2]
                crc_status = "OK" if expected_crc == received_crc else f"ОШИБКА CRC (Ожидалось: {expected_crc:04X}, Получено: {received_crc:04X})"
                
                frame_hex = " ".join(f"{b:02X}" for b in modbus_frame)
                parsed_text = parse_modbus_request(modbus_frame)
                
                print(f"  Кадр (HEX) : {frame_hex}")
                print(f"  Протокол   : {parsed_text}")
                print(f"  Статус CRC : {crc_status}")
                print("  " + "-" * 50)
                
            print("=" * 70 + "\n")
            
        except KeyboardInterrupt:
            print("\nСниффер остановлен.")
            break
        except Exception as e:
            # Выводим реальную ошибку, если она возникнет внутри парсера
            print(f"[Parser Error]: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    run_sniffer()
