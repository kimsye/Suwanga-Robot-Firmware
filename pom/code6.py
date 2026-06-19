import serial
import threading
import time
from scservo_sdk.port_handler import PortHandler
from scservo_sdk.packet_handler import PacketHandler

# =========================
# [설계 반영] 1. 시스템 상태 정의 (FSM)
# =========================
class SystemState:
    IDLE = 0
    MOVE = 1
    ERROR = 2

# =========================
# 설정 및 상수
# =========================
PORT_ADC = "COM6"
BAUD_ADC = 115200
DEVICENAME = 'COM10'
BAUDRATE = 1000000
PROTOCOL_END = 0

MOTORS = [1, 2, 3, 4, 5, 6, 7]
MIN_TICK, MAX_TICK = 500, 3500

# [설계 반영] 2. 신호 처리 상수
FILTER_SIZE = 5
DEAD_ZONE = 15      # 미세 떨림 방지를 위한 무시 범위
THRESHOLD_MOVE = 10 # 동작 시작 임계값

running = True
current_state = SystemState.IDLE

# 데이터 저장소
adc_raw = [0] * 7
adc_history = [[0] * FILTER_SIZE for _ in range(7)]
prev_ticks = [None] * 7

# =========================
# [설계 반영] 3. 핵심 로듈화 (Control/Security)
# =========================

def moving_average_filter(index, new_val):
    """신호 처리: 노이즈 제거"""
    adc_history[index].pop(0)
    adc_history[index].append(new_val)
    return sum(adc_history[index]) // FILTER_SIZE

def apply_dead_zone(prev_val, current_val):
    """신호 처리: Dead Zone 및 Smoothing"""
    if prev_val is None: return current_val
    if abs(prev_val - current_val) < DEAD_ZONE:
        return prev_val  # 변화가 작으면 무시 (미세 떨림 방지)
    return current_val

def integrity_check(parts):
    """보안: 입력 데이터 무결성 검증 (CSV 형식 검사)"""
    # 설계서의 Checksum은 C코드 수정이 필요하므로, 여기선 구조적 무결성 검사 수행
    if len(parts) < 7:
        return False
    for p in parts[:7]:
        if not ''.join(filter(str.isdigit, p)): return False
    return True

def log_status(msg, level="INFO"):
    """통신: UART 스타일 로그 출력"""
    timestamp = time.strftime("%H:%M:%S")
    print(f"[{timestamp}] [{level}] {msg}")

# =========================
# 시리얼 수신 스레드
# =========================
def read_serial_adc():
    global adc_raw, running, current_state
    try:
        ser = serial.Serial(PORT_ADC, BAUD_ADC, timeout=0.1)
    except Exception as e:
        log_status(f"시리얼 개방 실패: {e}", "ERROR")
        current_state = SystemState.ERROR
        return

    while running:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if not line: continue

            parts = line.split(",")
            
            # [보안 반영] 무결성 검증
            if not integrity_check(parts):
                log_status("데이터 무결성 검증 실패 (Invalid Packet)", "WARN")
                continue

            for i in range(7):
                val = int(''.join(filter(str.isdigit, parts[i])))
                if 0 <= val <= 4095:
                    adc_raw[i] = val
                else:
                    log_status(f"이상값 감지 (CH{i}: {val})", "WARN")

        except Exception as e:
            continue

# =========================
# 메인 제어 루프
# =========================
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_END)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    log_status("모터 포트 연결 실패", "ERROR")
    quit()

# 모터 초기화
for m in MOTORS:
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_TORQUE_ENABLE, 1)
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_ACCELERATION, 50)

threading.Thread(target=read_serial_adc, daemon=True).start()

log_status("시스템 초기화 완료. IDLE 상태 진입.")

try:
    while True:
        if current_state == SystemState.ERROR:
            log_status("시스템 에러 상태 - 동작 중단", "ERROR")
            break

        moved = False
        output_display = []

        for i, m in enumerate(MOTORS):
            # 1. 필터링
            filtered = moving_average_filter(i, adc_raw[i])
            
            # 2. Tick 변환
            target_tick = int(MIN_TICK + (filtered / 4095.0) * (MAX_TICK - MIN_TICK))
            
            # 3. Dead Zone 적용 (안정화)
            safe_tick = apply_dead_zone(prev_ticks[i], target_tick)

            # 4. 상태 제어 및 출력
            if safe_tick != prev_ticks[i]:
                packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, safe_tick)
                prev_ticks[i] = safe_tick
                moved = True
            
            output_display.append(f"{filtered:4d}")

        # FSM 상태 전환 로직
        if moved and current_state == SystemState.IDLE:
            current_state = SystemState.MOVE
            log_status("상태 전환: IDLE -> MOVE")
        elif not moved and current_state == SystemState.MOVE:
            # 일정 시간 변화 없으면 IDLE 복귀 (여기선 단순화)
            current_state = SystemState.IDLE
            log_status("상태 전환: MOVE -> IDLE")

        # 실시간 모니터링 출력
        print(f"\r[{'MOVE' if moved else 'IDLE'}] ADC: {'|'.join(output_display)}", end="")
        time.sleep(0.05)

except KeyboardInterrupt:
    log_status("사용자 종료 요청")
finally:
    running = False
    for m in MOTORS:
        packetHandler.write1ByteTxRx(portHandler, m, ADDR_TORQUE_ENABLE, 0)
    portHandler.closePort()
    log_status("시스템 안전 종료")