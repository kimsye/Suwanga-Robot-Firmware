import serial
import threading
import time
from scservo_sdk import *

from pantilt import update_pantilt
from pantilt import scs_write_pos
from pantilt import pan_pos
from pantilt import tilt_pos

# =========================
# STM32 ADC 시리얼
# =========================
PORT_ADC = "COM13"
BAUD_ADC = 115200

adc_raw = [0] * 22 #19에서 22로 변경
parsed = [0] * 16
sw_toggle = 0
running = True

# =========================
# 팬틸트 ID
# =========================
PAN_ID  = 22
TILT_ID = 21

# =========================
# STS3215 설정
# =========================
DEVICENAME = 'COM14'
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# =========================
# 일반 모터
# =========================
MOTORS = [9, 10, 11, 12, 13, 14, 15]

# =====================================================
# 방어 파라미터 (떨림 완전 제거용)
# =====================================================

# EMA 필터
EMA_ALPHA = 0.08
ema_values = [None] * 7

# 히스테리시스 Dead Zone
DEAD_ZONE = 15
HYSTERESIS = 5  # Dead Zone 안에 있었으면 탈출 임계값 추가
dz_inside = [False] * 7  # 각 채널이 Dead Zone 안에 있는지

# 변화량 제한
MAX_DELTA = 15

# 시작 안정화
system_ready = False
startup_count = 0
STARTUP_WAIT = 80  # 약 1.6초 (충분히 기다림)

# 이상값 차단: 4080 이상이면 가변저항 미연결로 판단
FLOATING_THRESHOLD = 4080

# =====================================================

# =========================
# ADC 스레드
# =========================
def read_serial_adc():
    global adc_raw, parsed, sw_toggle, running

    try:
        ser = serial.Serial(PORT_ADC, BAUD_ADC, timeout=1)
    except Exception as e:
        print("시리얼 열기 실패:", e)
        return

    while running:
        try:
            line = ser.readline().decode(errors='ignore')
            line = line.replace('\x00', '').strip()

            if not line:
                continue

            parts = line.split(",")

            if len(parts) < 22: #파싱조건 19에서 22로 변경
                continue

            for i in range(min(19, len(parts))):
                val_str = ''.join(filter(str.isdigit, parts[i]))
                if val_str != "":
                    adc_raw[i] = int(val_str)

            for i in range(7):
                parsed[i] = adc_raw[i + 1]
            for i in range(7):
                parsed[i + 7] = adc_raw[i + 9]

            parsed[14] = adc_raw[16]
            parsed[15] = adc_raw[17]
            sw_toggle = adc_raw[20] #maincv2.c기준 sw0_toggle 20으로 변경 

        except Exception as e:
            print("ERR:", e)

# =========================
# 포트 오픈
# =========================
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_END)

if not portHandler.openPort():
    print("포트 열기 실패")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print("보레이트 실패")
    quit()

# =========================
# 모터 초기화 (Acceleration 낮춤 → 모터 자체 부드러움)
# =========================
for m in MOTORS:
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_ACCELERATION, 20)

packetHandler.write1ByteTxRx(portHandler, PAN_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
packetHandler.write1ByteTxRx(portHandler, TILT_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
packetHandler.write1ByteTxRx(portHandler, PAN_ID, ADDR_ACCELERATION, 20)
packetHandler.write1ByteTxRx(portHandler, TILT_ID, ADDR_ACCELERATION, 20)

# =========================
# 중앙 이동
# =========================
scs_write_pos(packetHandler, portHandler, PAN_ID, 511)
time.sleep(0.5)
scs_write_pos(packetHandler, portHandler, TILT_ID, 511)
time.sleep(0.5)

# =========================
# ADC thread
# =========================
t = threading.Thread(target=read_serial_adc)
t.daemon = True
t.start()

print("시작")
print("시스템 안정화 대기 중...")

# =========================
# 메인 루프
# =========================
prev_ticks = [None] * 7
# 각 채널이 가변저항 연결되어 있는지 (시작 시 판별)
channel_valid = [True] * 7

try:
    while True:

        # =====================================================
        # 시작 안정화 대기
        # =====================================================
        if not system_ready:
            startup_count += 1
            if startup_count >= STARTUP_WAIT:
                system_ready = True

                for i in range(7):
                    raw = parsed[i + 7]
                    ema_values[i] = float(raw)

                    # 가변저항 미연결 채널 판별
                    if raw >= FLOATING_THRESHOLD:
                        channel_valid[i] = False
                        print(f"  채널 {i} (모터{MOTORS[i]}): 미연결 (값={raw}) → 제어 안 함")
                    else:
                        channel_valid[i] = True
                        print(f"  채널 {i} (모터{MOTORS[i]}): 정상 (값={raw})")

                print(">>> 시스템 준비 완료")
            else:
                time.sleep(0.02)
                continue

        # =========================
        # 모터 9~15
        # =========================
        for i in range(7):

            # 미연결 채널은 건너뜀 (4095 노이즈 차단)
            if not channel_valid[i]:
                continue

            m = MOTORS[i]
            raw = parsed[i + 7]

            # 런타임 이상값 차단 (갑자기 4095로 튀면 무시)
            if raw >= FLOATING_THRESHOLD:
                continue

            # EMA 필터
            if ema_values[i] is None:
                ema_values[i] = float(raw)
            else:
                ema_values[i] = EMA_ALPHA * raw + (1 - EMA_ALPHA) * ema_values[i]

            tick = int(ema_values[i])

            # 변화량 제한 (급발진 방지)
            if prev_ticks[i] is not None:
                delta = tick - prev_ticks[i]
                if delta > MAX_DELTA:
                    tick = prev_ticks[i] + MAX_DELTA
                elif delta < -MAX_DELTA:
                    tick = prev_ticks[i] - MAX_DELTA

            # 히스테리시스 Dead Zone
            if prev_ticks[i] is not None:
                threshold = DEAD_ZONE
                if dz_inside[i]:
                    threshold += HYSTERESIS  # 안에 있었으면 탈출 더 어렵게

                if abs(tick - prev_ticks[i]) <= threshold:
                    dz_inside[i] = True
                    continue  # 모터 명령 안 보냄
                else:
                    dz_inside[i] = False

            packetHandler.write2ByteTxRx(
                portHandler,
                m,
                ADDR_GOAL_POSITION,
                tick
            )

            prev_ticks[i] = tick

        # =========================
        # 팬틸트 업데이트
        # =========================
        update_pantilt(
            parsed,
            sw_toggle,
            packetHandler,
            portHandler,
            PAN_ID,
            TILT_ID
        )

        # =========================
        # 출력
        # =========================
        print("\033[F", end="")

        status = ""
        for i in range(7):
            if not channel_valid[i]:
                status += "  --- "
            elif ema_values[i] is not None:
                status += f"{int(ema_values[i]):5d} "
            else:
                status += "    ? "

        print(
            f"FLT: {status}"
            + f"SW:{sw_toggle}"
            + f" PAN:{pan_pos:4d}"
            + f" TILT:{tilt_pos:4d}"
        )

        time.sleep(0.01)

except KeyboardInterrupt:
    pass

# =========================
# 종료
# =========================
running = False

for m in MOTORS:
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

packetHandler.write1ByteTxRx(portHandler, PAN_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, TILT_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

portHandler.closePort()
print("종료")