import serial
import threading
import time
from scservo_sdk import *
#--------------
from pantilt_safe2 import update_pantilt
from pantilt_safe2 import scs_write_pos
from pantilt_safe2 import pan_pos
from pantilt_safe2 import tilt_pos


#-----------
#from pantilt import update_pantilt
#from pantilt import scs_write_pos
#from pantilt import pan_pos
#from pantilt import tilt_pos

# =========================
# STM32 ADC 시리얼
# =========================
PORT_ADC = "COM13"
BAUD_ADC = 115200

# 원본 STM32 데이터 저장용
adc_raw = [0] * 22 #19에서 22로 변경

# 실제 사용할 ADC 데이터
parsed = [0] * 16

# SW
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

REVERSE_CHANNELS = [0, 3, 4, 5, 6]  # 9, 12, 13, 14, 15번 반전

# =====================================================
# [추가] 방어 파라미터
# =====================================================

# EMA 필터 (0에 가까울수록 부드러움, 1에 가까울수록 민감)
EMA_ALPHA = 0.1
ema_values = [None] * 7  # 모터 7채널용

# Dead Zone (이 이하의 변화는 무시)
DEAD_ZONE = 12

# 변화량 제한 (1 루프당 최대 허용 변화)
MAX_DELTA = 50

# 시작 안정화
system_ready = False
startup_count = 0
STARTUP_WAIT = 50  # 약 1초

# =====================================================

# =========================
# ADC 스레드
# =========================
def read_serial_adc():

    global adc_raw
    global parsed
    global sw_toggle
    global running

    try:
        ser = serial.Serial(
            PORT_ADC,
            BAUD_ADC,
            timeout=1
        )

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

            # =========================
            # ADC 저장
            # =========================
            for i in range(min(22, len(parts))): #22로 변경

                val_str = ''.join(filter(str.isdigit, parts[i]))

                if val_str != "":
                    adc_raw[i] = int(val_str)

            # =========================
            # MUX ADC
            # =========================
            for i in range(7):
                parsed[i] = adc_raw[i + 1]

            for i in range(7):
                parsed[i + 7] = adc_raw[i + 9]

            # =========================
            # IND ADC
            # =========================
            parsed[14] = adc_raw[16]
            parsed[15] = adc_raw[17]

            # =========================
            # SW (디지털)
            # =========================
            sw_toggle = adc_raw[20] #숫자 18에서 20으로 바꿈

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
# 모터 초기화
# =========================
for m in MOTORS:

    packetHandler.write1ByteTxRx(portHandler, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_ACCELERATION, 50)

packetHandler.write1ByteTxRx(portHandler, PAN_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
packetHandler.write1ByteTxRx(portHandler, TILT_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

packetHandler.write1ByteTxRx(portHandler, PAN_ID, ADDR_ACCELERATION, 50)
packetHandler.write1ByteTxRx(portHandler, TILT_ID, ADDR_ACCELERATION, 50)

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

# =========================
# 메인 루프
# =========================
prev_ticks = [None] * 7

try:

    while True:

        # =====================================================
        # [추가] 시작 안정화 대기
        # ADC 스레드가 실제 값을 받기 전에 모터 명령 차단
        # =====================================================
        if not system_ready:
            startup_count += 1
            if startup_count >= STARTUP_WAIT and any(parsed[i + 7] > 0 for i in range(7)):
                system_ready = True
                # EMA 초기값을 현재 값으로 설정 (급발진 방지)
                for i in range(7):
                    ema_values[i] = float(parsed[i + 7])
                print(">>> 시스템 준비 완료, 모터 제어 시작")
            else:
                time.sleep(0.02)
                continue

        # =========================
        # 모터 9~15
        # parsed[7]~[13] → 모터 제어용
        # ADC → EMA → 변화량 제한 → Dead Zone → 모터
        # =========================
        for i in range(7):

            m = MOTORS[i]
            raw = parsed[i + 7]

            # EMA 필터
            if ema_values[i] is None:
                ema_values[i] = float(raw)
            else:
                ema_values[i] = EMA_ALPHA * raw + (1 - EMA_ALPHA) * ema_values[i]

#-----------------그리퍼 로직 분리------------------
            if i == 6:
                # ── 그리퍼(15번) 전용 처리 ──────────────────────────────
                # adc_monitor.py 로 실측 후 아래 두 값을 수정하세요
                GRIPPER_ADC_MIN  = 2973   # 조이스틱 릴리즈 시 ADC 값
                GRIPPER_ADC_MAX  = 3993   # 조이스틱 최대 시 ADC 값
                GRIPPER_POS_OPEN  = 3935  # 그리퍼 열림 모터 위치
                GRIPPER_POS_CLOSE = 500   # 그리퍼 닫힘 모터 위치 (낮출수록 더 닫힘)
                adc = int(ema_values[6])
                ratio = max(0.0, min(1.0, (adc - GRIPPER_ADC_MIN) / (GRIPPER_ADC_MAX - GRIPPER_ADC_MIN)))
                # 반전 (닫힘→열림)
                tick = int(GRIPPER_POS_CLOSE + ratio * (GRIPPER_POS_OPEN - GRIPPER_POS_CLOSE))  # 반전

            else:
                # ── 일반 모터 처리 ───────────────────────────────────────
                tick = int(ema_values[i])
                if i in REVERSE_CHANNELS:
                    tick = 4095 - tick

#---------------------------------------------------
            # MAX_DELTA (매핑 후 적용 → prev_ticks와 같은 스케일)
            if prev_ticks[i] is not None:
                delta = tick - prev_ticks[i]
                if delta > MAX_DELTA:
                    tick = prev_ticks[i] + MAX_DELTA
                elif delta < -MAX_DELTA:
                    tick = prev_ticks[i] - MAX_DELTA

            # Dead Zone
            if prev_ticks[i] is not None and abs(tick - prev_ticks[i]) <= DEAD_ZONE:
                continue
            
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
        # 팬틸트 업데이트
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

        # [변경] raw vs filtered 비교 출력
        raw_str = " ".join([f"{parsed[i+7]:5d}" for i in range(7)])
        flt_str = " ".join([f"{int(v) if v else 0:5d}" for v in ema_values])

        print(
            f"RAW:{raw_str} | FLT:{flt_str}"
            + f" SW:{sw_toggle}"
            + f" PAN:{pan_pos:4d}"
            + f" TILT:{tilt_pos:4d}"
        )

        time.sleep(0.02)

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