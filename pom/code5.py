import serial
import threading
import time
from scservo_sdk import *

# =========================
# STM32 ADC 시리얼
# =========================
PORT_ADC = "COM6"
BAUD_ADC = 115200

# ★ 10채널로 변경
adc = [0] * 10
running = True


# =========================
# ★ SCS0009 팬틸트
# =========================
PAN_ID  = 12
TILT_ID = 11

PAN_MIN  = 200
PAN_MAX  = 800

TILT_MIN = 400
TILT_MAX = 700

# =========================
# STS3215 설정
# =========================
DEVICENAME = 'COM9'
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# ★ 모터 1~7, 11, 12   
MOTORS = [1, 2, 3, 4, 5, 6, 7, 11, 12]

MIN_TICK = 500
MAX_TICK = 3500

# =========================
# ADC → 시리얼 읽기
# =========================
def read_serial_adc():
    global adc, running

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

            # print("RAW:", line)

            # ★ "adc0,adc1,...adc6"
            parts = line.split(",")

            for i in range(min(10, len(parts))):
                val_str = ''.join(filter(str.isdigit, parts[i]))
                if val_str != "":
                    adc[i] = int(val_str)

        except Exception as e:
            print("ERR:", e)

# =========================
# ADC → PAN
# 0~4095 -> 200~800
# =========================
def adc_to_pan(value):
    if value < 0:
        value = 0
    if value > 4095:
        value = 4095
    return int(
        PAN_MIN +
        (value / 4095.0) * (PAN_MAX - PAN_MIN)
    )
# =========================
# ADC → TILT
# 0~4095 -> 400~700
# =========================
def adc_to_tilt(value):
    if value < 0:
        value = 0
    if value > 4095:
        value = 4095
    return int(
        TILT_MIN +
        (value / 4095.0) * (TILT_MAX - TILT_MIN)
    )

# =========================
# ADC → 모터 변환
# =========================
def adc_to_tick(value):
    if value < 0:
        value = 0
    if value > 4095:
        value = 4095

    return int(MIN_TICK + (value / 4095.0) * (MAX_TICK - MIN_TICK))

# =========================
# 메인
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
# 모터 1~7 활성화
# =========================
for m in MOTORS:
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_ACCELERATION, 50)

# ADC 스레드 시작
t = threading.Thread(target=read_serial_adc)
t.daemon = True
t.start()

print("시작")
print("  A0    A1    A2    A3    A4   A5   A6   A7   A8   A9")
print("--------------------------------------------------------")
print("")  # 데이터 자리 확보

# =========================
# 루프
# =========================
prev_ticks = [None] * 7

try:
    while True:

        # =========================
        # 기존 모터 1~7
        # =========================
        for i in range(7):

            m = MOTORS[i]

            tick = adc_to_tick(adc[i])

            if tick != prev_ticks[i]:

                packetHandler.write1ByteTxRx(
                    portHandler,
                    m,
                    ADDR_ACCELERATION,
                    50
                )

                packetHandler.write2ByteTxRx(
                    portHandler,
                    m,
                    ADDR_GOAL_POSITION,
                    tick
                )

                prev_ticks[i] = tick

        # =========================
        # 조이스틱 팬틸트
        # adc[7] = X
        # adc[8] = Y
        # adc[9] = sw_toggle
        # =========================
        if adc[9] == 1:

            pan_tick  = adc_to_pan(adc[7])
            tilt_tick = adc_to_tilt(adc[8])

            packetHandler.write2ByteTxRx(
                portHandler,
                PAN_ID,
                ADDR_GOAL_POSITION,
                pan_tick
            )

            packetHandler.write2ByteTxRx(
                portHandler,
                TILT_ID,
                ADDR_GOAL_POSITION,
                tilt_tick
            )

        # 출력
        print("\033[F", end="")

        print(
            " ".join([f"{adc[i]:5d}" for i in range(10)])
        )

        time.sleep(0.05)

except KeyboardInterrupt:
    pass

# =========================
# 종료
# =========================
running = False

for m in MOTORS:
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

portHandler.closePort()

print("종료")