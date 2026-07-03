import serial
import threading
import time
from scservo_sdk import *

# =========================
# STM32 ADC 시리얼
# =========================
PORT_ADC = "COM13"
BAUD_ADC = 115200

adc_raw = [0] * 22
parsed = [0] * 16
running = True

# =========================
# STS3215 설정
# =========================
DEVICENAME = "COM12"
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# =========================
# 왼팔 모터
# =========================
MOTORS = [1, 2, 3, 4, 5, 6, 7]

# =========================
# 방어 파라미터 (오른팔과 동일 구조)
# =========================
EMA_ALPHA = 0.1
ema_values = [None] * 7

DEAD_ZONE = 12
MAX_DELTA = 50

system_ready = False
startup_count = 0
STARTUP_WAIT = 80

FLOATING_THRESHOLD = 4080


# =========================
# ADC 스레드
# =========================
def read_serial_adc():
    global adc_raw, parsed, running

    try:
        ser = serial.Serial(PORT_ADC, BAUD_ADC, timeout=1)
    except Exception as e:
        print("시리얼 열기 실패:", e)
        return

    while running:
        try:
            line = ser.readline().decode(errors="ignore")
            line = line.replace("\x00", "").strip()

            if not line:
                continue

            parts = line.split(",")

            if len(parts) < 22:
                continue

            for i in range(min(22, len(parts))):
                val_str = "".join(filter(str.isdigit, parts[i]))
                if val_str != "":
                    adc_raw[i] = int(val_str)

            # 왼팔: MUX C1~C7 → parsed[0]~[6]
            for i in range(7):
                parsed[i] = adc_raw[i + 1]

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

        if not system_ready:
            startup_count += 1
            if startup_count >= STARTUP_WAIT and any(parsed[i] > 0 for i in range(7)):
                system_ready = True
                for i in range(7):
                    ema_values[i] = float(parsed[i])
                print(">>> 시스템 준비 완료, 모터 제어 시작")
            else:
                time.sleep(0.02)
                continue

        # =========================
        # 왼팔 모터 1~7
        # =========================
        for i in range(7):

            m = MOTORS[i]
            raw = parsed[i]

            if raw >= FLOATING_THRESHOLD:
                continue

            # EMA 필터
            if ema_values[i] is None:
                ema_values[i] = float(raw)
            else:
                ema_values[i] = EMA_ALPHA * raw + (1 - EMA_ALPHA) * ema_values[i]

            if i == 6:
                # ── 그리퍼(7번) 전용 처리 ──────────────────────────
                GRIPPER_ADC_MIN = 145  # 완전히 쥐었을 때 ADC (실측)
                GRIPPER_ADC_MAX = 1270  # 완전히 놓았을 때 ADC (실측)
                GRIPPER_POS_OPEN = 4100  # 그리퍼 열림 모터 위치
                GRIPPER_POS_CLOSE = 500  # 그리퍼 닫힘 모터 위치 (낮출수록 더 닫힘)
                adc = int(ema_values[6])
                ratio = max(
                    0.0,
                    min(
                        1.0,
                        (adc - GRIPPER_ADC_MIN) / (GRIPPER_ADC_MAX - GRIPPER_ADC_MIN),
                    ),
                )
                tick = int(
                    GRIPPER_POS_CLOSE + ratio * (GRIPPER_POS_OPEN - GRIPPER_POS_CLOSE)
                )

            else:
                tick = int(ema_values[i])

            # MAX_DELTA
            if prev_ticks[i] is not None:
                delta = tick - prev_ticks[i]
                if delta > MAX_DELTA:
                    tick = prev_ticks[i] + MAX_DELTA
                elif delta < -MAX_DELTA:
                    tick = prev_ticks[i] - MAX_DELTA

            # Dead Zone
            if prev_ticks[i] is not None and abs(tick - prev_ticks[i]) <= DEAD_ZONE:
                continue

            packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, tick)
            prev_ticks[i] = tick

        # =========================
        # 출력
        # =========================
        # =========================
        # 출력
        # =========================
        print("\033[F", end="")

        raw_str = " ".join([f"{parsed[i]:5d}" for i in range(7)])
        flt_str = " ".join([f"{int(v) if v else 0:5d}" for v in ema_values])

        print(
            f"RAW:{raw_str} | FLT:{flt_str}"
            + f" GRIPPER:{prev_ticks[6] if prev_ticks[6] is not None else 0:4d}"
        )

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
