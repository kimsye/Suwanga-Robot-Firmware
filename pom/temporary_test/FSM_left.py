import serial
import threading
import time
from scservo_sdk import *

# ================================
# fsm 추가 =======================================
# =====================================
system_ready = False
startup_count = 0
STARTUP_WAIT = 80  # 50 → 80 (ADC 안정화 시간 늘림)

# =====================================================
# [추가] FSM 상태 정의 (오른팔과 동일한 개념, 왼팔 독립 상태)
# =====================================================
STATE_IDLE = "IDLE"
STATE_MOVE = "MOVE"
STATE_ERROR = "ERROR"

current_state = STATE_IDLE

idle_confirm_count = 0
IDLE_CONFIRM_LOOPS = 10


def check_anomaly(parsed, ema_values, prev_ticks):
    """
    [자리표시자] 왼팔 1단계 룰 기반 이상탐지 (다음 todo 항목에서 구현)
    왼팔은 parsed[6](그리퍼) ADC 이상 의심 건이 있으니,
    실제 구현 시 이 채널을 우선 점검 대상으로 포함할 것
    """
    return False


# =======================================

# =========================
# STM32 ADC 시리얼
# =========================
PORT_ADC = "COM13"
BAUD_ADC = 115200

# 원본 STM32 데이터 저장용
adc_raw = [0] * 22

# 실제 사용할 ADC 데이터
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

# =====================================================
# 방어 파라미터
# =====================================================
# alpha 추가 !!!!!!!!!!!!!!!!-------
EMA_ALPHA_ARM = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
EMA_ALPHA_GRIPPER = 0.3

ema_values = [None] * 7
# alpha 추가 !!!!!!!!!!!!!!!!-------
# =====================================================
# [추가] 히스테리시스 Dead Zone
# =====================================================
DEAD_ZONE_ENTER = 7
DEAD_ZONE_EXIT = 14
in_dead_zone = [False] * 7
# =================================

MAX_DELTA = 50

system_ready = False
startup_count = 0
STARTUP_WAIT = 80  # 50 → 80 (ADC 안정화 시간 늘림)

# 모터별 위치 한계 (EEPROM 설정과 일치)
MOTOR_LIMITS = {7: (935, 1055)}

# 부유 채널 감지 임계값
FLOATING_THRESHOLD = 4080

# =====================================================


# =========================
# ADC 스레드
# =========================
def read_serial_adc():

    global adc_raw
    global parsed
    global running

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
            if startup_count >= STARTUP_WAIT and any(
                0 < parsed[i] < FLOATING_THRESHOLD for i in range(7)
            ):
                system_ready = True
                for i in range(7):
                    if parsed[i] < FLOATING_THRESHOLD:
                        ema_values[i] = float(parsed[i])
                print(">>> 시스템 준비 완료, 모터 제어 시작")
            else:
                time.sleep(0.02)
                continue

        # =====================================================
        # [추가] FSM: ERROR 판단 (최우선 체크)
        # =====================================================
        if check_anomaly(parsed, ema_values, prev_ticks):
            current_state = STATE_ERROR

        if current_state == STATE_ERROR:
            # TODO: 다음 단계에서 안전 정지/복구 절차 정의
            print(">>> ERROR 상태 — 모터 명령 차단")
            time.sleep(0.02)
            continue

        # =====================================================
        # [추가] FSM: IDLE ↔ MOVE 판단
        # 부유 채널(raw >= FLOATING_THRESHOLD)은 판단에서 제외 —
        # 미연결 채널 때문에 항상 MOVE로 오판되는 것 방지
        # =====================================================
        all_channels_idle = all(
            prev_ticks[i] is None or in_dead_zone[i] or parsed[i] >= FLOATING_THRESHOLD
            for i in range(7)
        )

        if all_channels_idle:
            idle_confirm_count += 1
            if idle_confirm_count >= IDLE_CONFIRM_LOOPS:
                current_state = STATE_IDLE
        else:
            idle_confirm_count = 0
            current_state = STATE_MOVE

        # =========================
        # 왼팔 모터 1~7
        # =========================
        for i in range(7):
            m = MOTORS[i]
            raw = parsed[i]

            if raw >= FLOATING_THRESHOLD:
                continue

            alpha = EMA_ALPHA_GRIPPER if i == 6 else EMA_ALPHA_ARM[i]

            if ema_values[i] is None:
                ema_values[i] = float(raw)
            else:
                ema_values[i] = alpha * raw + (1 - alpha) * ema_values[i]
            # alpha 추가 !!!!!!!!!!!!!!!!-------

            if i == 6:
                # 그리퍼(7번) 전용 처리 - adc_monitor.py로 실측 후 아래 값 수정
                GRIPPER_ADC_MIN = 841  # 조이스틱 릴리즈 시 ADC 값
                GRIPPER_ADC_MAX = 1283  # 조이스틱 최대 시 ADC 값
                GRIPPER_POS_MIN = 935  # 그리퍼 최소 위치 (EEPROM 한계)
                GRIPPER_POS_MAX = 1055  # 그리퍼 최대 위치 (EEPROM 한계)
                adc = int(ema_values[6])
                ratio = max(
                    0.0,
                    min(
                        1.0,
                        (adc - GRIPPER_ADC_MIN) / (GRIPPER_ADC_MAX - GRIPPER_ADC_MIN),
                    ),
                )
                tick = int(
                    GRIPPER_POS_MIN + ratio * (GRIPPER_POS_MAX - GRIPPER_POS_MIN)
                )

            else:
                tick = int(ema_values[i])

            if prev_ticks[i] is not None:
                delta = tick - prev_ticks[i]
                if delta > MAX_DELTA:
                    tick = prev_ticks[i] + MAX_DELTA
                elif delta < -MAX_DELTA:
                    tick = prev_ticks[i] - MAX_DELTA

            # =====================================================
            # [수정] MAX_DELTA 이중 적용 버그 수정
            # 그리퍼(i==6)는 MOTOR_LIMITS 클램프까지 거친 뒤 값이
            # 또 바뀔 수 있어서 여기서 동기화하지 않고,
            # MOTOR_LIMITS 클램프 이후 최종값으로 아래에서 동기화
            # =====================================================
            if i != 6:
                ema_values[i] = float(tick)

            # MOTOR_LIMITS 클램프 (MAX_DELTA 후 강제 범위 적용, 주로 그리퍼용)
            if m in MOTOR_LIMITS:
                lo, hi = MOTOR_LIMITS[m]
                tick = max(lo, min(hi, tick))

            # =====================================================
            # [추가] 히스테리시스 Dead Zone
            # MOTOR_LIMITS 클램프 이후 최종 tick 기준으로 판단
            # =====================================================
            if prev_ticks[i] is not None:
                diff = abs(tick - prev_ticks[i])
                if in_dead_zone[i]:
                    if diff <= DEAD_ZONE_EXIT:
                        continue
                    else:
                        in_dead_zone[i] = False
                else:
                    if diff <= DEAD_ZONE_ENTER:
                        in_dead_zone[i] = True
                        continue

            packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, tick)

            prev_ticks[i] = tick

        # =========================
        # 출력
        # =========================
        print("\033[F", end="")

        raw_str = " ".join([f"{parsed[i]:5d}" for i in range(7)])
        flt_str = " ".join([f"{int(v) if v else 0:5d}" for v in ema_values])

        print(f"RAW:{raw_str} | FLT:{flt_str}")

        time.sleep(0.02)

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
