import serial
import threading
import time
from scservo_sdk import *
from pantilt_safe2 import update_pantilt
from pantilt_safe2 import scs_write_pos
from pantilt_safe2 import pan_pos
from pantilt_safe2 import tilt_pos

# =====================================================
# [공통] FSM 상태 정의
# IDLE  : 해당 팔 전 채널 정지 상태, 모터 명령 안 보냄
# MOVE  : 해당 팔 최소 1채널 이상 움직이는 정상 동작 상태
# ERROR : 이상 감지 시 진입, 해당 팔 모든 모터 정지 및 명령 차단
# 왼팔/오른팔은 서로 독립된 FSM 상태를 가짐
# =====================================================
STATE_IDLE = "IDLE"
STATE_MOVE = "MOVE"
STATE_ERROR = "ERROR"

IDLE_CONFIRM_LOOPS = 10  # 약 0.2초(루프 0.02s 기준), 채터링 방지용 debounce


def check_anomaly(parsed, ema_values, prev_ticks):
    """
    [자리표시자] 1단계 룰 기반 이상탐지 (다음 todo 항목에서 구현)
    - range check: ADC 값이 정상 범위(0~4095) 벗어나는지
    - delta 급변: 한 루프 사이 변화량이 비정상적으로 큰지
    현재는 항상 False 리턴 → ERROR 진입 안 함 (골격만 존재)
    """
    return False


# =========================
# STM32 ADC 시리얼 (양팔 공용, 스레드 1개만 실행)
# =========================
PORT_ADC = "COM13"
BAUD_ADC = 115200

adc_raw = [0] * 22
parsed = [0] * 16  # [0:7]=왼팔, [7:14]=오른팔, [14:16]=IND, sw_toggle 별도

sw_toggle = 0
running = True

FLOATING_THRESHOLD = 4080


def read_serial_adc():
    global adc_raw, parsed, sw_toggle, running

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

            # 오른팔: MUX C9~C15 → parsed[7]~[13]
            for i in range(7):
                parsed[i + 7] = adc_raw[i + 9]

            # IND ADC
            parsed[14] = adc_raw[16]
            parsed[15] = adc_raw[17]

            # SW (디지털)
            sw_toggle = adc_raw[20]

        except Exception as e:
            print("ERR:", e)


# =========================
# 왼팔 STS3215 설정
# =========================
DEVICENAME_LEFT = "COM12"
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
REVERSE_CHANNELS_LEFT = [5]  # 모터 ID 6 반전

EMA_ALPHA_ARM_LEFT = [0.4, 0.4, 0.4, 0.4, 0.4, 0.5]
EMA_ALPHA_GRIPPER_LEFT = 0.5

DEAD_ZONE_ENTER_LEFT = 15
DEAD_ZONE_EXIT_LEFT = 25
MAX_DELTA_LEFT = 70

GRIPPER_ADC_MIN_LEFT = 145
GRIPPER_ADC_MAX_LEFT = 1270
GRIPPER_POS_OPEN_LEFT = 4100
GRIPPER_POS_CLOSE_LEFT = 500

ema_values_left = [None] * 7
prev_ticks_left = [None] * 7
in_dead_zone_left = [False] * 7
current_state_left = STATE_IDLE
idle_confirm_count_left = 0

system_ready_left = False
startup_count_left = 0
STARTUP_WAIT_LEFT = 80

# =========================
# 오른팔 + 팬틸트 STS3215 설정
# =========================
DEVICENAME_RIGHT = "COM14"

MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]
REVERSE_CHANNELS_RIGHT = [0, 3, 4, 5, 6]  # 9, 12, 13, 14, 15번 반전

PAN_ID = 22
TILT_ID = 21

EMA_ALPHA_ARM_RIGHT = [0.45, 0.45, 0.45, 0.45, 0.45, 0.5]
EMA_ALPHA_GRIPPER_RIGHT = 0.5

DEAD_ZONE_ENTER_RIGHT = 12
DEAD_ZONE_EXIT_RIGHT = 20
MAX_DELTA_RIGHT = 70

GRIPPER_ADC_MIN_RIGHT = 2973
GRIPPER_ADC_MAX_RIGHT = 3993
GRIPPER_POS_OPEN_RIGHT = 3935
GRIPPER_POS_CLOSE_RIGHT = 0

ema_values_right = [None] * 7
prev_ticks_right = [None] * 7
in_dead_zone_right = [False] * 7
current_state_right = STATE_IDLE
idle_confirm_count_right = 0

system_ready_right = False
startup_count_right = 0
STARTUP_WAIT_RIGHT = 50


# =========================
# 왼팔 처리 함수
# =========================
def process_left_arm(portHandler, packetHandler):
    global ema_values_left, prev_ticks_left, in_dead_zone_left
    global current_state_left, idle_confirm_count_left

    if check_anomaly(parsed[0:7], ema_values_left, prev_ticks_left):
        current_state_left = STATE_ERROR

    if current_state_left == STATE_ERROR:
        return

    all_idle = all(
        prev_ticks_left[i] is None
        or in_dead_zone_left[i]
        or parsed[i] >= FLOATING_THRESHOLD
        for i in range(7)
    )
    if all_idle:
        idle_confirm_count_left += 1
        if idle_confirm_count_left >= IDLE_CONFIRM_LOOPS:
            current_state_left = STATE_IDLE
    else:
        idle_confirm_count_left = 0
        current_state_left = STATE_MOVE

    for i in range(7):
        m = MOTORS_LEFT[i]
        raw = parsed[i]

        if raw >= FLOATING_THRESHOLD:
            continue

        alpha = EMA_ALPHA_GRIPPER_LEFT if i == 6 else EMA_ALPHA_ARM_LEFT[i]

        if ema_values_left[i] is None:
            ema_values_left[i] = float(raw)
        else:
            ema_values_left[i] = alpha * raw + (1 - alpha) * ema_values_left[i]

        if i == 6:
            adc = int(ema_values_left[6])
            ratio = max(
                0.0,
                min(
                    1.0,
                    (adc - GRIPPER_ADC_MIN_LEFT)
                    / (GRIPPER_ADC_MAX_LEFT - GRIPPER_ADC_MIN_LEFT),
                ),
            )
            tick = int(
                GRIPPER_POS_CLOSE_LEFT
                + ratio * (GRIPPER_POS_OPEN_LEFT - GRIPPER_POS_CLOSE_LEFT)
            )
        else:
            tick = int(ema_values_left[i])
            if i in REVERSE_CHANNELS_LEFT:
                tick = 4095 - tick

        if prev_ticks_left[i] is not None:
            delta = tick - prev_ticks_left[i]
            if delta > MAX_DELTA_LEFT:
                tick = prev_ticks_left[i] + MAX_DELTA_LEFT
            elif delta < -MAX_DELTA_LEFT:
                tick = prev_ticks_left[i] - MAX_DELTA_LEFT

            if i != 6:
                if i in REVERSE_CHANNELS_LEFT:
                    ema_values_left[i] = float(4095 - tick)
                else:
                    ema_values_left[i] = float(tick)

        if prev_ticks_left[i] is not None:
            diff = abs(tick - prev_ticks_left[i])
            if in_dead_zone_left[i]:
                if diff <= DEAD_ZONE_EXIT_LEFT:
                    continue
                else:
                    in_dead_zone_left[i] = False
            else:
                if diff <= DEAD_ZONE_ENTER_LEFT:
                    in_dead_zone_left[i] = True
                    continue

        packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, tick)
        prev_ticks_left[i] = tick


# =========================
# 오른팔 처리 함수
# =========================
def process_right_arm(portHandler, packetHandler):
    global ema_values_right, prev_ticks_right, in_dead_zone_right
    global current_state_right, idle_confirm_count_right

    if check_anomaly(parsed[7:14], ema_values_right, prev_ticks_right):
        current_state_right = STATE_ERROR

    if current_state_right == STATE_ERROR:
        return

    all_idle = all(
        prev_ticks_right[i] is None or in_dead_zone_right[i] for i in range(7)
    )
    if all_idle:
        idle_confirm_count_right += 1
        if idle_confirm_count_right >= IDLE_CONFIRM_LOOPS:
            current_state_right = STATE_IDLE
    else:
        idle_confirm_count_right = 0
        current_state_right = STATE_MOVE

    for i in range(7):
        m = MOTORS_RIGHT[i]
        raw = parsed[i + 7]

        alpha = EMA_ALPHA_GRIPPER_RIGHT if i == 6 else EMA_ALPHA_ARM_RIGHT[i]

        if ema_values_right[i] is None:
            ema_values_right[i] = float(raw)
        else:
            ema_values_right[i] = alpha * raw + (1 - alpha) * ema_values_right[i]

        if i == 6:
            adc = int(ema_values_right[6])
            ratio = max(
                0.0,
                min(
                    1.0,
                    (adc - GRIPPER_ADC_MIN_RIGHT)
                    / (GRIPPER_ADC_MAX_RIGHT - GRIPPER_ADC_MIN_RIGHT),
                ),
            )
            tick = int(
                GRIPPER_POS_CLOSE_RIGHT
                + ratio * (GRIPPER_POS_OPEN_RIGHT - GRIPPER_POS_CLOSE_RIGHT)
            )
        else:
            tick = int(ema_values_right[i])
            if i in REVERSE_CHANNELS_RIGHT:
                tick = 4095 - tick

        if prev_ticks_right[i] is not None:
            delta = tick - prev_ticks_right[i]
            if delta > MAX_DELTA_RIGHT:
                tick = prev_ticks_right[i] + MAX_DELTA_RIGHT
            elif delta < -MAX_DELTA_RIGHT:
                tick = prev_ticks_right[i] - MAX_DELTA_RIGHT

            if i != 6:
                if i in REVERSE_CHANNELS_RIGHT:
                    ema_values_right[i] = float(4095 - tick)
                else:
                    ema_values_right[i] = float(tick)

            diff = abs(tick - prev_ticks_right[i])
            if in_dead_zone_right[i]:
                if diff <= DEAD_ZONE_EXIT_RIGHT:
                    continue
                else:
                    in_dead_zone_right[i] = False
            else:
                if diff <= DEAD_ZONE_ENTER_RIGHT:
                    in_dead_zone_right[i] = True
                    continue

        packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, tick)
        prev_ticks_right[i] = tick


# =========================
# 포트 오픈 (왼팔 / 오른팔 각각)
# =========================
portHandler_left = PortHandler(DEVICENAME_LEFT)
packetHandler_left = PacketHandler(PROTOCOL_END)

portHandler_right = PortHandler(DEVICENAME_RIGHT)
packetHandler_right = PacketHandler(PROTOCOL_END)

if not portHandler_left.openPort():
    print("왼팔 포트 열기 실패")
    quit()
if not portHandler_left.setBaudRate(BAUDRATE):
    print("왼팔 보레이트 실패")
    quit()

if not portHandler_right.openPort():
    print("오른팔 포트 열기 실패")
    quit()
if not portHandler_right.setBaudRate(BAUDRATE):
    print("오른팔 보레이트 실패")
    quit()

# =========================
# 모터 초기화
# =========================
for m in MOTORS_LEFT:
    packetHandler_left.write1ByteTxRx(
        portHandler_left, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
    )
    packetHandler_left.write1ByteTxRx(portHandler_left, m, ADDR_ACCELERATION, 50)

for m in MOTORS_RIGHT:
    packetHandler_right.write1ByteTxRx(
        portHandler_right, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
    )
    packetHandler_right.write1ByteTxRx(portHandler_right, m, ADDR_ACCELERATION, 50)

packetHandler_right.write1ByteTxRx(
    portHandler_right, PAN_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
)
packetHandler_right.write1ByteTxRx(
    portHandler_right, TILT_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
)
packetHandler_right.write1ByteTxRx(portHandler_right, PAN_ID, ADDR_ACCELERATION, 50)
packetHandler_right.write1ByteTxRx(portHandler_right, TILT_ID, ADDR_ACCELERATION, 50)

# =========================
# 팬틸트 중앙 이동
# =========================
scs_write_pos(packetHandler_right, portHandler_right, PAN_ID, 511)
time.sleep(0.5)
scs_write_pos(packetHandler_right, portHandler_right, TILT_ID, 511)
time.sleep(0.5)

# =========================
# ADC thread (양팔 공용, 1개만 실행)
# =========================
t = threading.Thread(target=read_serial_adc)
t.daemon = True
t.start()

print("시작")

# =========================
# 메인 루프
# =========================
try:
    while True:

        # =====================================================
        # 시작 안정화 대기 (양팔 모두 유효 데이터 들어올 때까지)
        # =====================================================
        if not (system_ready_left and system_ready_right):
            if not system_ready_left:
                startup_count_left += 1
                if startup_count_left >= STARTUP_WAIT_LEFT and any(
                    0 < parsed[i] < FLOATING_THRESHOLD for i in range(7)
                ):
                    system_ready_left = True
                    for i in range(7):
                        if parsed[i] < FLOATING_THRESHOLD:
                            ema_values_left[i] = float(parsed[i])
                    print(">>> 왼팔 준비 완료")

            if not system_ready_right:
                startup_count_right += 1
                if startup_count_right >= STARTUP_WAIT_RIGHT and any(
                    parsed[i + 7] > 0 for i in range(7)
                ):
                    system_ready_right = True
                    for i in range(7):
                        ema_values_right[i] = float(parsed[i + 7])
                    print(">>> 오른팔 준비 완료")

            time.sleep(0.02)
            continue

        # =========================
        # 왼팔 / 오른팔 각각 독립 처리
        # =========================
        process_left_arm(portHandler_left, packetHandler_left)
        process_right_arm(portHandler_right, packetHandler_right)

        # =========================
        # 팬틸트 업데이트
        # =========================
        update_pantilt(
            parsed, sw_toggle, packetHandler_right, portHandler_right, PAN_ID, TILT_ID
        )

        # =========================
        # 출력
        # =========================
        print("\033[F", end="")

        left_raw_str = " ".join([f"{parsed[i]:5d}" for i in range(7)])
        right_raw_str = " ".join([f"{parsed[i+7]:5d}" for i in range(7)])

        print(
            f"L:{left_raw_str} | R:{right_raw_str}"
            + f" SW:{sw_toggle}"
            + f" PAN:{pan_pos:4d}"
            + f" TILT:{tilt_pos:4d}"
            + f" L_STATE:{current_state_left}"
            + f" R_STATE:{current_state_right}"
        )

        time.sleep(0.02)

except KeyboardInterrupt:
    pass

# =========================
# 종료
# =========================
running = False

for m in MOTORS_LEFT:
    packetHandler_left.write1ByteTxRx(
        portHandler_left, m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
    )

for m in MOTORS_RIGHT:
    packetHandler_right.write1ByteTxRx(
        portHandler_right, m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
    )

packetHandler_right.write1ByteTxRx(
    portHandler_right, PAN_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
)
packetHandler_right.write1ByteTxRx(
    portHandler_right, TILT_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
)

portHandler_left.closePort()
portHandler_right.closePort()

print("종료")
