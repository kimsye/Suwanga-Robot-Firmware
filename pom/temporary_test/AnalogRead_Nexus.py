import serial
import threading
import time

# =====================================================
# [테스트 전용 파일] 서보 없이 가변저항 + STM32만으로
# Stage1/2 이상탐지 + 가속도 제한 + 통신 타임아웃 + CRC/시퀀스 자체테스트 검증
# =====================================================

STATE_IDLE = "IDLE"
STATE_MOVE = "MOVE"
STATE_ERROR = "ERROR"

IDLE_CONFIRM_LOOPS = 10

# =====================================================
# Stage 1/2 이상탐지 파라미터
# =====================================================
ADC_MIN_VALID = 0
ADC_MAX_VALID = 4095

MAX_RAW_DELTA = 1500
EXTREME_LOW = 30
EXTREME_HIGH = 4065
ANOMALY_CONFIRM_COUNT = 3

prev_raw_left = [None] * 7
prev_raw_right = [None] * 7
anomaly_count_left = [0] * 7
anomaly_count_right = [0] * 7


def calc_crc16_ccitt(data: bytes, initial=0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def crc16_self_test():
    test_data = b"123456789"
    result = calc_crc16_ccitt(test_data)
    expected = 0x29B1
    status = "PASS" if result == expected else "FAIL"
    print(
        f">>> [CRC16 자체테스트] 계산값=0x{result:04X} 기대값=0x{expected:04X} → {status}"
    )


class SequenceValidator:
    def __init__(self, max_seq=65535):
        self.last_seq = None
        self.max_seq = max_seq
        self.lost_count = 0
        self.duplicate_count = 0
        self.reorder_count = 0

    def check(self, seq):
        if self.last_seq is None:
            self.last_seq = seq
            return "OK"

        expected = (self.last_seq + 1) % (self.max_seq + 1)

        if seq == expected:
            result = "OK"
        elif seq == self.last_seq:
            result = "DUPLICATE"
            self.duplicate_count += 1
        elif self._is_ahead(seq, expected):
            result = "LOST"
            self.lost_count += 1
        else:
            result = "REORDER"
            self.reorder_count += 1

        # [수정] 실제로 전진한 경우(OK, LOST)에만 last_seq 갱신.
        # DUPLICATE/REORDER는 "이미 지나간 옛 패킷이 늦게 도착한 것"이라
        # 최신 위치 기준점을 되돌리면 안 됨 (그러면 다음 패킷까지 오판됨)
        if result in ("OK", "LOST"):
            self.last_seq = seq

        return result

    def _is_ahead(self, seq, expected):
        diff = (seq - expected) % (self.max_seq + 1)
        return diff < (self.max_seq // 2)


def sequence_validator_self_test():
    print(">>> [시퀀스 검증 자체테스트]")
    cases = [
        ("정상", [1, 2, 3, 4, 5], ["OK", "OK", "OK", "OK", "OK"]),
        ("유실", [1, 2, 4, 5], ["OK", "OK", "LOST", "OK"]),
        ("중복", [1, 2, 2, 3], ["OK", "OK", "DUPLICATE", "OK"]),
        ("뒤바뀜", [1, 3, 2, 4], ["OK", "LOST", "REORDER", "OK"]),
    ]
    for name, seq_list, expected_list in cases:
        v = SequenceValidator()
        results = [v.check(s) for s in seq_list]
        status = "PASS" if results == expected_list else "FAIL"
        print(
            f"    {name}: seq={seq_list} → {results} (기대:{expected_list}) → {status}"
        )


def check_anomaly(channel_parsed, prev_raw, anomaly_count, arm_name):
    """
    Stage 1: 절대범위(0~4095) 이탈 → 즉시 이상 판정
    Stage 2: 변화율 급변 또는 극단값 "왕복"(단선 패턴) → 채널별 연속 카운트,
            ANOMALY_CONFIRM_COUNT(3회) 연속되면 True(ERROR) 리턴
    [수정] 극단값 안에서의 미세한 흔들림(정상 가동범위 끝)은 오탐 방지를 위해 제외.
    LOW↔HIGH 사이를 실제로 왕복하는 경우만 단선 패턴으로 판정.
    """
    error_triggered = False

    for i in range(7):
        raw = channel_parsed[i]

        # ---------- Stage 1: 절대범위 검증 ----------
        if raw < ADC_MIN_VALID or raw > ADC_MAX_VALID:
            print(f">>> [Stage1] {arm_name} 채널{i} 범위 이탈 raw={raw}")
            error_triggered = True
            continue

        # ---------- Stage 2: 변화율 + 단선 패턴(왕복) 검증 ----------
        is_anomaly_frame = False

        if prev_raw[i] is not None:
            delta = abs(raw - prev_raw[i])

            if delta > MAX_RAW_DELTA:
                is_anomaly_frame = True

            # [수정] "극단값 안에서 미세하게 다름"이 아니라
            # "LOW 쪽에 있다가 HIGH 쪽으로, 또는 그 반대로 튀는지"만 판정
            prev_near_low = prev_raw[i] <= EXTREME_LOW
            curr_near_low = raw <= EXTREME_LOW
            prev_near_high = prev_raw[i] >= EXTREME_HIGH
            curr_near_high = raw >= EXTREME_HIGH

            if (prev_near_low and curr_near_high) or (prev_near_high and curr_near_low):
                is_anomaly_frame = True

        if is_anomaly_frame:
            anomaly_count[i] += 1
            if anomaly_count[i] >= ANOMALY_CONFIRM_COUNT:
                print(
                    f">>> [Stage2] {arm_name} 채널{i} 연속 이상 {anomaly_count[i]}회 감지"
                )
                error_triggered = True
        else:
            anomaly_count[i] = 0

        prev_raw[i] = raw

    return error_triggered


# =========================
# STM32 ADC 시리얼
# =========================
PORT_ADC = "COM13"
BAUD_ADC = 115200

adc_raw = [0] * 22
parsed = [0] * 16

sw_toggle = 0
running = True

FLOATING_THRESHOLD = 4080

SERIAL_TIMEOUT_SEC = 0.5
last_serial_rx_time = time.time()


def read_serial_adc():
    global adc_raw, parsed, sw_toggle, running, last_serial_rx_time

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

            for i in range(7):
                parsed[i] = adc_raw[i + 1]

            for i in range(7):
                parsed[i + 7] = adc_raw[i + 9]

            parsed[14] = adc_raw[16]
            parsed[15] = adc_raw[17]

            sw_toggle = adc_raw[20]

            last_serial_rx_time = time.time()

        except Exception as e:
            print("ERR:", e)


# =========================
# 왼팔 파라미터
# =========================
MOTORS_LEFT = [1, 2, 3, 4, 5, 6, 7]
REVERSE_CHANNELS_LEFT = [5]

EMA_ALPHA_ARM_LEFT = [0.4, 0.4, 0.4, 0.4, 0.4, 0.5]
EMA_ALPHA_GRIPPER_LEFT = 0.5

DEAD_ZONE_ENTER_LEFT = 15
DEAD_ZONE_EXIT_LEFT = 25
MAX_DELTA_LEFT = 70
MAX_ACCEL_LEFT = 15

GRIPPER_ADC_MIN_LEFT = 145
GRIPPER_ADC_MAX_LEFT = 1270
GRIPPER_POS_OPEN_LEFT = 4100
GRIPPER_POS_CLOSE_LEFT = 500

ema_values_left = [None] * 7
prev_ticks_left = [None] * 7
prev_delta_left = [0] * 7
in_dead_zone_left = [False] * 7
current_state_left = STATE_IDLE
prev_state_left = STATE_IDLE
idle_confirm_count_left = 0

system_ready_left = False
startup_count_left = 0
STARTUP_WAIT_LEFT = 80

# =========================
# 오른팔 파라미터
# =========================
MOTORS_RIGHT = [9, 10, 11, 12, 13, 14, 15]
REVERSE_CHANNELS_RIGHT = [0, 3, 4, 5, 6]

EMA_ALPHA_ARM_RIGHT = [0.45, 0.45, 0.45, 0.45, 0.45, 0.5]
EMA_ALPHA_GRIPPER_RIGHT = 0.5

DEAD_ZONE_ENTER_RIGHT = 12
DEAD_ZONE_EXIT_RIGHT = 20
MAX_DELTA_RIGHT = 70
MAX_ACCEL_RIGHT = 15

GRIPPER_ADC_MIN_RIGHT = 2973
GRIPPER_ADC_MAX_RIGHT = 3993
GRIPPER_POS_OPEN_RIGHT = 3935
GRIPPER_POS_CLOSE_RIGHT = 0

ema_values_right = [None] * 7
prev_ticks_right = [None] * 7
prev_delta_right = [0] * 7
in_dead_zone_right = [False] * 7
current_state_right = STATE_IDLE
prev_state_right = STATE_IDLE
idle_confirm_count_right = 0

system_ready_right = False
startup_count_right = 0
STARTUP_WAIT_RIGHT = 50


def disable_left_torque():
    print(">>> ERROR: 왼팔 전 모터 토크 OFF (테스트 모드: 서보 없음)")


def process_left_arm():
    global ema_values_left, prev_ticks_left, in_dead_zone_left
    global current_state_left, idle_confirm_count_left, prev_state_left

    if check_anomaly(parsed[0:7], prev_raw_left, anomaly_count_left, "왼팔"):
        current_state_left = STATE_ERROR

    if current_state_left == STATE_ERROR:
        if prev_state_left != STATE_ERROR:
            disable_left_torque()
        prev_state_left = current_state_left
        return

    prev_state_left = current_state_left

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

            actual_delta = tick - prev_ticks_left[i]
            accel = actual_delta - prev_delta_left[i]
            if accel > MAX_ACCEL_LEFT:
                actual_delta = prev_delta_left[i] + MAX_ACCEL_LEFT
                tick = prev_ticks_left[i] + actual_delta
            elif accel < -MAX_ACCEL_LEFT:
                actual_delta = prev_delta_left[i] - MAX_ACCEL_LEFT
                tick = prev_ticks_left[i] + actual_delta
            prev_delta_left[i] = actual_delta

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

        # [서보 write 없음 — 테스트 전용]
        prev_ticks_left[i] = tick


def disable_right_torque():
    print(">>> ERROR: 오른팔 전 모터 토크 OFF (테스트 모드: 서보 없음)")


def process_right_arm():
    global ema_values_right, prev_ticks_right, in_dead_zone_right
    global current_state_right, idle_confirm_count_right, prev_state_right

    if check_anomaly(parsed[7:14], prev_raw_right, anomaly_count_right, "오른팔"):
        current_state_right = STATE_ERROR

    if current_state_right == STATE_ERROR:
        if prev_state_right != STATE_ERROR:
            disable_right_torque()
        prev_state_right = current_state_right
        return

    prev_state_right = current_state_right

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

            # [테스트 파일에서 추가] 오른팔 가속도 제한 (원본에서 누락돼 있던 부분)
            actual_delta = tick - prev_ticks_right[i]
            accel = actual_delta - prev_delta_right[i]
            if accel > MAX_ACCEL_RIGHT:
                actual_delta = prev_delta_right[i] + MAX_ACCEL_RIGHT
                tick = prev_ticks_right[i] + actual_delta
            elif accel < -MAX_ACCEL_RIGHT:
                actual_delta = prev_delta_right[i] - MAX_ACCEL_RIGHT
                tick = prev_ticks_right[i] + actual_delta
            prev_delta_right[i] = actual_delta

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

        # [서보 write 없음 — 테스트 전용]
        prev_ticks_right[i] = tick


t = threading.Thread(target=read_serial_adc)
t.daemon = True
t.start()

crc16_self_test()
sequence_validator_self_test()

print("시작 (서보 없음 — Stage1/2 + 가속도 제한 + 통신 타임아웃 전용 테스트)")

try:
    while True:

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

        if time.time() - last_serial_rx_time > SERIAL_TIMEOUT_SEC:
            if current_state_left != STATE_ERROR or current_state_right != STATE_ERROR:
                print(
                    f">>> [통신오류] {SERIAL_TIMEOUT_SEC}초 이상 ADC 데이터 없음 — 양팔 ERROR 전환"
                )
            current_state_left = STATE_ERROR
            current_state_right = STATE_ERROR

        process_left_arm()
        process_right_arm()

        left_raw_str = " ".join([f"{parsed[i]:5d}" for i in range(7)])
        right_raw_str = " ".join([f"{parsed[i+7]:5d}" for i in range(7)])

        print(
            f"L:{left_raw_str} | R:{right_raw_str}"
            + f" SW:{sw_toggle}"
            + f" L_STATE:{current_state_left}(prev:{prev_state_left}, cnt:{anomaly_count_left})"
            + f" R_STATE:{current_state_right}(prev:{prev_state_right}, cnt:{anomaly_count_right})"
        )

        time.sleep(0.02)

except KeyboardInterrupt:
    pass

running = False
time.sleep(0.1)  # [추가] ADC 스레드가 루프를 빠져나올 시간 확보

print("종료")
