import serial
import threading
import time
from scservo_sdk import *

# =====================================================
# A0 + 모터 1개 테스트
# 목적: EMA 필터 + Dead Zone이 실제로 떨림을 잡는지 확인
# 안전: 모터 1개만 구동, 나머지는 토크 OFF
# =====================================================

# =========================
# STM32 ADC 시리얼
# =========================
PORT_ADC = "COM13"
BAUD_ADC = 115200

adc = [0] * 10
running = True

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
# 테스트 대상: 모터 1번 (A0 채널)
# =========================
TEST_MOTOR_ID = 1
TEST_ADC_CH = 0

# =========================
# 필터 파라미터 (여기서 조절)
# =========================
EMA_ALPHA = 0.3      # 0.1=매우 부드러움 / 0.3=균형 / 0.5=반응 빠름
DEAD_ZONE = 5        # 이 이하의 변화는 무시
POS_MIN = 0
POS_MAX = 1023

# =========================
# 필터 상태
# =========================
ema_value = None
prev_tick = None

# =========================
# ADC 읽기 스레드
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

            parts = line.split(",")
            for i in range(min(10, len(parts))):
                val_str = ''.join(filter(str.isdigit, parts[i]))
                if val_str != "":
                    adc[i] = int(val_str)

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
# 모터 1번만 토크 ON
# =========================
packetHandler.write1ByteTxRx(
    portHandler,
    TEST_MOTOR_ID,
    ADDR_TORQUE_ENABLE,
    TORQUE_ENABLE
)

packetHandler.write1ByteTxRx(
    portHandler,
    TEST_MOTOR_ID,
    ADDR_ACCELERATION,
    50
)

print("=" * 60)
print(f" A0 + 모터 {TEST_MOTOR_ID} 단독 테스트")
print(f" EMA α={EMA_ALPHA}  Dead Zone={DEAD_ZONE}")
print("=" * 60)
print(" 모터 1번만 토크 ON, 나머지는 OFF 상태")
print(" 가변저항(A0)을 천천히 돌려보세요")
print(" Ctrl+C로 종료")
print("=" * 60)
print("")
print("  RAW   → EMA   → SEND  | 상태")
print("-" * 50)

# =========================
# ADC 스레드 시작
# =========================
t = threading.Thread(target=read_serial_adc)
t.daemon = True
t.start()

# =========================
# 시작 안정화 대기
# =========================
print("ADC 안정화 대기 중...")
time.sleep(1.5)

# ADC 첫 값이 들어올 때까지 대기
for _ in range(100):
    if adc[TEST_ADC_CH] > 0:
        break
    time.sleep(0.02)

# EMA 초기값 설정
raw = max(POS_MIN, min(POS_MAX, adc[TEST_ADC_CH]))
ema_value = float(raw)
prev_tick = int(ema_value)

# 현재 위치로 모터를 먼저 이동 (급발진 방지)
packetHandler.write2ByteTxRx(
    portHandler,
    TEST_MOTOR_ID,
    ADDR_GOAL_POSITION,
    prev_tick
)

print(f"초기 위치: {prev_tick} → 모터 제어 시작!")
print("")

# =========================
# 통계 카운터
# =========================
total_loops = 0
sent_count = 0
skipped_count = 0

# =========================
# 메인 루프
# =========================
try:
    while True:
        total_loops += 1

        # [1] 원본 값 읽기 + 클램프
        raw = adc[TEST_ADC_CH]
        raw = max(POS_MIN, min(POS_MAX, raw))

        # [2] EMA 필터
        ema_value = EMA_ALPHA * raw + (1 - EMA_ALPHA) * ema_value
        filtered = int(ema_value)

        # [3] Dead Zone 체크
        if abs(filtered - prev_tick) <= DEAD_ZONE:
            # 변화가 너무 작으면 무시 (떨림 방지)
            skipped_count += 1
            status = "SKIP (Dead Zone)"
        else:
            # 모터에 전송
            packetHandler.write2ByteTxRx(
                portHandler,
                TEST_MOTOR_ID,
                ADDR_GOAL_POSITION,
                filtered
            )
            prev_tick = filtered
            sent_count += 1
            status = ">>> SENT"

        # [4] 화면 출력
        print(f"\033[F {raw:5d} → {filtered:5d} → {prev_tick:5d} | {status:<20s}"
              + f" (전송:{sent_count} 생략:{skipped_count})")

        time.sleep(0.02)

except KeyboardInterrupt:
    pass

# =========================
# 종료
# =========================
running = False

print("")
print("=" * 50)
print(f" 총 루프: {total_loops}")
print(f" 전송 횟수: {sent_count}")
print(f" 생략 횟수: {skipped_count}")
if total_loops > 0:
    skip_rate = skipped_count / total_loops * 100
    print(f" 생략률: {skip_rate:.1f}% (높을수록 떨림 적음)")
print("=" * 50)

# 모터 토크 OFF
packetHandler.write1ByteTxRx(
    portHandler,
    TEST_MOTOR_ID,
    ADDR_TORQUE_ENABLE,
    TORQUE_DISABLE
)

portHandler.closePort()
print("종료")