import serial
import threading
import numpy as np
import math
import rerun as rr  # 유니티 대신 사용할 고성능 시뮬레이터
from scservo_sdk import *

# =========================
# 1. 서현님 환경 및 IMU 설정
# =========================
PORT_IMU = "COM14"  # 서현님의 포트 번호 반영
BAUD_IMU = 115200

current_quat = np.array([1.0, 0.0, 0.0, 0.0])
Q_ref = np.array([1.0, 0.0, 0.0, 0.0])
running = True

# =========================
# 2. STS3215 모터 설정
# =========================
DEVICENAME = 'COM14' # 서현님의 모터 포트 반영
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# 서현님의 MOTORS_CONFIG 매핑 반영
MOTOR1 = 1  # shoulder_pan
MOTOR2 = 2  # shoulder_lift
MOTOR3 = 3  # elbow_flex

# 캘리브레이션 파일 경로 (참조용)
CALIB_PATH = r"C:\Users\kim nh\.cache\huggingface\lerobot\calibration\robots\so_follower\seohyun_follower.json"

# =========================
# 3. 유틸리티 함수 (연산 및 변환)
# =========================
MIN_TICK, MAX_TICK, BASE_TICK = 500, 3500, 2024

def deg_to_tick(angle_deg):
    angle_deg = (angle_deg + 180) % 360 - 180
    tick_offset = int(angle_deg * (MAX_TICK - MIN_TICK) / 360)
    return int(np.clip(BASE_TICK + tick_offset, MIN_TICK, MAX_TICK))

def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

# =========================
# 4. 시리얼 및 모터 제어 로직
# =========================
def read_serial_imu():
    global current_quat, running
    try:
        ser = serial.Serial(PORT_IMU, BAUD_IMU, timeout=1)
        while running:
            line = ser.readline().decode().strip()
            if line.startswith("$"):
                data = line[1:].split(",")
                if len(data) == 4:
                    current_quat = np.array([float(v) for v in data])
    except Exception as e:
        print(f"IMU 연결 오류: {e}")

# STS3215 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_END)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("모터 포트 설정 실패. 포트 번호와 전원을 확인하세요.")
    # quit() # 시뮬레이션만 테스트하려면 주석 처리 가능

def send_motor_commands():
    global current_quat, Q_ref
    
    # 1. 상대 쿼터니언 계산: $$Q_{rel} = Q_{ref}^{-1} \otimes Q_{curr}$$
    Q_rel = quat_multiply(quat_conjugate(Q_ref), current_quat)
    w, x, y, z = Q_rel

    # 2. 회전 행렬 변환 및 벡터 추출
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])
    upper_vec = R @ np.array([0, 1, 0])
    x_v, y_v, z_v = upper_vec

    # 3. 각도 계산 (Inverse Kinematics 기초)
    pitch = math.atan2(-x_v, math.sqrt(y_v**2 + z_v**2))
    roll = math.atan2(-z_v, y_v)
    psi = -2 * math.atan2(y, w)

    # 4. 모터 명령 전송 (STS3215 Tick)
    tick_p = int(BASE_TICK - (math.degrees(pitch) / 180) * (MAX_TICK - MIN_TICK))
    tick_r = BASE_TICK - (int(BASE_TICK - (math.degrees(roll) / 180) * (MAX_TICK - MIN_TICK)) - BASE_TICK)
    tick_y = deg_to_tick(math.degrees(psi))

    for m, t in zip([MOTOR1, MOTOR2, MOTOR3], [tick_p, tick_r, tick_y]):
        packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, t)

    # 5. Rerun 시각화 로그 (유니티 대체)
    # 쿼터니언 순서 주의: Rerun은 [x, y, z, w] 형식을 사용함
    rr.log("world/arm_model", rr.Boxes3D(half_sizes=[[0.05, 0.2, 0.05]], colors=[0, 255, 0]))
    rr.log("world/arm_model", rr.Transform3D(rotation=rr.Quaternion(xyzw=[x, y, z, w])))

# =========================
# 5. 메인 실행부
# =========================
if __name__ == "__main__":
    # Rerun 초기화
    rr.init("Seohyun_EOD_Robot_Sim", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    t = threading.Thread(target=read_serial_imu)
    t.daemon = True
    t.start()

    print("시스템 시작 / 'C' 키는 지원되지 않으므로 캘리브레이션은 코드로 관리 / ESC로 종료")
    
    try:
        while running:
            send_motor_commands()
            # 50Hz 제어 (0.02초 간격)
    except KeyboardInterrupt:
        running = False

    # 종료 시 토크 해제
    for m in [MOTOR1, MOTOR2, MOTOR3]:
        packetHandler.write1ByteTxRx(portHandler, m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()
    print("종료 완료")