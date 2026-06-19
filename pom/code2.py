import os
import sys
import serial
import threading
import numpy as np
import math
import matplotlib.pyplot as plt

# 현재 경로 추가 (라이브러리 로드용)
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

# 펌웨어 제어를 위한 핵심 라이브러리 로드
try:
    from scservo_sdk.port_handler import PortHandler
    from scservo_sdk.group_sync_write import GroupSyncWrite
    # STS3215 전용 클래스를 PacketHandler라는 이름으로 가져옵니다.
    from _scservo_sdk.sms_sts import sms_sts as PacketHandler
    print("✅ STS3215 전용 라이브러리 로드 성공!")
except Exception as e:
    print(f"\n❌ 임포트 실패: {e}")
    sys.exit()

# =========================
# 1. IMU 설정 (BNO055 등)
# =========================
PORT_IMU = "COM11"
BAUD_IMU = 115200

current_quat = np.array([1.0, 0.0, 0.0, 0.0])
Q_ref = np.array([1.0, 0.0, 0.0, 0.0])
running = True

# =========================
# 2. STS3215 모터 설정
# =========================
DEVICENAME = 'COM14'
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

MOTOR1 = 1 # shoulder_pan
MOTOR2 = 2 # shoulder_lift
MOTOR3 = 3 # elbow_flex


# =========================
# 3. 모터 각도 변환 (Tick 연산)
# =========================
MIN_TICK = 500
MAX_TICK = 3500
BASE_TICK = 2024

def deg_to_tick(angle_deg):
    angle_deg = (angle_deg + 180) % 360 - 180
    tick_offset = int(angle_deg * (MAX_TICK - MIN_TICK) / 360)
    tick = BASE_TICK + tick_offset
    return int(np.clip(tick, MIN_TICK, MAX_TICK))

# =========================
# 4. 쿼터니언 수학 연산
# =========================
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

def rotate_vector(q, v):
    qv = np.array([0] + list(v))
    return quat_multiply(quat_multiply(q, qv), quat_conjugate(q))[1:]

# =========================
# 5. IMU 데이터 수집 스레드
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

# =========================
# 6. 하드웨어 연결 및 초기화
# =========================
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(portHandler) # STS3215용 sms_sts는 인자를 받지 않거나 포트와 연동됨

if not portHandler.openPort():
    print("❌ 로봇 팔 포트 열기 실패 (전원과 연결을 확인하세요)")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print("❌ 보레이트 설정 실패")
    quit()

# 모터 토크 및 가속도 설정 (portHandler 인자 제거)
for m in [MOTOR1, MOTOR2, MOTOR3]:
    packetHandler.write1ByteTxRx(m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(m, ADDR_ACCELERATION, 50)

print("✅ 하드웨어 연결 완료 / 'C' 키: 캘리브레이션 / 'ESC': 종료")

def read_motor_positions():
    # portHandler 인자 제거
    pos1, _, _ = packetHandler.read2ByteTxRx(MOTOR1, ADDR_PRESENT_POSITION)
    pos2, _, _ = packetHandler.read2ByteTxRx(MOTOR2, ADDR_PRESENT_POSITION)
    pos3, _, _ = packetHandler.read2ByteTxRx(MOTOR3, ADDR_PRESENT_POSITION)
    return pos1, pos2, pos3

# =========================
# 7. 제어 및 이벤트 처리
# =========================
def on_key(event):
    global Q_ref, running
    if event.key == 'c':
        Q_ref = current_quat.copy()
        for m in [MOTOR1, MOTOR2, MOTOR3]:
            packetHandler.write2ByteTxRx(m, ADDR_GOAL_POSITION, BASE_TICK)
        print("🎯 캘리브레이션 완료: 현재 자세를 기준으로 잡습니다.")
    if event.key == 'escape':
        running = False
        plt.close()

def send_motor_commands():
    global current_quat, Q_ref
    # 상대 자세 계산
    Q_rel = quat_multiply(quat_conjugate(Q_ref), current_quat)
    w, x, y, z = Q_rel
    
    # 회전 행렬 변환
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])
    
    upper_vec = R @ np.array([0, 1, 0])
    x_v, y_v, z_v = upper_vec

    # 관절 각도 추출 (Inverse Kinematics)
    pitch = math.atan2(-x_v, math.sqrt(y_v**2 + z_v**2))
    pitch = math.atan(math.tan(pitch) / 3) # 둔감도 조정
    roll = math.atan2(-z_v, y_v)
    psi = -2 * math.atan2(y, w)

    # Tick 값 변환
    t_pitch = int(BASE_TICK - (math.degrees(pitch) / 180) * (MAX_TICK - MIN_TICK))
    raw_t_roll = int(BASE_TICK - (math.degrees(roll) / 180) * (MAX_TICK - MIN_TICK))
    t_roll = 2048 - (raw_t_roll - 2048) if raw_t_roll > 2048 else raw_t_roll
    t_yaw = deg_to_tick(math.degrees(psi))

    # 모터 명령 전송 (portHandler 인자 제거)
    packetHandler.write2ByteTxRx(MOTOR1, ADDR_GOAL_POSITION, t_pitch)
    packetHandler.write2ByteTxRx(MOTOR2, ADDR_GOAL_POSITION, t_roll)
    packetHandler.write2ByteTxRx(MOTOR3, ADDR_GOAL_POSITION, t_yaw)

    pos1, pos2, pos3 = read_motor_positions()
    print(f"M1:{pos1} | M2:{pos2} | M3:{pos3} | 쿼터니언:[{w:.2f},{x:.2f},{y:.2f},{z:.2f}]", end='\r')

# =========================
# 8. 메인 실행 루프
# =========================
if __name__ == "__main__":
    t = threading.Thread(target=read_serial_imu)
    t.daemon = True
    t.start()

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.ion()

    try:
        while running:
            ax.cla()
            Q_rel = quat_multiply(quat_conjugate(Q_ref), current_quat)
            
            # 시각화 (R, G, B 축 그리기)
            for axis, color in zip([(1,0,0), (0,1,0), (0,0,1)], ['r', 'g', 'b']):
                vec = rotate_vector(Q_rel, axis)
                ax.quiver(0, 0, 0, *vec, color=color, linewidth=2)

            ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([-1, 1])
            ax.set_title("EOD Robot Arm 'SUWAN' - Sensor Orientation")
            
            send_motor_commands()
            plt.draw()
            plt.pause(0.01)
    except KeyboardInterrupt:
        running = False

    # 종료 시 안전하게 토크 해제
    for m in [MOTOR1, MOTOR2, MOTOR3]:
        packetHandler.write1ByteTxRx(m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()
    print("\n시스템이 안전하게 종료되었습니다.")