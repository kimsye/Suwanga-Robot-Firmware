import os
import sys
import serial
import threading
import numpy as np
import math
import matplotlib.pyplot as plt

# 현재 경로 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.append(current_dir)

# 펌웨어 라이브러리 로드
try:
    from scservo_sdk.port_handler import PortHandler
    from scservo_sdk.group_sync_write import GroupSyncWrite
    from _scservo_sdk.sms_sts import sms_sts as PacketHandler
    print("✅ STS3215 전용 라이브러리 로드 성공!")
except Exception as e:
    print(f"\n❌ 임포트 실패: {e}")
    sys.exit()

# =========================
# 1. IMU 및 로봇 설정
# =========================
PORT_IMU = "COM11"
BAUD_IMU = 115200
DEVICENAME = 'COM14'
BAUDRATE = 1000000

current_quat = np.array([1.0, 0.0, 0.0, 0.0])
Q_ref = np.array([1.0, 0.0, 0.0, 0.0])
running = True

MOTOR1, MOTOR2, MOTOR3 = 1, 2, 3
ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
TORQUE_ENABLE, TORQUE_DISABLE = 1, 0

MIN_TICK, MAX_TICK, BASE_TICK = 500, 3500, 2024

# ==========================================
# [신규 추가] 부드러운 움직임을 위한 글로벌 변수
# ==========================================
prev_t_pitch = BASE_TICK
prev_t_roll = BASE_TICK
prev_t_yaw = BASE_TICK

# ALPHA가 낮을수록 로봇이 천천히, 묵직하게 움직입니다 (0.1 추천)
ALPHA = 0.1 

# =========================
# 2. 수학 및 통신 함수
# =========================
def deg_to_tick(angle_deg):
    angle_deg = (angle_deg + 180) % 360 - 180
    tick_offset = int(angle_deg * (MAX_TICK - MIN_TICK) / 360)
    tick = BASE_TICK + tick_offset
    return int(np.clip(tick, MIN_TICK, MAX_TICK))

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

def read_serial_imu():
    global current_quat, running
    try:
        ser = serial.Serial(PORT_IMU, BAUD_IMU, timeout=1)
        while running:
            line = ser.readline().decode().strip()
            if line.startswith("$") and len(line.split(",")) == 4:
                data = line[1:].split(",")
                current_quat = np.array([float(v) for v in data])
    except: pass

# =========================
# 3. 하드웨어 연결 및 초기화
# =========================
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(portHandler)

if not portHandler.openPort() or not portHandler.setBaudRate(BAUDRATE):
    print("❌ 하드웨어 연결 실패!")
    quit()

# [수정] 가속도를 20으로 낮춰 부드러운 출발 유도
for m in [MOTOR1, MOTOR2, MOTOR3]:
    packetHandler.write1ByteTxRx(m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(m, ADDR_ACCELERATION, 20) 

print("✅ '수완' 연결 완료 / 'C': 캘리브레이션 / 'ESC': 종료")

# ==========================================
# 4. [교체] 부드러운 명령 전송 함수 (EMA + Deadzone)
# ==========================================
def send_motor_commands():
    global current_quat, Q_ref, prev_t_pitch, prev_t_roll, prev_t_yaw

    # 1. 쿼터니언 기반 상대 자세 계산
    Q_rel = quat_multiply(quat_conjugate(Q_ref), current_quat)
    w, x, y, z = Q_rel
    
    R = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])
    
    upper_vec = R @ np.array([0, 1, 0])
    x_v, y_v, z_v = upper_vec

    # 2. 목표 각도 계산
    pitch = math.atan2(-x_v, math.sqrt(y_v**2 + z_v**2))
    roll = math.atan2(-z_v, y_v)
    psi = -2 * math.atan2(y, w)

    raw_t_pitch = int(BASE_TICK - (math.degrees(pitch) / 180) * (MAX_TICK - MIN_TICK))
    temp_roll = int(BASE_TICK - (math.degrees(roll) / 180) * (MAX_TICK - MIN_TICK))
    raw_t_roll = 2048 - (temp_roll - 2048) if temp_roll > 2048 else temp_roll
    raw_t_yaw = deg_to_tick(math.degrees(psi))

    # 3. [핵심] EMA 필터 적용 (천천히 움직이게 함)
    smooth_t_pitch = int(ALPHA * raw_t_pitch + (1 - ALPHA) * prev_t_pitch)
    smooth_t_roll = int(ALPHA * raw_t_roll + (1 - ALPHA) * prev_t_roll)
    smooth_t_yaw = int(ALPHA * raw_t_yaw + (1 - ALPHA) * prev_t_yaw)

    # 4. [핵심] 데드존 설정 (미세 떨림 방지)
    if abs(smooth_t_pitch - prev_t_pitch) > 5:
        packetHandler.write2ByteTxRx(MOTOR1, ADDR_GOAL_POSITION, smooth_t_pitch)
        prev_t_pitch = smooth_t_pitch

    if abs(smooth_t_roll - prev_t_roll) > 5:
        packetHandler.write2ByteTxRx(MOTOR2, ADDR_GOAL_POSITION, smooth_t_roll)
        prev_t_roll = smooth_t_roll

    if abs(smooth_t_yaw - prev_t_yaw) > 5:
        packetHandler.write2ByteTxRx(MOTOR3, ADDR_GOAL_POSITION, smooth_t_yaw)
        prev_t_yaw = smooth_t_yaw

# =========================
# 5. 메인 루프 및 이벤트
# =========================
def on_key(event):
    global Q_ref, running
    if event.key == 'c':
        Q_ref = current_quat.copy()
        for m in [MOTOR1, MOTOR2, MOTOR3]:
            packetHandler.write2ByteTxRx(m, ADDR_GOAL_POSITION, BASE_TICK)
        print("🎯 기준 자세 설정 완료")
    if event.key == 'escape':
        running = False
        plt.close()

if __name__ == "__main__":
    t = threading.Thread(target=read_serial_imu); t.daemon = True; t.start()
    fig = plt.figure(); ax = fig.add_subplot(111, projection='3d')
    fig.canvas.mpl_connect('key_press_event', on_key); plt.ion()

    try:
        while running:
            ax.cla()
            Q_rel = quat_multiply(quat_conjugate(Q_ref), current_quat)
            for axis, col in zip([(1,0,0), (0,1,0), (0,0,1)], ['r', 'g', 'b']):
                v = rotate_vector(Q_rel, axis)
                ax.quiver(0, 0, 0, *v, color=col, linewidth=2)
            ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([-1, 1])
            
            send_motor_commands()
            plt.draw(); plt.pause(0.01)
    except: running = False

    for m in [MOTOR1, MOTOR2, MOTOR3]:
        packetHandler.write1ByteTxRx(m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()
    print("시스템 종료")
    