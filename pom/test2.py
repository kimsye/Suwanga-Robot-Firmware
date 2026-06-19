import serial
import threading
import numpy as np
import math
from scservo_sdk import *
import matplotlib.pyplot as plt

# =========================
# IMU 설정
# =========================
PORT_IMU = "COM11"
BAUD_IMU = 115200

current_quat = np.array([1.0, 0.0, 0.0, 0.0])
Q_ref = np.array([1.0, 0.0, 0.0, 0.0])
running = True

# =========================
# STS3215 설정
# =========================
DEVICENAME = 'COM10'
BAUDRATE = 1000000
PROTOCOL_END = 0

ADDR_TORQUE_ENABLE = 40
ADDR_ACCELERATION = 41
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

MOTOR1 = 1
MOTOR2 = 2
MOTOR3 = 3








# =========================
# 모터 각도 변환
# =========================
MIN_TICK = 500
MAX_TICK = 3500
BASE_TICK = 2024

def deg_to_tick(angle_deg):
    if angle_deg > 180:
        angle_deg -= 360
    if angle_deg < -180:
        angle_deg += 360

    tick_offset = int(angle_deg * (MAX_TICK - MIN_TICK) / 360)
    tick = BASE_TICK + tick_offset
    return int(np.clip(tick, MIN_TICK, MAX_TICK))


# =========================
# 쿼터니언 연산
# =========================
def quat_multiply(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
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
# IMU 시리얼 스레드
# =========================
def read_serial_imu():
    global current_quat, running
    ser = serial.Serial(PORT_IMU, BAUD_IMU, timeout=1)

    while running:
        try:
            line = ser.readline().decode().strip()
            if line.startswith("$"):
                data = line[1:].split(",")
                if len(data) == 4:
                    current_quat = np.array([float(v) for v in data])
        except:
            pass

# =========================
# STS3215 연결
# =========================
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_END)

if not portHandler.openPort():
    print("포트 열기 실패")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print("보레이트 설정 실패")
    quit()

for m in [MOTOR1, MOTOR2, MOTOR3]:
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_ACCELERATION, 50)

print("연결 완료 / C=캘리 / ESC=종료")

# =========================
# 현재 모터 위치
# =========================
def read_motor_positions():
    pos1, _, _ = packetHandler.read2ByteTxRx(portHandler, MOTOR1, ADDR_PRESENT_POSITION)
    pos2, _, _ = packetHandler.read2ByteTxRx(portHandler, MOTOR2, ADDR_PRESENT_POSITION)
    pos3, _, _ = packetHandler.read2ByteTxRx(portHandler, MOTOR3, ADDR_PRESENT_POSITION)
    return pos1, pos2, pos3

# =========================
# 키 이벤트
# =========================
def on_key(event):
    global Q_ref, running

    if event.key == 'c':
        Q_ref = current_quat.copy()
        for m in [MOTOR1, MOTOR2, MOTOR3]:
            packetHandler.write2ByteTxRx(portHandler, m, ADDR_GOAL_POSITION, BASE_TICK)
        print("캘리브레이션 완료")

    if event.key == 'escape':
        running = False
        plt.close()












def send_motor_commands():
    global current_quat, Q_ref

    # 1. 센서 기준 상대 쿼터니언
    Q_rel = quat_multiply(quat_conjugate(Q_ref), current_quat)

    # 2. 회전 행렬로 변환
    w, x, y, z = Q_rel
    R = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x**2 + z**2),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])
 
    # 기존 upper_vec = R @ [0,1,0]  # y축 벡터
    # 센서 기준 상완 벡터
    upper_vec = R @ np.array([0,1,0])
    x_vec, y_vec, z_vec = upper_vec

    # pitch 계산: y_vec 대신 projection on x-y plane
    # yaw 영향을 제거하려면 y_vec 대신 sqrt(x^2 + y^2) 등 단순화
    pitch = math.atan2(-x_vec, math.sqrt(y_vec**2 + z_vec**2))  # MOTOR1
    pitch = math.atan(math.tan(pitch) / 3)  # 3배 둔감

    # roll 계산: 좌/우는 z_vec 기준 그대로
    roll = math.atan2(-z_vec, y_vec)  # MOTOR2

    # yaw 계산은 기존 psi
    psi = -2 * math.atan2(y, w)

    # 6. degree → tick 변환 (모터 1,2 동작 반전)
    tick_pitch = int(BASE_TICK - (math.degrees(pitch) / 180) * (MAX_TICK - MIN_TICK))  # MOTOR1 반전

    # MOTOR2 거울 반전: 2048 기준
    raw_tick_roll = int(BASE_TICK - (math.degrees(roll) / 180) * (MAX_TICK - MIN_TICK))
    if raw_tick_roll > 2048:
        tick_roll = 2048 - (raw_tick_roll - 2048)
    else:
        tick_roll = raw_tick_roll

    tick_yaw = deg_to_tick(math.degrees(psi))  # MOTOR3

    # 7. 모터 전송
    packetHandler.write2ByteTxRx(portHandler, MOTOR1, ADDR_GOAL_POSITION, tick_pitch)
    packetHandler.write2ByteTxRx(portHandler, MOTOR2, ADDR_GOAL_POSITION, tick_roll)
    packetHandler.write2ByteTxRx(portHandler, MOTOR3, ADDR_GOAL_POSITION, tick_yaw)

    pos1, pos2, pos3 = read_motor_positions()
    print(f"V:[{x_vec:.2f},{y_vec:.2f},{z_vec:.2f}] | Pitch(M1):{pos1} ({math.degrees(pitch):.1f}°) | Roll(M2):{pos2} ({math.degrees(roll):.1f}°) | Yaw(M3):{pos3} ({math.degrees(psi):.1f}°)")







# =========================
# 메인 루프
# =========================
if __name__ == "__main__":

    t = threading.Thread(target=read_serial_imu)
    t.daemon = True
    t.start()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.ion()

    while running:
        ax.cla()

        Q_rel = quat_multiply(quat_conjugate(Q_ref), current_quat)

        x_axis = rotate_vector(Q_rel, (1,0,0))
        y_axis = rotate_vector(Q_rel, (0,1,0))
        z_axis = rotate_vector(Q_rel, (0,0,1))

        ax.quiver(0,0,0,*x_axis,color='r',linewidth=3)
        ax.quiver(0,0,0,*y_axis,color='g',linewidth=3)
        ax.quiver(0,0,0,*z_axis,color='b',linewidth=3)

        ax.set_xlim([-1,1])
        ax.set_ylim([-1,1])
        ax.set_zlim([-1,1])

        send_motor_commands()

        plt.draw()
        plt.pause(0.02)

# =========================
# 종료
# =========================
for m in [MOTOR1, MOTOR2, MOTOR3]:
    packetHandler.write1ByteTxRx(portHandler, m, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

portHandler.closePort()
print("종료 완료")






