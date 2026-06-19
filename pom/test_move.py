import json
import time
# LeRobot 라이브러리에서 필요한 모듈 로드
from lerobot.common.robot_devices.motors.feetech import FeetechMotorsBus
from lerobot.common.robot_devices.motors.configs import MotorControlConfig, MotorCalibration

# ==== 1. 서현님의 환경 설정 ====
PORT = "COM14"  # 서현님이 찾은 포트 번호
USE_DEGREES = False
# 캘리브레이션 파일 경로 (사용자 계정명이 'kim nh'인 경우)
CALIB_PATH = r"C:\Users\kim nh\.cache\huggingface\lerobot\calibration\robots\so_follower\seohyun_follower.json"

# ==== 2. 모터 및 ID 매핑 설정 ====
MOTORS_CONFIG = {
    "shoulder_pan": MotorControlConfig(1, "sts3215"),
    "shoulder_lift": MotorControlConfig(2, "sts3215"),
    "elbow_flex": MotorControlConfig(3, "sts3215"),
    "wrist_flex": MotorControlConfig(4, "sts3215"),
    "wrist_roll": MotorControlConfig(5, "sts3215"),
    "gripper": MotorControlConfig(6, "sts3215"),
}

ID_TO_NAME = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}

# ==== 3. Calibration 데이터 로드 ====
try:
    with open(CALIB_PATH, "r") as f:
        calib_dict = json.load(f)
except FileNotFoundError:
    print(f"에러: {CALIB_PATH} 파일을 찾을 수 없습니다. 경로를 다시 확인해주세요.")
    exit()

calibration = {}
for motor_name, data in calib_dict.items():
    calibration[motor_name] = MotorCalibration(
        id=data["id"],
        drive_mode=data.get("drive_mode", 0),
        homing_offset=data.get("homing_offset", 0),
        range_min=data.get("range_min", 0),
        range_max=data.get("range_max", 4095),
    )

# ==== 4. 모터 버스 연결 ====
bus = FeetechMotorsBus(port=PORT, motors=MOTORS_CONFIG, calibration=calibration)

print(f"포트 {PORT}에 연결된 모터 버스를 시작합니다...")
bus.connect()
print("연결 성공! 이제 키보드로 제어할 수 있습니다.")

# ==== 5. 제어 루프 ====
try:
    while True:
        # 현재 위치 읽기
        current_pos = bus.read("Present_Position")
        print("\n--- 현재 모터 상태 ---")
        for name, pos in current_pos.items():
            print(f"{name}: {pos}")

        user_input = input("\n[명령 입력] 모터번호 목표값 (예: 3 2000), 종료는 'exit': ").strip()
        
        if user_input.lower() == 'exit':
            break

        try:
            m_id, target = map(int, user_input.split())
            if m_id in ID_TO_NAME:
                name = ID_TO_NAME[m_id]
                # 안전을 위해 한 번에 500 이상 움직이지 않도록 제한
                current = current_pos[name]
                target = max(min(target, current + 500), current - 500)
                
                bus.write("Goal_Position", {name: target})
                print(f">> {name}(ID {m_id})를 {target}으로 이동 명령 전송.")
                time.sleep(0.5)
            else:
                print("잘못된 모터 번호입니다 (1~6 입력).")
        except ValueError:
            print("입력 형식이 잘못되었습니다. '번호 값' 형태로 입력하세요.")

finally:
    print("연결을 안전하게 해제합니다...")
    bus.disconnect()