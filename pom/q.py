import serial
import time

# =====================================================
# 모터를 전혀 건드리지 않습니다. 안전합니다.
# 서보 전원 빼놓고 돌려도 됩니다.
#
# 가변저항을 하나씩 돌려보면서
# 어떤 숫자가 바뀌는지 확인하세요
# =====================================================

PORT_ADC = "COM13"
BAUD_ADC = 115200

try:
    ser = serial.Serial(PORT_ADC, BAUD_ADC, timeout=1)
    print("포트 열기 성공!")
except Exception as e:
    print(f"포트 열기 실패: {e}")
    quit()

adc_raw = [0] * 19
parsed = [0] * 16
sw_toggle = 0

print("")
print("=" * 70)
print(" 가변저항을 하나씩 돌려보면서 어떤 숫자가 바뀌는지 확인하세요")
print(" 서보 전원은 빼놓으세요!")
print(" Ctrl+C로 종료")
print("=" * 70)
print("")
print(" p0    p1    p2    p3    p4    p5    p6  | p7    p8    p9   p10   p11   p12   p13 | Pan   Tilt  SW")
print(" ----전반부(parsed 0~6)----              | ----모터용(parsed 7~13)----              | --조이스틱--")
print("-" * 100)

try:
    while True:
        line = ser.readline().decode(errors='ignore').replace('\x00', '').strip()

        if not line:
            continue

        parts = line.split(",")

        if len(parts) < 19:
            continue

        for i in range(min(19, len(parts))):
            val_str = ''.join(filter(str.isdigit, parts[i]))
            if val_str != "":
                adc_raw[i] = int(val_str)

        for i in range(7):
            parsed[i] = adc_raw[i + 1]
        for i in range(7):
            parsed[i + 7] = adc_raw[i + 9]

        parsed[14] = adc_raw[16]
        parsed[15] = adc_raw[17]
        sw_toggle = adc_raw[18]

        front = " ".join([f"{parsed[i]:5d}" for i in range(7)])
        motor = " ".join([f"{parsed[i+7]:5d}" for i in range(7)])

        print(f"\033[F{front} | {motor} | {parsed[14]:5d} {parsed[15]:5d} {sw_toggle:3d}")

except KeyboardInterrupt:
    pass

ser.close()
print("\n종료")