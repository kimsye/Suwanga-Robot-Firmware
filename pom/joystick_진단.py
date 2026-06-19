import serial
import time

# 모터를 전혀 건드리지 않습니다. 안전합니다.

PORT_ADC = "COM13"
BAUD_ADC = 115200

print(f"포트 {PORT_ADC} 열기 시도...")

try:
    ser = serial.Serial(PORT_ADC, BAUD_ADC, timeout=1)
    print("포트 열기 성공!")
except Exception as e:
    print(f"포트 열기 실패: {e}")
    quit()

print("")
print("=== 조이스틱 진단 ===")
print("조이스틱을 상하좌우로 움직여보세요")
print("조이스틱 버튼도 꾹 눌러보세요")
print("A7(좌우), A8(상하), A9(버튼) 값이 변하는지 확인!")
print("Ctrl+C로 종료")
print("")
print("  A0    A1    A2    A3    A4    A5    A6   | A7    A8    A9")
print("                                          | ← 여기를 보세요!")
print("-" * 70)

adc = [0] * 10

try:
    while True:
        raw_bytes = ser.readline()
        line = raw_bytes.decode(errors='ignore').replace('\x00', '').strip()

        if not line:
            continue

        parts = line.split(",")
        for i in range(min(10, len(parts))):
            val_str = ''.join(filter(str.isdigit, parts[i]))
            if val_str != "":
                adc[i] = int(val_str)

        a0_6 = " ".join([f"{adc[i]:5d}" for i in range(7)])
        a7 = adc[7]
        a8 = adc[8]
        a9 = adc[9]

        print(f"\033[F{a0_6} | {a7:5d} {a8:5d} {a9:5d}")

except KeyboardInterrupt:
    pass

ser.close()
print("\n종료")