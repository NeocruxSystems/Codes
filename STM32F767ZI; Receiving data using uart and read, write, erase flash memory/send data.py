import serial

PORT = "COM5"
BAUD = 1200
msg = "Hello my embedded colleagues"

with serial.Serial(PORT, BAUD, timeout=1) as ser:
    print(ser.readline().decode(errors="ignore").strip())
    ser.write((msg + "\n").encode("utf-8"))
