import serial
import time
import glob
import os

def next_filename(base="test", ext=".txt"):
    pattern = f"{base}_*{ext}"
    files = glob.glob(pattern)
    max_i = -1
    for f in files:
        name, _ = os.path.splitext(os.path.basename(f))
        parts = name.split("_")
        if parts[-1].isdigit():
            idx = int(parts[-1])
            if idx > max_i:
                max_i = idx
    return f"{base}_{max_i+1}{ext}"

outfile = next_filename("test", ".txt")
print(f"Writing data to {outfile}")

arduino = serial.Serial(port='COM3', baudrate=115200, timeout=1)
time.sleep(7)  # let the USB serial settle
start = time.perf_counter()
with open(outfile, "a", buffering=1) as f:
    try:
        while True:
            line = arduino.readline()
            if not line:
                continue
            text = line.decode(errors="replace").rstrip("\r\n")
            # elapsed seconds since program start
            ts = time.perf_counter() - start # starts at 0.0 when Python launched
            entry = f"[{ts:.3f}] {text}"
            print(entry)
            f.write(entry + "\n")
    except KeyboardInterrupt:
        print("\nStopping")
        arduino.close()