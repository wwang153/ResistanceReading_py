import serial
import time
import threading
import csv
import os
import argparse
import math

# ============================================================
# ARGUMENT PARSING
# ============================================================

parser = argparse.ArgumentParser(
    description="Record 6 capacitive sensor values from 2 Arduinos"
)

parser.add_argument(
    "filename",
    help="Output CSV filename"
)

args = parser.parse_args()

CSV_FILENAME = os.path.join("data", args.filename)
LOG_RATE_HZ = 10

# ============================================================
# SERIAL CONFIGURATION
# ============================================================

ARDUINO_1_PORT = "/dev/ttyACM0"
ARDUINO_2_PORT = "/dev/ttyACM1"
ARDUINO_BAUD = 115200

# ============================================================
# SHARED DATA (THREAD SAFE)
# ============================================================

latest_caps = [math.nan] * 6
data_lock = threading.Lock()

# ============================================================
# THREAD: GENERIC ARDUINO CAP READER
# ============================================================

def arduino_cap_thread(port, index_offset):
    """
    Reads 3 capacitive values from an Arduino.
    index_offset: 0 for Arduino 1, 3 for Arduino 2
    """
    ser = serial.Serial(port, ARDUINO_BAUD, timeout=1)
    buffer = ""

    print(f"[INFO] Connected to Arduino on {port}")

    while True:
        buffer += ser.read(ser.in_waiting).decode(errors="ignore")

        if "\n" in buffer:
            line, buffer = buffer.split("\n", 1)
            parts = line.strip().split(",")

            if len(parts) != 3:
                continue

            try:
                values = [float(v) for v in parts]
                with data_lock:
                    for i in range(3):
                        latest_caps[index_offset + i] = values[i]
            except ValueError:
                pass

# ============================================================
# CSV LOGGER
# ============================================================

def csv_logger():
    os.makedirs("data", exist_ok=True)
    file_exists = os.path.isfile(CSV_FILENAME)

    start_time = time.perf_counter()

    with open(CSV_FILENAME, "a", newline="") as f:
        writer = csv.writer(f)

        if not file_exists:
            header = ["time_sec"] + [f"cap_{i+1}" for i in range(6)]
            writer.writerow(header)

        print("\n================= LOGGING STARTED =================")
        print(f"File: {CSV_FILENAME}")
        print("Sensors: 6 capacitive (2 Arduinos × 3)")
        print("Rate: 10 Hz")
        print("===================================================\n")

        while True:
            time.sleep(1.0 / LOG_RATE_HZ)

            elapsed_time = round(time.perf_counter() - start_time, 3)

            with data_lock:
                row = [elapsed_time] + latest_caps.copy()

            writer.writerow(row)
            f.flush()

            caps_str = ", ".join(f"{v:8.3f}" for v in row[1:])
            print(f"t={elapsed_time:6.3f} | {caps_str}")

# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    threading.Thread(
        target=arduino_cap_thread,
        args=(ARDUINO_1_PORT, 0),
        daemon=True
    ).start()

    threading.Thread(
        target=arduino_cap_thread,
        args=(ARDUINO_2_PORT, 3),
        daemon=True
    ).start()

    csv_logger()
