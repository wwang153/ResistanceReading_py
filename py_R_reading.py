import serial
import time
import threading
import csv
import os
import argparse
import tkinter.messagebox
import math

# ============================================================
# ARGUMENT PARSING
# ============================================================

parser = argparse.ArgumentParser(description="Record encoder and resistance data")

parser.add_argument(
    "filename",
    help="Output CSV filename"
)

parser.add_argument(
    "--res",
    choices=["arduino", "usb", "none"],
    default="arduino",
    help="Resistance device (default: arduino)"
)

parser.add_argument(
    "--arduino_sensors",
    type=int,
    default=1,
    help="Number of resistance sensors connected to Arduino (default: 1)"
)

args = parser.parse_args()

USE_ARDUINO_RES = args.res == "arduino"
USE_USB_RES = args.res == "usb"

N_ARDUINO = args.arduino_sensors

# CSV_FILENAME = args.filename
CSV_FILENAME = os.path.join('data', args.filename)
LOG_RATE_HZ = 10

# ============================================================
# SERIAL CONFIGURATION
# ============================================================

ENCODER_PORT = "/dev/ttyACM0"
ENCODER_BAUD = 9600

ARDUINO_RES_PORT = "/dev/ttyACM2"
ARDUINO_RES_BAUD = 9600

USB_RES_PORT = "/dev/ttyACM2"
USB_RES_BAUD = 9600

# ============================================================
# RESISTANCE METER CLASS (UNCHANGED)
# ============================================================

class Usb_rs:

    def __init__(self, gui=False):
        self.ser = serial
        self.gui = gui
    
    def open(self, port, speed):
        try:
            self.ser = serial.Serial(port, speed, timeout=0)
            return True
        except Exception as e:
            if self.gui:
                tkinter.messagebox.showerror("Open Error", e)
            else:
                print("Open Error:", e)
            return False

    def close(self):
        try:
            self.ser.close()
            return True
        except Exception as e:
            print("Close Error:", e)
            return False

    def sendMsg(self, strMsg):
        try:
            self.ser.write(bytes(strMsg + '\r\n', 'utf-8'))
            return True
        except Exception as e:
            print("Send Error:", e)
            return False

    def receiveMsg(self, timeout):
        msgBuf = bytes()
        start = time.time()

        while True:
            if self.ser.inWaiting() > 0:
                rcv = self.ser.read(1)
                if rcv == b"\n":
                    return msgBuf.decode('utf-8')
                elif rcv != b"\r":
                    msgBuf += rcv

            if time.time() - start > timeout:
                return "Timeout"

    def SendQueryMsg(self, strMsg, timeout):
        if self.sendMsg(strMsg):
            return self.receiveMsg(timeout)
        return "Error"

# ============================================================
# USB RESISTANCE READING
# ============================================================

def read_usb_resistance():
    meter = Usb_rs()
    if not meter.open(USB_RES_PORT, USB_RES_BAUD):
        return None

    command = ":MEASure?"
    response = meter.SendQueryMsg(command, 1)
    meter.close()

    try:
        return float(response)
    except:
        return None

# ============================================================
# SHARED DATA (THREAD SAFE)
# ============================================================

latest_encoder = math.nan
latest_usb_resistance = math.nan
latest_arduino_resistance = [math.nan] * N_ARDUINO

data_lock = threading.Lock()

# ============================================================
# THREAD: ENCODER
# ============================================================

def encoder_thread():
    global latest_encoder
    ser = serial.Serial(ENCODER_PORT, ENCODER_BAUD, timeout=1)
    buffer = ""

    while True:
        buffer += ser.read(ser.inWaiting()).decode(errors="ignore")
        if '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            try:
                with data_lock:
                    latest_encoder = float(line.strip())
            except:
                pass

# ============================================================
# THREAD: ARDUINO RESISTANCE (R1,R2,R3,...)
# ============================================================

def arduino_resistance_thread():
    global latest_arduino_resistance
    ser = serial.Serial(ARDUINO_RES_PORT, ARDUINO_RES_BAUD, timeout=1)
    buffer = ""

    while True:
        buffer += ser.read(ser.inWaiting()).decode(errors="ignore")
        if '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            parts = line.strip().split(',')

            if len(parts) != N_ARDUINO:
                continue

            try:
                values = [float(p) for p in parts]
                with data_lock:
                    latest_arduino_resistance = values
            except:
                pass

# ============================================================
# THREAD: USB RESISTANCE
# ============================================================

def usb_resistance_thread():
    global latest_usb_resistance
    while True:
        value = read_usb_resistance()
        if value is not None:
            with data_lock:
                latest_usb_resistance = value
        time.sleep(0.1)

# ============================================================
# CSV LOGGER (TIME STARTS AT 0)
# ============================================================

def csv_logger():
    file_exists = os.path.isfile(CSV_FILENAME)

    start_time = time.perf_counter()  # t = 0 reference

    with open(CSV_FILENAME, "a", newline="") as f:
        writer = csv.writer(f)

        if not file_exists:
            header = ["time_sec", "encoder"]
            for i in range(N_ARDUINO):
                header.append(f"arduino_f{i+1}")
            header.append("usb_resistance")
            writer.writerow(header)

        print("\n================= LOGGING STARTED =================")
        print(f"File: {CSV_FILENAME}")
        print(f"Resistance mode: {args.res}")
        print(f"Arduino sensors: {N_ARDUINO}")
        print("Time resolution: 0.1 s")
        print("===================================================\n")

        while True:
            time.sleep(1.0 / LOG_RATE_HZ)

            # --- time with 0.1 s accuracy ---
            elapsed_time = round(time.perf_counter() - start_time, 2)

            with data_lock:
                arduino_vals = (
                    latest_arduino_resistance if USE_ARDUINO_RES
                    else [math.nan] * N_ARDUINO
                )

                usb_val = latest_usb_resistance if USE_USB_RES else math.nan
                encoder_val = latest_encoder

                row = [
                    elapsed_time,
                    encoder_val,
                    *arduino_vals,
                    usb_val
                ]

            # --- write CSV ---
            writer.writerow(row)
            f.flush()

            # --- terminal output ---
            arduino_str = (
                "[" + ", ".join(f"{v:.3f}" for v in arduino_vals) + "]"
                if USE_ARDUINO_RES else "disabled"
            )

            usb_str = f"{usb_val:.3f}" if USE_USB_RES else "disabled"

            print(
                f"t={elapsed_time:>4.2f} | "
                f"enc={encoder_val:.4f} | "
                f"A={arduino_str} | "
                f"USB={usb_str}"
            )

# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    threading.Thread(target=encoder_thread, daemon=True).start()

    if USE_ARDUINO_RES:
        threading.Thread(target=arduino_resistance_thread, daemon=True).start()

    if USE_USB_RES:
        threading.Thread(target=usb_resistance_thread, daemon=True).start()

    csv_logger()
