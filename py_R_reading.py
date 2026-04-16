import serial
import time
import threading
import csv
import os
import argparse
import tkinter.messagebox
import math
from ViscoelasticCompensator import Compensator

import socket # Ensure this is at the top with other imports

# ============================================================
# UDP CONFIGURATION (Laptop B - Ubuntu)
# ============================================================
UDP_IP = "0.0.0.0"  # Listen on all available network interfaces
UDP_PORT = 5005     # Must match the port used on Laptop A (Mac)

# Add to SHARED DATA section
latest_cable_lengths = [math.nan, math.nan, math.nan]
latest_t_diff = math.nan

# ===============================
# COMPENSATOR PARAMETERS
# ===============================

DATA_RATE = 100

# ===============================
# COMPENSATOR PARAMETERS
# ===============================

DATA_RATE = 100

comp_modes = ["creep", "relaxation"]

sensor_params = {
    1: {
        "p0": 7.460277003272818,
        "p1": 58.1839911145026852,
        "q0": 6.889476568600796,
        "q1": 54.732219793646117,
        "pm": "+"
    },
    2: {
        "p0": 0.273887176281407,
        "p1": 2.951018677512885,
        "q0": 0.193781115841128,
        "q1": 3.087909700485219,
        "pm": "+"
    },
    # add more sensors here if needed
    3: {
        "p0": 0.05,
        "p1": 3.1,
        "q0": 0.04,
        "q1": 3.2
    },
}

# ============================================================
# ARGUMENT PARSING
# ============================================================

parser = argparse.ArgumentParser(description="Record encoder and resistance data")

parser.add_argument("filename", help="Output CSV filename")

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

parser.add_argument(
    "--use_comp",
    action="store_true",
    help="Enable viscoelastic compensator (default: disabled)"
)

args = parser.parse_args()

USE_ARDUINO_RES = args.res == "arduino"
USE_USB_RES = args.res == "usb"
USE_COMP = args.use_comp
N_ARDUINO = args.arduino_sensors

CSV_FILENAME = os.path.join('data', args.filename)

# ===============================
# Create compensators (one per sensor)
# ===============================

if USE_COMP and USE_ARDUINO_RES:

    compensators = []

    for i in range(1, N_ARDUINO + 1):

        if i not in sensor_params:
            raise ValueError(f"No parameters defined for sensor {i}")

        params = sensor_params[i]

        comp = Compensator(
            mode=comp_modes[i-1],
            p0=params["p0"],
            p1=params["p1"],
            q0=params["q0"],
            q1=params["q1"],
            data_rate=DATA_RATE,
            pm=params["pm"]
        )

        compensators.append(comp)

else:
    compensators = None

# ============================================================
# SERIAL CONFIGURATION
# ============================================================

ENCODER_PORT = "/dev/ttyACM0"
ENCODER_BAUD = 9600

ARDUINO_RES_PORT = "/dev/ttyACM2"
ARDUINO_RES_BAUD = 115200

USB_RES_PORT = "/dev/ttyACM2"
USB_RES_BAUD = 9600

# ============================================================
# RESISTANCE METER CLASS
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
# SHARED DATA
# ============================================================

latest_encoder = math.nan
latest_usb_resistance = math.nan
latest_arduino_resistance = [math.nan] * N_ARDUINO

data_lock = threading.Lock()

# ============================================================
# THREADS
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

def usb_resistance_thread():
    global latest_usb_resistance
    while True:
        value = read_usb_resistance()
        if value is not None:
            with data_lock:
                latest_usb_resistance = value
        time.sleep(0.1)

def udp_cable_thread():
    global latest_cable_lengths, latest_t_diff
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Increase buffer size to prevent lag if data builds up
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1) 
    sock.bind((UDP_IP, UDP_PORT))
    
    print(f"UDP Listener started on port {UDP_PORT}...")
    
    while True:
        try:
            # Receive data
            data, addr = sock.recvfrom(1024)
            t_arrival = time.time()
            
            # Expecting: [t_mac, l1, l2, l3]
            parts = data.decode().split(',')
            
            if len(parts) >= 4:
                t_mac = float(parts[0]) # Now index 0
                lengths = [float(x) for x in parts[1:4]] # Now indices 1, 2, 3
                
                with data_lock:
                    latest_cable_lengths = lengths
                    latest_t_diff = t_arrival - t_mac
        except Exception as e:
            # Using pass to keep the terminal clean, or print for debugging
            pass

# ============================================================
# CSV LOGGER
# ============================================================

def csv_logger():
    file_exists = os.path.isfile(CSV_FILENAME)
    start_time = time.perf_counter()

    with open(CSV_FILENAME, "a", newline="") as f:
        writer = csv.writer(f)

        # ============================================================
        # 1. HEADER GENERATION
        # ============================================================
        if not file_exists:
            # Base headers
            header = ["time_sec", "encoder"]
            
            # Arduino Resistance headers
            for i in range(N_ARDUINO):
                header.append(f"arduino_f{i+1}")
            
            # USB Resistance header
            header.append("usb_resistance")

            # NEW: Cable length and Latency headers
            header.extend(["l1", "l2", "l3", "t_diff"])

            # Compensator headers (if enabled)
            if USE_COMP and USE_ARDUINO_RES:
                for i in range(N_ARDUINO):
                    header.append(f"filtered_f{i+1}")

            writer.writerow(header)

        print("\n================= LOGGING STARTED =================")
        print(f"File: {CSV_FILENAME}")
        print(f"Use compensator: {USE_COMP}")
        print(f"UDP Target: {UDP_IP}:{UDP_PORT}")
        print("===================================================\n")

        # ============================================================
        # 2. DATA COLLECTION LOOP
        # ============================================================
        while True:
            # Maintain the loop at the specified DATA_RATE (e.g., 100Hz)
            time.sleep(1.0 / DATA_RATE)
            elapsed_time = round(time.perf_counter() - start_time, 2)

            with data_lock:
                # Capture current snapshots of all shared variables
                arduino_vals = (
                    latest_arduino_resistance if USE_ARDUINO_RES
                    else [math.nan] * N_ARDUINO
                )
                usb_val = latest_usb_resistance if USE_USB_RES else math.nan
                encoder_val = latest_encoder
                
                # Capture current cable data from UDP thread
                cable_vals = latest_cable_lengths  # [l1, l2, l3]
                t_diff = latest_t_diff             # Latency calculation

            # Construct the row: [Time, Encoder, Arduinos..., USB, L1, L2, L3, T_Diff]
            row = [elapsed_time, encoder_val, *arduino_vals, usb_val, *cable_vals, t_diff]

            # ---- Apply Viscoelastic Compensator (if enabled) ----
            filtered_vals = []
            if USE_COMP and USE_ARDUINO_RES:
                for i in range(N_ARDUINO):
                    R = arduino_vals[i]
                    # Note: compensators list is 0-indexed
                    filtered = compensators[i].update(R, t=elapsed_time)
                    filtered_vals.append(filtered)
                
                # Append filtered values to the end of the CSV row
                row.extend(filtered_vals)

            # Write to file and force sync to disk
            writer.writerow(row)
            f.flush()

            # ============================================================
            # 3. TERMINAL FEEDBACK (MONITORING)
            # ============================================================
            arduino_str = (
                "[" + ", ".join(f"{v:.2f}" for v in arduino_vals) + "]"
                if USE_ARDUINO_RES else "N/A"
            )

            cable_str = (
                f"L:[{cable_vals[0]:.1f}, {cable_vals[1]:.1f}, {cable_vals[2]:.1f}]"
                if not math.isnan(cable_vals[0]) else "L:[N/A]"
            )

            latency_str = f"dt={t_diff:.4f}s" if not math.isnan(t_diff) else "dt=N/A"

            # Print a condensed status line to the console
            print(
                f"t={elapsed_time:>6.2f} | "
                f"Enc:{encoder_val:.3f} | "
                f"Ard:{arduino_str} | "
                f"{cable_str} | "
                f"{latency_str}"
            )
            
# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    threading.Thread(target=encoder_thread, daemon=True).start()

    threading.Thread(target=udp_cable_thread, daemon=True).start()

    if USE_ARDUINO_RES:
        threading.Thread(target=arduino_resistance_thread, daemon=True).start()

    if USE_USB_RES:
        threading.Thread(target=usb_resistance_thread, daemon=True).start()

    csv_logger()