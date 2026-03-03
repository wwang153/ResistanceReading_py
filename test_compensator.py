import csv
import time
import math
from ViscoelasticCompensator import Compensator


# ===============================
# USER SETTINGS
# ===============================

CSV_FILE = "your_creep_file.csv"
DATA_RATE = 100  # Hz

# your model parameters
p0 = 0.073761781995436
p1 = 3.452818489875698
q0 = 0.052643248351074
q1 = 3.464251491171303

# ===============================
# Create compensator
# ===============================

comp = Compensator(
    mode="creep",
    p0=p0,
    p1=p1,
    q0=q0,
    q1=q1,
    data_rate=DATA_RATE
)

# ===============================
# Replay CSV as real-time stream
# ===============================

with open(CSV_FILE, newline="") as f:
    reader = csv.DictReader(f)

    print("\n===== STARTING REPLAY =====\n")

    for row in reader:

        try:
            R = float(row["arduino_f1"])
        except:
            R = math.nan

        compensated = comp.update(R)

        print(f"Raw: {R:.6f}  |  Compensated: {compensated:.6f}")

        time.sleep(1.0 / DATA_RATE)

print("\n===== REPLAY FINISHED =====")