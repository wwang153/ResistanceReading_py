import csv
import math
import time
from ViscoelasticCompensator import Compensator


# ===============================
# USER SETTINGS
# ===============================

CSV_FILE = "data/HeatShrink_2mmTopMold_SingleCycle.csv"
OUTPUT_FILE = "data/HeatShrink_2mmTopMold_SingleCycle_filtered.csv"

DATA_RATE = 100 # Hz

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
# Process CSV (scalar R and t)
# ===============================

with open(CSV_FILE, newline="") as infile, \
     open(OUTPUT_FILE, "w", newline="") as outfile:

    reader = csv.DictReader(infile)
    fieldnames = reader.fieldnames + ["filtered_f1"]
    writer = csv.DictWriter(outfile, fieldnames=fieldnames)

    writer.writeheader()

    for row in reader:

        # ---- scalar resistance ----
        try:
            R = float(row["arduino_f1"])
        except:
            R = math.nan

        # ---- scalar time ----
        try:
            t_file = float(row["time_sec"])
        except:
            t_file = None

        # ---- single-value update ----
        filtered = comp.update(R, t=t_file)
        print("Compensation Done: ")
        print(f"t={t_file:.3f}, R={R:.3f}, filtered={filtered:.3f}")

        row["filtered_f1"] = filtered
        time.sleep(0.0001)
        writer.writerow(row)

print("Filtering complete.")