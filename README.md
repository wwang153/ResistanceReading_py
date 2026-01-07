
# Sensor Data Logger (Python)

A single Python script for recording **encoder data** from CTR platform and **resistance sensor data** from Arduino or a USB Hioki meter.  
All data is written to CSV with **time starting at 0 seconds**.

---

## Features

- Encoder reading via Arduino on CTR platform (serial)
- Resistance sensors connected to Arduino reading by ADS1220
- Optional USB resistance meter (Hioki IM3536 LCR meter)
- Time starts at **0 seconds** when logging begins
- Always records the **newest available data**
- Sampling Rate: **10 Hz**, adjustable by `LOG_RATE_HZ = 10
`

---

## Time Behavior

- Time reference: elapsed time
- Start value: `0`
- Units: seconds
- Implementation: `time.perf_counter()` (monotonic, high resolution)

---

## How to Run it?
```bash
python3 py_R_reading.py <fileName.csv> --res <arduino|usb|none> --arduino_sensors <sensor_number>
```

## Example Output Format with 3 arduino sensors
```csv
time_sec,encoder,arduino_R1,arduino_R2,arduino_R3,usb_resistance
0.000,0.134,12.3,45.6,78.9,nan
0.100,0.136,12.4,45.5,78.8,nan
0.200,0.139,12.5,45.4,78.7,nan
```