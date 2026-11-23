import csv
import math
import matplotlib.pyplot as plt
import numpy as np
from utm import utmconv

path = "/home/daniele/Documents/shared_drone/drone_gps_log_and_antenna_daniele/logs/filt_out_good.csv"
uc = utmconv()

normal_speed = 5 / 3.6  # m/s
margin = 1.5

MAX_SPEED = normal_speed * margin * margin  # Effective speed threshold in m/s

t, e, n = [], [], []

with open(path, "r", newline="", encoding="utf-8") as f:
    log = csv.reader(f, delimiter=",")
    next(log, None)  # Skip CSV header

    for row in log:
        try:
            lat = float(row[2])
            lon = float(row[3])
            if math.isnan(lat) or math.isnan(lon):
                continue

            # Prefer exact ROS message stamp when available, otherwise use loop timestamp
            ros_ts = float(row[1])
            ts = ros_ts if not math.isnan(ros_ts) else float(row[0])

            (_, _, _, e1, n1) = uc.geodetic_to_utm(lat, lon)
            t.append(ts)
            e.append(e1)
            n.append(n1)
        except (ValueError, IndexError):
            continue

if not e:
    raise RuntimeError("No valid GNSS coordinates found in the CSV file.")

t = np.array(t)
# Center coordinates on the starting point
e = np.array(e) - e[0]
n = np.array(n) - n[0]

# Sequential outlier filtering based on the last valid point
t_clean = [t[0]]
e_clean = [e[0]]
n_clean = [n[0]]

outlier_e, outlier_n = [], []

for i in range(1, len(e)):
    dt = t[i] - t_clean[-1]
    if dt <= 0:
        continue

    dx = np.sqrt((e[i] - e_clean[-1])**2 + (n[i] - n_clean[-1])**2)
    speed = dx / dt

    if speed <= MAX_SPEED:
        t_clean.append(t[i])
        e_clean.append(e[i])
        n_clean.append(n[i])
    else:
        outlier_e.append(e[i])
        outlier_n.append(n[i])

print(f"Outliers detected: {len(outlier_e)}")

# Plot overlapping raw and filtered trajectories
plt.axis('equal')
plt.plot(e, n, 'r--', linewidth=0.8, alpha=0.5, label='Raw trajectory')
plt.plot(outlier_e, outlier_n, 'rx', markersize=6, label='Outliers')
plt.plot(e_clean, n_clean, 'b.-', markersize=3, linewidth=1.2, label='Filtered trajectory')
plt.plot(e_clean[0], n_clean[0], 'go', markersize=8, label='Start point')
plt.plot(e_clean[-1], n_clean[-1], 'ko', markersize=6, label='End point')

plt.xlabel('Easting [m]')
plt.ylabel('Northing [m]')
plt.legend()
plt.grid()
plt.show()