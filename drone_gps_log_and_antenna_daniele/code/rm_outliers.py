from datetime import datetime
from utm import utmconv
import csv
import matplotlib.pyplot as plt
import numpy as np

path = "/home/daniele/Documents/shared_drone/drone_gps_log_and_antenna_daniele/logs/miao.csv"
log = csv.reader(open(path, "r"), delimiter=",")
uc = utmconv()

normal_speed = 5 / 3.6  # m/s
margin = 2
MAX_SPEED = normal_speed * margin * margin  # Effective speed threshold in m/s

t, e, n = [], [], []

for row in log:
    try:
        # Uncomment to filter by a single ROS 2 topic:
        # if row[1] != "global_pos_raw":
        #     continue

        ts = datetime.fromisoformat(row[0]).timestamp()
        (_, _, _, e1, n1) = uc.geodetic_to_utm(float(row[3]), float(row[4]))
        t.append(ts)
        e.append(e1)
        n.append(n1)
    except:
        pass

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