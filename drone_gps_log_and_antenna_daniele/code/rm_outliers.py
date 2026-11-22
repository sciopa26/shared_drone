#wip
from utm import utmconv
import csv
import matplotlib.pyplot as plt
import numpy as np
path = "/home/daniele/Documents/shared_drone/drone_gps_log_and_antenna_daniele/logs/test_good_antenna.csv"
log = csv.reader(open(path, "r"), delimiter=",")
uc = utmconv()

MAX_SPEED = 1.5 #m/s


e = []
n = []
counter = 0
for row in log:
    if counter == 0:
        counter += 1
        continue
    if row[1] == "NaN" or row[2] == "NaN":
        continue
    (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm (float(row[1]),float(row[2]))
    e.append(e1)
    n.append(n1)

e = np.array(e)-np.mean(e)
n = np.array(n)-np.mean(n)

plt.plot(e, n, 'b.')
plt.axis('equal')
plt.grid()
plt.show()
