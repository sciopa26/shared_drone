#prints the trajectory given a csv with lat and lon

from utm import utmconv
import csv
import matplotlib.pyplot as plt
import numpy as np
path = "/home/daniele/Documents/introduction_to_drone_technology/Module 7 - UAS positioning and navigation/miao.csv"
log = csv.reader(open(path, "r"), delimiter=",")
uc = utmconv()

e = []
n = []
e_point = []
n_point = []
counter = 0
for row in log:
    try:
        counter += 1
        (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm (float(row[3]),float(row[4]))
        e.append(e1)
        n.append(n1)
        if counter % 10 == 0:
            e_point.append(e1)
            n_point.append(n1)
    except:
        pass

e = np.array(e)
n = np.array(n)
e = e - e[0]*np.ones(len(e))
n = n - n[0]*np.ones(len(n))
plt.axis('equal')
plt.plot(e, n,'ro', markersize=1, label='Trajectory')
plt.plot(e[0], n[0],'go', markersize=10, label='Start point')
plt.plot(e[len(e)-1], n[len(n)-1],'bo', markersize=5, label='End point')
plt.xlabel('Easting [m]')
plt.ylabel('Northing [m]')
plt.legend()
plt.grid()
plt.show()
