#!/usr/bin/env python3
#*****************************************************************************
# UTM projection conversion test
# Copyright (c) 2013-2020, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*****************************************************************************
"""
This file contains a simple Python script to test the UTM conversion class.

Revision
2013-04-05 KJ First version
2015-03-09 KJ Minor update of the license text.
2016-01-16 KJ Corrected a minor problem with the library location reference.
2020-02-03 KJ Python 3 compatible, changed functionality: it now moves a
              geodetic position 100 meter East
2020-09-17 KJ Changed first line to python3
"""

def great_circle_distance(lat1, lon1, lat2, lon2):
    lat_1_rad = lat1 * pi / 180.0
    lat_2_rad = lat2 * pi / 180.0
    lon_1_rad = lon1 * pi / 180.0
    lon_2_rad = lon2 * pi / 180.0
    d = acos(sin(lat_1_rad) * sin(lat_2_rad) + cos(lat_1_rad) * cos(lat_2_rad) * cos(lon_2_rad - lon_1_rad)) * 6371000.0
    #If the distance is short (<1km) switch to the haversine formula to avoid rounding errors (nowadays we don't have rounding problems usually)
    if d < 1000:
        d = 2*asin(sqrt((sin((lat_2_rad - lat_1_rad)/2.0))**2 + cos(lat_1_rad) * cos(lat_2_rad) * (sin((lon_2_rad - lon_1_rad)/2.0))**2)) * 6371000.0 
    return d

# import utmconv class
from utm import utmconv
from math import pi, cos, acos, sqrt, sin, asin
import matplotlib.pyplot as plt
import numpy as np
import os

CARTELLA_SCRIPT = os.path.dirname(os.path.abspath(__file__))

# geodetic reference coordinate
lat1 =  55.0
lon1 = 009.0

print ('First position [deg]:')
print ('  latitude:  %.8f'  % (lat1))
print ('  longitude: %.8f'  % (lon1))

# instantiate utmconv class
uc = utmconv()

# convert from geodetic to UTM
(hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm (lat1,lon1)
print ('\nConverted from geodetic to UTM [m]')
print ('  %d %c %.5fe %.5fn' % (zone, letter, e1, n1))

# now generating the second UTM coordinate
e2 = e1 + 100.0 
n2 = n1 

# convert back from UTM to geodetic
(lat2, lon2) = uc.utm_to_geodetic (hemisphere, zone, e2, n2)
print ('\nSecond position 100 meter East [deg]:')
print ('  latitude:  %.8f'  % (lat2))
print ('  longitude: %.8f'  % (lon2))
print ('\nDistance between the two positions [m]: %.3f' % (great_circle_distance(lat1, lon1, lat2, lon2)))

plot_all = False

if plot_all == True:
    lat1 =  -70.0
    lon1 = 090.0
    (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm (lat1,lon1)
    print ('\nConverted from geodetic to UTM [m]')
    print ('  %d %c %.5fe %.5fn' % (zone, letter, e1, n1))
    error = []
    angle = []
    legend = []
    distance = 10000
    step = 10
    colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'orange', 'purple']
    plt.figure(figsize=(10, 6))
    for j in range(0, 90, step):
        lat1 = float(j)
        lon1 = 0.0
        error = []
        angle = []
        (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm (lat1,lon1)
        legend.append('Latitude: %d deg' % (lat1))
        for i in range(1, 90):
            e2 = e1 + distance*cos(i*pi/180.0)
            n2 = n1 + distance*sin(i*pi/180.0)
            (lat2, lon2) = uc.utm_to_geodetic (hemisphere, zone, e2, n2)
            ep = (distance-great_circle_distance(lat1, lon1, lat2, lon2))/distance
            error.append(ep*100)
            angle.append(i)
        plt.plot(angle, error, color=colors[j//step])
    plt.title('Error of UTM conversion')
    plt.xlabel('Angle [deg]')
    plt.ylabel('Error (%)')
    plt.legend(legend)
    plt.grid()
    file_path = os.path.join(CARTELLA_SCRIPT, 'utm_error_lat.png')
    plt.savefig(file_path, dpi=300)
    plt.show()


    legend = []
    plt.figure(figsize=(10, 6))
    for j in range(0, 10, 1):
        lat1 = 0
        lon1 = float(j)
        error = []
        angle = []
        (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm (lat1,lon1)
        legend.append('Longitude: %d deg' % (lon1))
        for i in range(1, 90):
            e2 = e1 + distance*cos(i*pi/180.0)
            n2 = n1 + distance*sin(i*pi/180.0)
            (lat2, lon2) = uc.utm_to_geodetic (hemisphere, zone, e2, n2)
            ep = (distance-great_circle_distance(lat1, lon1, lat2, lon2))/distance
            error.append(ep*100)
            angle.append(i)
        plt.plot(angle, error, color=colors[j//10])
    plt.title('Error of UTM conversion')
    plt.xlabel('Angle [deg]')
    plt.ylabel('Error (%)')
    plt.legend(legend)
    plt.grid()
    file_path = os.path.join(CARTELLA_SCRIPT, 'utm_error_lon.png')
    plt.savefig(file_path, dpi=300)
    plt.show()

    legend = []
    plt.figure(figsize=(10, 6))
    for j in range(0, 90, 1):
        lat1 = 0
        lon1 = j
        error = []
        angle = []
        (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm (lat1,lon1)
        for distance in range(1000, 10000, 100):
            e2 = e1 + distance
            n2 = n1
            (lat2, lon2) = uc.utm_to_geodetic (hemisphere, zone, e2, n2)
            ep = (distance-great_circle_distance(lat1, lon1, lat2, lon2))/distance
            error.append(ep*100)
            angle.append(distance)
            legend.append('Distance: %d m' % (distance))
        plt.plot(angle, error, color=colors[j//10])
    plt.title('Error of UTM conversion')
    plt.xlabel('Distance [m]')
    plt.ylabel('Error (%)')
    plt.legend(legend)
    plt.grid()
    file_path = os.path.join(CARTELLA_SCRIPT, 'utm_error_distance.png')
    plt.savefig(file_path, dpi=300)
    plt.show()



#calcola l'errore di conversione da geodetico a UTM partendo da longitudine 3° a longitudine 6
#e per ogni longitudine calcola l'errore per latitudini da 0° a 90° con passo di 1°
#e plotta un grafico 3D con latitudine, longitudine e errore di conversione
lat_plot = []
lon_plot = []
error_plot = []
distance = 1000
for lat in range(0, 80):
    for i in range(0, 100, 5):
        lon = 54 + i*6/100
        (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm(lat, lon)
        (lat2, lon2) = uc.utm_to_geodetic(hemisphere, zone, e1+distance, n1)
        error = (1-(great_circle_distance(lat, lon, lat2, lon2) /distance))*100
        lat_plot.append(lat)
        lon_plot.append(lon)
        error_plot.append(error)

# --- Plot 3D Mesh ---
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Mesh triangolata
surf = ax.plot_trisurf(
    lon_plot, lat_plot, error_plot, cmap='viridis', edgecolor='black', linewidth=0.2
)
lon = 55.47
lat = 010.33
(hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm(lat, lon)
(lat2, lon2) = uc.utm_to_geodetic(hemisphere, zone, e1+distance, n1)
error = (1-(great_circle_distance(lat, lon, lat2, lon2) /distance))*100
ax.view_init(elev=30, azim=-45)
ax.set_box_aspect(None, zoom=0.9)
ax.set_xlabel('Longitude [°]')
ax.set_ylabel('Latitude [°]')
ax.set_zlabel('Error [%]')
ax.set_title('Error of UTM conversion for 1000 m East')
fig.colorbar(surf, shrink=0.5, aspect=5)
plt.show()

lat_plot = []
lon_plot = []
error_plot = []
distance = 1000
for lat in range(0, 80):
    for i in range(0, 100, 5):
        lon = 54 + i*6/100
        (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm(lat, lon)
        (lat2, lon2) = uc.utm_to_geodetic(hemisphere, zone, e1, n1+distance)
        error = (1-(great_circle_distance(lat, lon, lat2, lon2) /distance))*100
        lat_plot.append(lat)
        lon_plot.append(lon)
        error_plot.append(error)

# --- Plot 3D Mesh ---
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Mesh triangolata
surf = ax.plot_trisurf(
    lon_plot, lat_plot, error_plot, cmap='viridis', edgecolor='black', linewidth=0.2
)
ax.view_init(elev=30, azim=-45)
ax.set_box_aspect(None, zoom=0.9)
ax.set_xlabel('Longitude [°]')
ax.set_ylabel('Latitude [°]')
ax.set_zlabel('Error [%]')
ax.set_title('Error of UTM conversion for 1000 m North')
fig.colorbar(surf, shrink=0.5, aspect=5)
plt.show()
# Add a color bar which maps values to colors.

a = 6378137.0
b = 6356752.3
f = (a-b)/a
e = sqrt(2*f-f**2)
N0 = a/sqrt(1-e**2*sin(lat2*pi/180.0)**2)
h = 0.0
X = (N0+h)*cos(lat2*pi/180.0)*cos(lon2*pi/180.0)
Y = (N0+h)*cos(lat2*pi/180.0)*sin(lon2*pi/180.0)
Z = (N0*(1-e**2)+h)*sin(lat2*pi/180.0)