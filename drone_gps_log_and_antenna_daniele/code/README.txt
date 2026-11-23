These scripts form a small toolkit for a drone telemetry and positioning course (Module 7). They log GPS and radio data from a Pixhawk flight controller through ROS 2/MAVROS, convert it to metric coordinates, and plot or simulate trajectories.

Data acquisition (ROS 2 / MAVROS)

all.py is a live monitor node called TelemetryMonitor. It subscribes to the MAVROS topics for connection state, radio status (RSSI), diagnostics, fused global position and raw GPS. It prints incoming values and warns every 5 s if the FCU isn't connected or no data is arriving. 
It logs nothing to disk and is mainly a connectivity and debugging tool.

all_filtered.py is the actual logger, TelemetryLogger. It samples at a fixed rate (default 1 Hz) and writes a CSV with these columns: timestamp, lat, lon, alt (WGS84), horizontal and vertical accuracy, local RSSI and remote RSSI. Details:

Position comes from the EKF-filtered /mavros/global_position/global.
Accuracy comes from the covariance matrix, with raw GPS eph/epv as a fallback.
RSSI comes from /mavros/radio_status, with /diagnostics as a fallback.
Missing readings are written as NaN so columns never shift.
You type save + Enter to stop cleanly. Usage: python3 all_filtered.py log.csv [--rate N].
It needs MAVROS running in another shell (the launch command is in the header comment).
Coordinate conversion library (Kjeld Jensen, SDU)

transverse_mercator.py contains the tranmerc class. It converts between geodetic coordinates and the Transverse Mercator projection, using series expansions based on US Army TEC and Snyder.

utm.py contains the utmconv class, built on tranmerc. It provides geodetic_to_utm(lat, lon), which returns hemisphere, zone, letter, easting and northing. It also provides utm_to_geodetic(...) for the inverse. It handles the special zones for Denmark/SW Norway and Svalbard and supports a zone override.

utm_test.py is a test and analysis script for that library. It converts a point, moves it 100 m east, converts back and checks the distance with a great-circle/haversine function. It then produces 3D surface plots of the conversion error (for 1000 m east and 1000 m north offsets) as a function of latitude and longitude. A plot_all flag enables extra error plots versus angle, longitude and distance. The script ends with a geodetic-to-ECEF (X, Y, Z) computation. Note that the latitude/longitude loops use lon = 54 + ..., and the lon = 55.47, lat = 010.33 point looks like swapped values, so those experiments may not be testing what was intended.

Analysis and plotting

conv.py reads a CSV, converts lat/lon (columns 3 and 4) to UTM, subtracts the first point so the start is at (0, 0), and plots the trajectory in easting/northing with start and end markers.

read_ros_log.py is the fuller visualization of logs from all_filtered.py. It parses the CSV with DictReader and drops rows without a position. After converting to UTM relative to the start point, it produces:

A 2D trajectory colored by time (sample index).
A 3D trajectory with altitude on the Z axis.
A two-panel RSSI figure: a spatial map of local RSSI over the trajectory, and local vs remote RSSI over time.

Note that the final RSSI arrays are built from log while e and n may be shorter if any conversion failed, so they could misalign in that case.

Simulation and work in progress

fake_trajectory.py generates a synthetic "iron hysteresis loop" trajectory in east/north coordinates. Its parameters are the bounding box, loop opening width (found by bisection), rotation, saturation sharpness, number of points, and uniform arc-length resampling. It can add Gaussian noise with a separate mean and variance for each axis. 
It exports a CSV and plots the nominal vs noisy path. It is useful for testing filtering or outlier removal on data with a known ground truth.

rm_outliers.py is an unfinished stub marked #wip. It only sets up imports, loads the file path and defines MAX_SPEED = 1.5 m/s, which suggests the plan is to reject GPS points that imply a speed above that.

How they fit together
all.py (monitor)           all_filtered.py (log to CSV)
                                   │
                  ┌────────────────┴───────────────┐
            read_ros_log.py                    conv.py
        (2D/3D/RSSI plots)               (simple 2D plot)
                  └──────── utm.py ← transverse_mercator.py
                                   ▲
                              utm_test.py (validates accuracy)

fake_trajectory.py → synthetic CSV → rm_outliers.py (wip)