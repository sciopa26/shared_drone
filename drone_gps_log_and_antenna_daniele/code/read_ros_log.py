#colorful prints for the data logged from the flight controller
#!/usr/bin/env python3
import csv
import math
import matplotlib.pyplot as plt
import numpy as np
from utm import utmconv

# Internal path configuration
CSV_PATH = '/home/daniele/Documents/introduction_to_drone_technology/Module 7 - UAS positioning and navigation/test_good_antenna.csv'


def read_telemetry_csv(file_path: str) -> list[dict]:
    """Load lat, lon, alt, and RSSI values from the CSV into a Python list."""
    records = []

    with open(file_path, mode='r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # float('NaN') natively converts 'NaN' strings into IEEE 754 float NaNs
            records.append({
                'lat': float(row['lat_wgs84_deg']),
                'lon': float(row['lon_wgs84_deg']),
                'alt': float(row['alt_wgs84_m']),
                'local_rssi': float(row['local_rssi_dbm']),
                'remote_rssi': float(row['remote_rssi_dbm']),
            })

    return records


if __name__ == '__main__':
    telemetry_data = read_telemetry_csv(CSV_PATH)
    print(f"Loaded {len(telemetry_data)} samples.")
    if telemetry_data:
        print("Sample 0:", telemetry_data[0])

    # Adapt data structure for the UTM plotting block (expects lat at index 3, lon at index 4)
    uc = utmconv()
    log = [
        [d['local_rssi'], d['remote_rssi'], d['alt'], d['lat'], d['lon']]
        for d in telemetry_data
        if not math.isnan(d['lat']) and not math.isnan(d['lon'])
    ]

    e = []
    n = []
    e_point = []
    n_point = []
    altitude = []
    time = []
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
            altitude.append(row[2])
        except:
            pass
    e = np.array(e)
    n = np.array(n)
    e = e - e[0]*np.ones(len(e))
    n = n - n[0]*np.ones(len(n))

    # Progressive index to map time onto a color gradient
    t_idx = np.arange(len(e))

    plt.axis('equal')
    sc = plt.scatter(e, n, c=t_idx, cmap='viridis', s=8, label='Trajectory')
    plt.colorbar(sc, label='Sample index (Time)')
    plt.plot(e[0], n[0],'go', markersize=10, label='Start point')
    plt.plot(e[len(e)-1], n[len(n)-1],'bo', markersize=5, label='End point')
    plt.xlabel('Easting [m]')
    plt.ylabel('Northing [m]')
    plt.legend()
    plt.grid()
    plt.show()

    # 3D trajectory plot with altitude on Z-axis
    alt = np.array(altitude)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    sc3d = ax.scatter(e, n, alt, c=t_idx, cmap='viridis', s=8, label='Trajectory')
    fig.colorbar(sc3d, ax=ax, label='Sample index (Time)')
    ax.plot([e[0]], [n[0]], [alt[0]], 'go', markersize=10, label='Start point')
    ax.plot([e[-1]], [n[-1]], [alt[-1]], 'bo', markersize=5, label='End point')
    ax.set_xlabel('Easting [m]')
    ax.set_ylabel('Northing [m]')
    ax.set_zlabel('Altitude [m]')
    ax.legend()
    ax.grid(True)
    plt.show()

    # RSSI spatial distribution and link comparison
    local_rssi = np.array([float(row[0]) for row in log[:len(e)]])
    remote_rssi = np.array([float(row[1]) for row in log[:len(e)]])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # 1. Spatial RSSI map over trajectory
    ax1.set_aspect('equal')
    sc_rssi = ax1.scatter(e, n, c=local_rssi, cmap='RdYlGn', s=12, label='Local RSSI')
    fig.colorbar(sc_rssi, ax=ax1, label='Local RSSI [dBm]')
    ax1.plot(e[0], n[0], 'ko', markersize=8, label='Start / GCS')
    ax1.set_title('Spatial Signal Strength (Local RSSI)')
    ax1.set_xlabel('Easting [m]')
    ax1.set_ylabel('Northing [m]')
    ax1.legend()
    ax1.grid(True)

    # 2. Local vs Remote RSSI over time (gaps indicate NaN / dropped samples)
    ax2.plot(t_idx, local_rssi, '-o', markersize=3, label='Local RSSI (GCS)')
    ax2.plot(t_idx, remote_rssi, '-s', markersize=3, label='Remote RSSI (UAV)')
    ax2.set_title('Uplink vs Downlink RSSI Trend')
    ax2.set_xlabel('Sample index')
    ax2.set_ylabel('RSSI [dBm]')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.show()