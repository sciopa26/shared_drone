#colorful prints for the data logged from the flight controller
#!/usr/bin/env python3
import csv
import math
import matplotlib.pyplot as plt
import numpy as np
from utm import utmconv

# Internal path configuration
CSV_PATH = "/home/daniele/Documents/shared_drone/drone_gps_log_and_antenna_daniele/logs/unfilt_good.csv"


def read_telemetry_csv(file_path: str) -> list[dict]:
    """Load timestamp, lat, lon, alt, relative alt, and RSSI values from the CSV."""
    records = []

    with open(file_path, mode='r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # float('NaN') natively converts 'NaN' strings into IEEE 754 float NaNs
            records.append({
                'timestamp': float(row['timestamp_s']),
                'lat': float(row['lat_wgs84_deg']),
                'lon': float(row['lon_wgs84_deg']),
                'alt': float(row['alt_wgs84_m']),
                'rel_alt': float(row.get('rel_alt_m', 'NaN')),
                'local_rssi': float(row['local_rssi_dbm']),
                'remote_rssi': float(row['remote_rssi_dbm']),
            })

    return records


def altitude_over_time(telemetry_data: list[dict], plot = False):
    """Plot WGS84 altitude and (initial altitude + relative altitude) over time."""
    if not telemetry_data:
        print("No telemetry data to plot.")
        return

    t = np.array([d['timestamp'] for d in telemetry_data])
    alt = np.array([d['alt'] for d in telemetry_data])
    rel_alt = np.array([d['rel_alt'] for d in telemetry_data])

    # Use the first valid altitude sample as the t=0 reference
    valid_alt = alt[~np.isnan(alt)]
    if len(valid_alt) == 0:
        print("No valid altitude samples to plot.")
        return

    alt_0 = valid_alt[0]
    reconstructed_alt = alt_0 + rel_alt

    if plot: 
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

        # 1. Direct WGS84 altitude over time
        ax1.plot(t, alt, '-o', color='tab:blue', markersize=3, label='WGS84 Altitude')
        ax1.set_title('Altitude over Time (WGS84)')
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Altitude [m]')
        ax1.legend()
        ax1.grid(True)

        # 2. Initial altitude + relative altitude over time
        ax2.plot(
            t, reconstructed_alt, '-s', color='tab:orange', markersize=3,
            label=f'Alt(t=0) [{alt_0:.2f}m] + Rel Alt'
        )
        ax2.set_title('Initial Altitude + Relative Altitude over Time')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Altitude [m]')
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        plt.show()

    return t, alt, reconstructed_alt



def trajectory_with_rssi(telemetry_data: list[dict], plot = False):
    """Plot the trajectory in UTM coordinates with RSSI values."""
    if not telemetry_data:
        print("No telemetry data to plot.")
        return

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
            (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm(float(row[3]), float(row[4]))
            e.append(e1)
            n.append(n1)
            if counter % 10 == 0:
                e_point.append(e1)
                n_point.append(n1)
            altitude.append(row[2])
        except Exception as ex:
            print(f"Error converting to UTM: {ex}")
            pass
    e = np.array(e) - e[0]
    n = np.array(n) - n[0]
    local_rssi = np.array([float(row[0]) for row in log[:len(e)]])
    remote_rssi = np.array([float(row[1]) for row in log[:len(e)]])
    t_idx = np.arange(len(e))

    if plot:
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

    return e, n, local_rssi, remote_rssi, t_idx



def trajectory_2d(telemetry_data: list[dict], plot = False):
    """Plot the 2D trajectory in UTM coordinates."""
    if not telemetry_data:
        print("No telemetry data to plot.")
        return

    uc = utmconv()
    e = []
    n = []
    for d in telemetry_data:
        if not math.isnan(d['lat']) and not math.isnan(d['lon']):
            (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm(d['lat'], d['lon'])
            e.append(e1)
            n.append(n1)

    e = np.array(e) - e[0]
    n = np.array(n) - n[0]
    t_idx = np.arange(len(e))

    if plot:
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

    return e, n, t_idx


def trajectory_3d(telemetry_data: list[dict], plot = False):
    """Plot the 3D trajectory in UTM coordinates with altitude."""
    if not telemetry_data:
        print("No telemetry data to plot.")
        return

    uc = utmconv()
    e = []
    n = []
    alt = []
    for d in telemetry_data:
        if not math.isnan(d['lat']) and not math.isnan(d['lon']):
            (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm(d['lat'], d['lon'])
            e.append(e1)
            n.append(n1)
            alt.append(d['alt'])

    e = np.array(e) - e[0]
    n = np.array(n) - n[0]
    alt = np.array(alt)
    if plot:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        sc3d = ax.scatter(e, n, alt, c=np.arange(len(e)), cmap='viridis', s=8, label='Trajectory')
        fig.colorbar(sc3d, ax=ax, label='Sample index (Time)')
        ax.plot([e[0]], [n[0]], [alt[0]], 'go', markersize=10, label='Start point')
        ax.plot([e[-1]], [n[-1]], [alt[-1]], 'bo', markersize=5, label='End point')
        ax.set_xlabel('Easting [m]')
        ax.set_ylabel('Northing [m]')
        ax.set_zlabel('Altitude [m]')
        ax.legend()
        ax.grid(True)
        plt.title('3D Trajectory in UTM Coordinates with Altitude')
        plt.show()

    return e, n, alt


def rssi_vs_distance(telemetry_data: list[dict], plot = False):
    """Plot RSSI values against distance from the starting point."""
    if not telemetry_data:
        print("No telemetry data to plot.")
        return

    uc = utmconv()
    e = []
    n = []
    local_rssi = []
    remote_rssi = []

    for d in telemetry_data:
        if not math.isnan(d['lat']) and not math.isnan(d['lon']):
            (hemisphere, zone, letter, e1, n1) = uc.geodetic_to_utm(d['lat'], d['lon'])
            e.append(e1)
            n.append(n1)
            local_rssi.append(d['local_rssi'])
            remote_rssi.append(d['remote_rssi'])

    e = np.array(e) - e[0]
    n = np.array(n) - n[0]
    distances = np.sqrt(e**2 + n**2)
    local_rssi = np.array(local_rssi)
    remote_rssi = np.array(remote_rssi)

    #calculate the average RSSI values for each distance bin
    bin_size = 5  # meters
    max_distance = np.max(distances)
    bins = np.arange(0, max_distance + bin_size, bin_size)
    local_rssi_binned = []
    remote_rssi_binned = []
    bin_centers = []

    for i in range(len(bins) - 1):
        mask = (distances >= bins[i]) & (distances < bins[i + 1])
        if np.any(mask):
            local_rssi_binned.append(np.mean(local_rssi[mask]))
            remote_rssi_binned.append(np.mean(remote_rssi[mask]))
            bin_centers.append((bins[i] + bins[i + 1]) / 2)

    if plot:
        plt.figure(figsize=(10, 5))
        plt.plot(bin_centers, local_rssi_binned, '-o', markersize=3, label='Local RSSI (GCS)')
        plt.plot(bin_centers, remote_rssi_binned, '-s', markersize=3, label='Remote RSSI (UAV)')
        plt.title('RSSI vs Distance from Start Point')
        plt.xlabel('Distance [m]')
        plt.ylabel('RSSI [dBm]')
        plt.legend()
        plt.grid(True)
        plt.show()

    return bin_centers, local_rssi_binned, remote_rssi_binned

if __name__ == '__main__':
    telemetry_data = read_telemetry_csv(CSV_PATH)
    print(f"Loaded {len(telemetry_data)} samples.")
    if telemetry_data:
        print("Sample 0:", telemetry_data[0])
    
    trajectory_2d(telemetry_data, plot=True)
    trajectory_3d(telemetry_data, plot=True)
    trajectory_with_rssi(telemetry_data, plot=True)
    altitude_over_time(telemetry_data, plot=True)
    rssi_vs_distance(telemetry_data, plot=True)