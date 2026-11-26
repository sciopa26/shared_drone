from read_ros_log import read_telemetry_csv, rssi_vs_distance, trajectory_with_rssi
import numpy as np
import matplotlib.pyplot as plt


CSV_PATH_good = "/home/daniele/Documents/shared_drone/drone_gps_log_and_antenna_daniele/code/filt_good_run.csv"
CSV_PATH_bad = "/home/daniele/Documents/shared_drone/drone_gps_log_and_antenna_daniele/code/filt_bad_run.csv"

telemetry_data_good = read_telemetry_csv(CSV_PATH_good)
telemetry_data_bad = read_telemetry_csv(CSV_PATH_bad)

[bin_centers_good, local_rssi_binned_good, remote_rssi_binned_good] = rssi_vs_distance(telemetry_data_good)
[bin_centers_bad, local_rssi_binned_bad, remote_rssi_binned_bad] = rssi_vs_distance(telemetry_data_bad)
#normalization of the RSSI values to compare the two runs on the same first and last distance bins

min_distance = 10
max_distance = 400
min_idx_good = np.searchsorted(bin_centers_good, min_distance)
max_idx_good = np.searchsorted(bin_centers_good, max_distance)
min_idx_bad = np.searchsorted(bin_centers_bad, min_distance)
max_idx_bad = np.searchsorted(bin_centers_bad, max_distance)
local_rssi_binned_good = local_rssi_binned_good[min_idx_good:max_idx_good]
local_rssi_binned_bad = local_rssi_binned_bad[min_idx_bad:max_idx_bad]
plt.plot(bin_centers_good[min_idx_good:max_idx_good], local_rssi_binned_good, '-o',linewidth=2, markersize=3, label='Local RSSI (GCS) - Handmade Antenna')
plt.plot(bin_centers_bad[min_idx_bad:max_idx_bad], local_rssi_binned_bad, '-s',linewidth=2, markersize=3, label='Local RSSI (GCS) - Stock Antenna')
plt.title('RSSI vs Distance from Start Point (Handmade vs Stock Antenna)')
plt.xlabel('Distance [m]')
plt.ylabel('RSSI [dBm]')
plt.legend()
plt.grid(True)
plt.show()

delta = np.array(local_rssi_binned_good) - np.array(local_rssi_binned_bad)
mean_delta = np.mean(delta)
plt.plot(bin_centers_good[min_idx_good:max_idx_good], delta, '-o', linewidth=2, markersize=3, label='Delta RSSI (Handmade antenna - Stock antenna)')
plt.axhline(y=mean_delta, color='r', linestyle='--', label=f'Mean Delta: {mean_delta:.2f} dBm')
plt.title('Delta RSSI vs Distance from Start Point (Handmade vs Stock Antenna)')
plt.xlabel('Distance [m]')
plt.ylabel('Delta RSSI [dBm]')
plt.legend()
plt.grid(True)
plt.show()

[e, n, local_rssi, remote_rssi, t_idx] =trajectory_with_rssi(telemetry_data_good)
plt.scatter(e, n, c=local_rssi, cmap='viridis', marker='o', s=10)
plt.colorbar(label='Local RSSI [dBm]')
plt.title('Trajectory with Local RSSI (Handmade Antenna)')
plt.xlabel('East [m]')
plt.ylabel('North [m]')
plt.grid(True)
plt.show()