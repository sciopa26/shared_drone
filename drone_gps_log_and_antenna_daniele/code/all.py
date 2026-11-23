# Launch MAVROS in another shell before running this script:
# ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:57600 gcs_url:=udp://@127.0.0.1:14550
# Reads raw (unfiltered) GNSS data to evaluate EKF filtering performance
# Usage: python3 all_raw.py name_log.csv
#!/usr/bin/env python3
import argparse
import csv
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy, QoSProfile
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import GPSRAW, RadioStatus, State
from sensor_msgs.msg import NavSatFix


class RawTelemetryLogger(Node):
    def __init__(self, csv_path: str, sample_rate_hz: float = 1.0):
        super().__init__('raw_telemetry_logger')

        self.csv_path = csv_path
        self.csv_file = open(self.csv_path, mode='w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)

        # Keep identical header to the filtered logger for direct comparison
        self.header = [
            'timestamp_s',
            'lat_wgs84_deg',
            'lon_wgs84_deg',
            'alt_wgs84_m',
            'h_acc_m',
            'v_acc_m',
            'local_rssi_dbm',
            'remote_rssi_dbm',
        ]
        self.csv_writer.writerow(self.header)
        self.csv_file.flush()

        self.start_time = time.time()
        self.fcu_connected = False
        self._reset_sample_buffer()

        # MAVROS connection state
        self.sub_state = self.create_subscription(
            State, '/mavros/state', self.on_state_msg, qos_profile_sensor_data
        )

        # Unfiltered global position (raw GNSS fix in WGS84 coordinates, pre-EKF)
        self.sub_global_pos_raw = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.on_global_pos_raw_msg,
            qos_profile_sensor_data,
        )

        # Raw GNSS message (GPS_RAW_INT) used as fallback for accuracy and coordinates
        self.sub_gps_raw = self.create_subscription(
            GPSRAW,
            '/mavros/gpsstatus/gps1/raw',
            self.on_gps_raw_msg,
            qos_profile_sensor_data,
        )

        # Radio status topic for signal strength
        self.sub_radio_status = self.create_subscription(
            RadioStatus,
            '/mavros/radio_status',
            self.on_radio_status_msg,
            qos_profile_sensor_data,
        )

        # Diagnostics fallback for RSSI in dBm
        self.sub_diagnostics = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.on_diagnostics_msg,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )

        self.timer = self.create_timer(1.0 / sample_rate_hz, self.log_sample)
        self.get_logger().info(f"Logging RAW GPS to '{self.csv_path}'. Type 'save' and press Enter to stop.")

    def _reset_sample_buffer(self):
        """Reset current cycle values to NaN so missing readings never shift columns."""
        self.lat = math.nan
        self.lon = math.nan
        self.alt = math.nan
        self.h_acc = math.nan
        self.v_acc = math.nan
        self.local_rssi = math.nan
        self.remote_rssi = math.nan

    @staticmethod
    def _fmt(val: float, decimals: int = 6) -> str:
        return 'NaN' if math.isnan(val) else f'{val:.{decimals}f}'

    def on_state_msg(self, msg: State):
        self.fcu_connected = msg.connected

    def on_global_pos_raw_msg(self, msg: NavSatFix):
        # NavSatFix from /mavros/global_position/raw/fix is unfiltered GNSS in WGS84
        if msg.status.status >= 0:
            self.lat = msg.latitude
            self.lon = msg.longitude
            self.alt = msg.altitude

            # Extract horizontal and vertical accuracy from raw GNSS covariance
            if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
                cov_e = msg.position_covariance[0]
                cov_n = msg.position_covariance[4]
                cov_u = msg.position_covariance[8]
                if cov_e >= 0.0 and cov_n >= 0.0:
                    self.h_acc = math.sqrt(cov_e + cov_n)
                if cov_u >= 0.0:
                    self.v_acc = math.sqrt(cov_u)

    def on_gps_raw_msg(self, msg: GPSRAW):
        # Fallback if /mavros/global_position/raw/fix is not available
        if math.isnan(self.lat) and msg.fix_type >= 2:
            self.lat = msg.lat / 1e7
            self.lon = msg.lon / 1e7
            self.alt = msg.alt / 1e3  # Note: GPSRAW.alt is AMSL (mm)

        # Fallback accuracy from raw GNSS eph/epv (reported in cm, 65535 = unknown)
        if math.isnan(self.h_acc) and msg.eph != 65535:
            self.h_acc = msg.eph / 100.0
        if math.isnan(self.v_acc) and msg.epv != 65535:
            self.v_acc = msg.epv / 100.0

    def on_radio_status_msg(self, msg: RadioStatus):
        local_dbm = getattr(msg, 'rssi_dbm', math.nan)
        remote_dbm = getattr(msg, 'remrssi_dbm', math.nan)
        if not math.isnan(local_dbm) and local_dbm != 0.0:
            self.local_rssi = float(local_dbm)
        if not math.isnan(remote_dbm) and remote_dbm != 0.0:
            self.remote_rssi = float(remote_dbm)

    def on_diagnostics_msg(self, msg: DiagnosticArray):
        for status in msg.status:
            if 'radio' in status.name.lower():
                for item in status.values:
                    try:
                        if item.key == 'RSSI (dBm)':
                            self.local_rssi = float(item.value)
                        elif item.key == 'Remote RSSI (dBm)':
                            self.remote_rssi = float(item.value)
                    except ValueError:
                        pass

    def log_sample(self):
        elapsed = time.time() - self.start_time

        row = [
            f'{elapsed:.2f}',
            self._fmt(self.lat, 7),
            self._fmt(self.lon, 7),
            self._fmt(self.alt, 3),
            self._fmt(self.h_acc, 3),
            self._fmt(self.v_acc, 3),
            self._fmt(self.local_rssi, 1),
            self._fmt(self.remote_rssi, 1),
        ]

        self.csv_writer.writerow(row)
        self.csv_file.flush()

        conn_tag = 'OK' if self.fcu_connected else 'NO_FCU'
        print(
            f'[{conn_tag} RAW] t={row[0]}s | '
            f'WGS84: ({row[1]}, {row[2]}, {row[3]}m) | '
            f'Acc(H/V): {row[4]}m / {row[5]}m | '
            f'RSSI(L/R): {row[6]} / {row[7]} dBm'
        )

        # Clear buffer after each logged row
        self._reset_sample_buffer()

    def close(self):
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()


def stdin_watcher(stop_event: threading.Event):
    """Wait for the 'save' command on stdin to trigger a clean shutdown."""
    for line in sys.stdin:
        if line.strip().lower() == 'save':
            stop_event.set()
            break


def main():
    parser = argparse.ArgumentParser(description='Log raw MAVROS WGS84 GPS and Radio RSSI to CSV.')
    parser.add_argument('csv_file', help='Output CSV file path')
    parser.add_argument('--rate', type=float, default=1.0, help='Logging frequency in Hz')
    args = parser.parse_args()

    stop_event = threading.Event()
    input_thread = threading.Thread(target=stdin_watcher, args=(stop_event,), daemon=True)
    input_thread.start()

    rclpy.init()
    node = RawTelemetryLogger(csv_path=args.csv_file, sample_rate_hz=args.rate)

    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\nSaving and closing '{args.csv_file}'...")
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()