# Launch MAVROS in another shell before running this script:
# ros2 launch mavros px4.launch fcu_url:=/dev/ttyUSB0:57600 gcs_url:=udp://@127.0.0.1:14550
# Logs both EKF-filtered and raw GNSS telemetry with per-row timestamps into two CSV files
# Usage: python3 dual_logger.py filtered_log.csv raw_log.csv
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


class DualTelemetryLogger(Node):
    def __init__(self, filtered_csv_path: str, raw_csv_path: str, sample_rate_hz: float = 1.0):
        super().__init__('dual_telemetry_logger')

        self.filtered_path = filtered_csv_path
        self.raw_path = raw_csv_path

        self.filtered_file = open(self.filtered_path, mode='w', newline='', encoding='utf-8')
        self.raw_file = open(self.raw_path, mode='w', newline='', encoding='utf-8')

        self.filtered_writer = csv.writer(self.filtered_file)
        self.raw_writer = csv.writer(self.raw_file)

        # Include both loop timestamp and ROS message header timestamp on every row
        self.header = [
            'timestamp_s',
            'ros_stamp_s',
            'lat_wgs84_deg',
            'lon_wgs84_deg',
            'alt_wgs84_m',
            'h_acc_m',
            'v_acc_m',
            'local_rssi_dbm',
            'remote_rssi_dbm',
        ]
        self.filtered_writer.writerow(self.header)
        self.raw_writer.writerow(self.header)
        self.filtered_file.flush()
        self.raw_file.flush()

        self.start_time = time.time()
        self.fcu_connected = False
        self._reset_sample_buffer()

        # MAVROS connection state
        self.sub_state = self.create_subscription(
            State, '/mavros/state', self.on_state_msg, qos_profile_sensor_data
        )

        # Filtered global position (EKF estimate in WGS84)
        self.sub_global_pos = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.on_global_pos_msg,
            qos_profile_sensor_data,
        )

        # Unfiltered global position (raw GNSS fix in WGS84)
        self.sub_global_pos_raw = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.on_global_pos_raw_msg,
            qos_profile_sensor_data,
        )

        # Raw GNSS message (GPS_RAW_INT) used as fallback
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
        self.get_logger().info(
            f"Logging filtered -> '{self.filtered_path}' and raw -> '{self.raw_path}'. "
            "Type 'save' and press Enter to stop."
        )

    def _reset_sample_buffer(self):
        """Reset current cycle values to NaN so missing readings never shift columns."""
        # Filtered buffer
        self.filt_stamp = math.nan
        self.filt_lat = math.nan
        self.filt_lon = math.nan
        self.filt_alt = math.nan
        self.filt_h_acc = math.nan
        self.filt_v_acc = math.nan

        # Raw buffer
        self.raw_stamp = math.nan
        self.raw_lat = math.nan
        self.raw_lon = math.nan
        self.raw_alt = math.nan
        self.raw_h_acc = math.nan
        self.raw_v_acc = math.nan

        # Shared radio metrics
        self.local_rssi = math.nan
        self.remote_rssi = math.nan

    @staticmethod
    def _fmt(val: float, decimals: int = 6) -> str:
        return 'NaN' if math.isnan(val) else f'{val:.{decimals}f}'

    @staticmethod
    def _stamp_to_sec(msg) -> float:
        """Convert ROS 2 builtin_interfaces/Time header stamp to floating-point seconds."""
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    @staticmethod
    def _extract_cov_accuracy(msg: NavSatFix) -> tuple[float, float]:
        """Compute 1-sigma horizontal and vertical accuracy from NavSatFix covariance."""
        h_acc, v_acc = math.nan, math.nan
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            cov_e = msg.position_covariance[0]
            cov_n = msg.position_covariance[4]
            cov_u = msg.position_covariance[8]
            if cov_e >= 0.0 and cov_n >= 0.0:
                h_acc = math.sqrt(cov_e + cov_n)
            if cov_u >= 0.0:
                v_acc = math.sqrt(cov_u)
        return h_acc, v_acc

    def on_state_msg(self, msg: State):
        self.fcu_connected = msg.connected

    def on_global_pos_msg(self, msg: NavSatFix):
        if msg.status.status >= 0:
            self.filt_stamp = self._stamp_to_sec(msg)
            self.filt_lat = msg.latitude
            self.filt_lon = msg.longitude
            self.filt_alt = msg.altitude
            self.filt_h_acc, self.filt_v_acc = self._extract_cov_accuracy(msg)

    def on_global_pos_raw_msg(self, msg: NavSatFix):
        if msg.status.status >= 0:
            self.raw_stamp = self._stamp_to_sec(msg)
            self.raw_lat = msg.latitude
            self.raw_lon = msg.longitude
            self.raw_alt = msg.altitude
            self.raw_h_acc, self.raw_v_acc = self._extract_cov_accuracy(msg)

    def on_gps_raw_msg(self, msg: GPSRAW):
        # Fallback coordinates for raw stream if /mavros/global_position/raw/fix is missing
        if math.isnan(self.raw_lat) and msg.fix_type >= 2:
            self.raw_stamp = self._stamp_to_sec(msg)
            self.raw_lat = msg.lat / 1e7
            self.raw_lon = msg.lon / 1e7
            self.raw_alt = msg.alt / 1e3

        # Fallback accuracy from eph/epv (cm -> m)
        if msg.eph != 65535:
            eph_m = msg.eph / 100.0
            if math.isnan(self.raw_h_acc):
                self.raw_h_acc = eph_m
            if math.isnan(self.filt_h_acc):
                self.filt_h_acc = eph_m

        if msg.epv != 65535:
            epv_m = msg.epv / 100.0
            if math.isnan(self.raw_v_acc):
                self.raw_v_acc = epv_m
            if math.isnan(self.filt_v_acc):
                self.filt_v_acc = epv_m

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
        elapsed = f'{time.time() - self.start_time:.2f}'
        rssi_cols = [self._fmt(self.local_rssi, 1), self._fmt(self.remote_rssi, 1)]

        filt_row = [
            elapsed,
            self._fmt(self.filt_stamp, 6),
            self._fmt(self.filt_lat, 7),
            self._fmt(self.filt_lon, 7),
            self._fmt(self.filt_alt, 3),
            self._fmt(self.filt_h_acc, 3),
            self._fmt(self.filt_v_acc, 3),
            *rssi_cols,
        ]

        raw_row = [
            elapsed,
            self._fmt(self.raw_stamp, 6),
            self._fmt(self.raw_lat, 7),
            self._fmt(self.raw_lon, 7),
            self._fmt(self.raw_alt, 3),
            self._fmt(self.raw_h_acc, 3),
            self._fmt(self.raw_v_acc, 3),
            *rssi_cols,
        ]

        self.filtered_writer.writerow(filt_row)
        self.raw_writer.writerow(raw_row)
        self.filtered_file.flush()
        self.raw_file.flush()

        conn_tag = 'OK' if self.fcu_connected else 'NO_FCU'
        print(
            f'[{conn_tag}] t={elapsed}s | '
            f'FILT: ({filt_row[2]}, {filt_row[3]}, {filt_row[4]}m) | '
            f'RAW: ({raw_row[2]}, {raw_row[3]}, {raw_row[4]}m) | '
            f'RSSI(L/R): {rssi_cols[0]}/{rssi_cols[1]} dBm'
        )

        self._reset_sample_buffer()

    def close(self):
        for f in (self.filtered_file, self.raw_file):
            if f and not f.closed:
                f.flush()
                f.close()


def stdin_watcher(stop_event: threading.Event):
    """Wait for the 'save' command on stdin to trigger a clean shutdown."""
    for line in sys.stdin:
        if line.strip().lower() == 'save':
            stop_event.set()
            break


def main():
    parser = argparse.ArgumentParser(
        description='Log filtered and raw MAVROS WGS84 GPS plus Radio RSSI to two CSV files.'
    )
    parser.add_argument('filtered_csv', help='Output CSV file path for EKF-filtered data')
    parser.add_argument('raw_csv', help='Output CSV file path for raw GNSS data')
    parser.add_argument('--rate', type=float, default=1.0, help='Logging frequency in Hz')
    args = parser.parse_args()

    stop_event = threading.Event()
    input_thread = threading.Thread(target=stdin_watcher, args=(stop_event,), daemon=True)
    input_thread.start()

    rclpy.init()
    node = DualTelemetryLogger(
        filtered_csv_path=args.filtered_csv,
        raw_csv_path=args.raw_csv,
        sample_rate_hz=args.rate,
    )

    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"\nSaving and closing '{args.filtered_csv}' and '{args.raw_csv}'...")
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()