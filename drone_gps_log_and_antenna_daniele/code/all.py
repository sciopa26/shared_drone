#reads all data (in theory) from the flight controller without filtering with EKF
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy, QoSProfile
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import GPSRAW, RadioStatus, State
from sensor_msgs.msg import NavSatFix


class TelemetryMonitor(Node):
    def __init__(self):
        super().__init__('telemetry_monitor')

        self.fcu_connected = False
        self.received_any_data = False

        # Check MAVROS connection state to the Pixhawk
        self.sub_state = self.create_subscription(
            State,
            '/mavros/state',
            self.on_state_msg,
            qos_profile_sensor_data
        )

        # Subscribe directly to MAVROS radio status (preferred over diagnostics)
        self.sub_radio_status = self.create_subscription(
            RadioStatus,
            '/mavros/radio_status',
            self.on_radio_status_msg,
            qos_profile_sensor_data
        )

        # Fallback subscription to /diagnostics with RELIABLE QoS
        self.sub_diagnostics = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.on_diagnostics_msg,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Subscribe to fused global position (requires valid GPS fix)
        self.sub_global_pos = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.on_global_pos_msg,
            qos_profile_sensor_data
        )

        # Subscribe to raw GNSS status (publishes even without a 3D fix)
        self.sub_gps_raw = self.create_subscription(
            GPSRAW,
            '/mavros/gpsstatus/gps1/raw',
            self.on_gps_raw_msg,
            qos_profile_sensor_data
        )

        # Periodic status check
        self.timer = self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('Telemetry monitor node started...')

    def on_state_msg(self, msg: State):
        self.fcu_connected = msg.connected

    def on_radio_status_msg(self, msg: RadioStatus):
        self.received_any_data = True
        local_dbm = getattr(msg, 'rssi_dbm', None)
        remote_dbm = getattr(msg, 'remrssi_dbm', None)

        if local_dbm is not None and remote_dbm is not None:
            self.get_logger().info(
                f'[Radio] Local RSSI: {local_dbm:.1f} dBm | Remote RSSI: {remote_dbm:.1f} dBm'
            )
        else:
            self.get_logger().info(
                f'[Radio] Raw RSSI: {msg.rssi} | Remote Raw RSSI: {msg.remrssi}'
            )

    def on_diagnostics_msg(self, msg: DiagnosticArray):
        for status in msg.status:
            # Case-insensitive match to handle different MAVROS naming conventions
            if 'radio' in status.name.lower():
                self.received_any_data = True
                rssi_vals = {v.key: v.value for v in status.values if 'RSSI' in v.key}
                if rssi_vals:
                    self.get_logger().info(f'[Diagnostics - {status.name}] {rssi_vals}')

    def on_global_pos_msg(self, msg: NavSatFix):
        self.received_any_data = True
        self.get_logger().info(
            f'[GPS Fused] Lat: {msg.latitude:.6f}, Lon: {msg.longitude:.6f}, Alt: {msg.altitude:.2f} m'
        )

    def on_gps_raw_msg(self, msg: GPSRAW):
        self.received_any_data = True
        fix_status = 'No fix' if msg.fix_type < 2 else f'Fix type {msg.fix_type}'
        self.get_logger().info(
            f'[GPS Raw] {fix_status} | Sats: {msg.satellites_visible} | '
            f'Lat: {msg.lat / 1e7:.6f}, Lon: {msg.lon / 1e7:.6f}, Alt: {msg.alt / 1e3:.2f} m'
        )

    def timer_callback(self):
        if not self.fcu_connected:
            self.get_logger().warn('Waiting for FCU connection on /mavros/state (check serial port/baudrate)...')
        elif not self.received_any_data:
            self.get_logger().warn('FCU connected, but no GPS/Radio messages received yet (check MAVLink stream rate).')


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()