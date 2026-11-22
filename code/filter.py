import csv
from datetime import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry


class EkfLoggerNode(Node):
    def __init__(self):
        super().__init__('ekf_odometry_logger')

        # 1. Configurazione QoS obbligatoria per comunicare con PX4 uXRCE-DDS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 2. Creazione file CSV con timestamp nel nome
        filename = f"px4_ekf_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp_ros_s',
            'timestamp_px4_us',
            'pos_x_m', 'pos_y_m', 'pos_z_m',
            'vel_x_ms', 'vel_y_ms', 'vel_z_ms'
        ])
        self.get_logger().info(f"Registrazione avviata su: {filename}")

        # 3. Sottoscrizione al topic dell'EKF2
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )

        self.msg_count = 0

    def odometry_callback(self, msg: VehicleOdometry):
        # Estrazione posizione [x, y, z] e velocità [vx, vy, vz] nel frame NED
        px, py, pz = float(msg.position[0]), float(msg.position[1]), float(msg.position[2])
        vx, vy, vz = float(msg.velocity[0]), float(msg.velocity[1]), float(msg.velocity[2])
        ros_time = self.get_clock().now().nanoseconds / 1e9

        # Scrittura su CSV (ad ogni campione ricevuto)
        self.csv_writer.writerow([
            f"{ros_time:.6f}",
            msg.timestamp,
            f"{px:.4f}", f"{py:.4f}", f"{pz:.4f}",
            f"{vx:.4f}", f"{vy:.4f}", f"{vz:.4f}"
        ])

        # Stampa a schermo limitata (ogni 10 messaggi) per non intasare il terminale
        self.msg_count += 1
        if self.msg_count % 10 == 0:
            self.get_logger().info(
                f"Pos (NED) [m]: X={px:6.2f}, Y={py:6.2f}, Z={pz:6.2f} | "
                f"Vel [m/s]: Vx={vx:5.2f}, Vy={vy:5.2f}, Vz={vz:5.2f}"
            )

    def close_file(self):
        self.csv_file.flush()
        self.csv_file.close()
        self.get_logger().info("File CSV salvato e chiuso correttamente.")


def main(args=None):
    rclpy.init(args=args)
    node = EkfLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_file()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()