import csv
import argparse
import json

import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from nav_msgs.msg import Odometry


def load_transform_matrix(config_file, matrix_name):
    """Load transformation matrix from config file or use default."""
    try:
        with open(config_file, "r") as f:
            config = json.load(f)
            if matrix_name in config:
                matrix_data = config[matrix_name]
                if isinstance(matrix_data, list) and len(matrix_data) == 16:
                    return np.array(matrix_data).reshape(4, 4)
                else:
                    rospy.logwarn(
                        "Invalid matrix format for %s, using default", matrix_name
                    )
            else:
                rospy.logwarn(
                    "Matrix %s not found in config, using default", matrix_name
                )
    except (FileNotFoundError, json.JSONDecodeError) as e:
        rospy.logwarn("Config file error: %s, using defaults", e)

    return np.eye(4)


class OdomCSVLogger:
    def __init__(self, output_path="odom.csv", tf_config=None):
        self.first_write = True
        self.csv_file = open(output_path, "w", newline="")
        self.writer = None

        # Load transformation matrices
        self.T_IMU_BASE = np.eye(4)
        self.T_BASE_LIDAR = np.eye(4)

        if tf_config:
            self.T_IMU_BASE = load_transform_matrix(tf_config, "T_IMU_BASE")
            self.T_BASE_LIDAR = load_transform_matrix(tf_config, "T_BASE_LIDAR")

        rospy.Subscriber("/Odometry", Odometry, self.callback)
        rospy.loginfo("OdomCSVLogger started, saving to %s", output_path)
        rospy.loginfo("T^IMU_BASE:\n%s", self.T_IMU_BASE)
        rospy.loginfo("T^BASE_LIDAR:\n%s", self.T_BASE_LIDAR)

    def callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        T_lidar0_imu = np.eye(4)
        T_lidar0_imu[:3, 3] = [p.x, p.y, p.z]
        T_lidar0_imu[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        pose = self.T_BASE_LIDAR @ T_lidar0_imu @ self.T_IMU_BASE
        quat = R.from_matrix(pose[:3, :3]).as_quat()

        data = {
            "timestamp": msg.header.stamp.to_sec(),
            "x": pose[0, 3],
            "y": pose[1, 3],
            "z": pose[2, 3],
            "qx": quat[0],
            "qy": quat[1],
            "qz": quat[2],
            "qw": quat[3],
        }

        # Write header once
        if self.first_write:
            self.writer = csv.DictWriter(self.csv_file, fieldnames=data.keys())
            self.writer.writeheader()
            self.first_write = False

        # Write data row
        formatted = {k: f"{v:.8f}" for k, v in data.items()}
        self.writer.writerow(formatted)

    def shutdown(self):
        rospy.loginfo("Shutting down OdomCSVLogger")
        self.csv_file.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", type=str, default="odom.csv", help="Path to output CSV file"
    )
    parser.add_argument(
        "--config",
        type=str,
        help="Path to JSON config file with transformation matrices",
    )
    args = parser.parse_args()

    rospy.init_node("odom_csv_logger", anonymous=True)
    logger = OdomCSVLogger(args.output, args.config)
    rospy.on_shutdown(logger.shutdown)
    rospy.spin()
