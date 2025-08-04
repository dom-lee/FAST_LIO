import csv
import argparse

import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from nav_msgs.msg import Odometry

# NOTE: change these transformations based on your setup
T_IMU_BASE = np.array(
    [
        0.01817104200588691,
        0.9998192272358706,
        -0.0055969707777272434,
        0.0016790912333181731,
        -0.9998342729010758,
        0.018176980519564845,
        0.001011983976142187,
        -0.00030359519284265605,
        0.001113537065796887,
        0.005577654404657725,
        0.9999838247724537,
        -0.2999951474317361,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
).reshape(4, 4)

T_BASE_LIDAR = np.array(
    [
        -1.0,
        0.0,
        0.0,
        0.12,
        0.0,
        -1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.35118,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
).reshape(4, 4)


class OdomCSVLogger:
    def __init__(self, output_path="odom.csv"):
        self.first_write = True
        self.csv_file = open(output_path, "w", newline="")
        self.writer = None
        rospy.Subscriber("/Odometry", Odometry, self.callback)
        rospy.loginfo("OdomCSVLogger started, saving to %s", output_path)
        rospy.loginfo("T^IMU_BASE:\n%s", T_IMU_BASE)
        rospy.loginfo("T^BASE_LIDAR:\n%s", T_BASE_LIDAR)

    def callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        T_lidar0_imu = np.eye(4)
        T_lidar0_imu[:3, 3] = [p.x, p.y, p.z]
        T_lidar0_imu[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        pose = T_BASE_LIDAR @ T_lidar0_imu @ T_IMU_BASE
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
    parser.add_argument("--output", default="odom.csv", help="Path to output CSV file")
    args = parser.parse_args()

    rospy.init_node("odom_csv_logger", anonymous=True)
    logger = OdomCSVLogger(args.output)
    rospy.on_shutdown(logger.shutdown)
    rospy.spin()
