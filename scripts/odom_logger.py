import csv
import argparse

import rospy
from nav_msgs.msg import Odometry


class OdomCSVLogger:
    def __init__(self, odom_topic="/Odometry", output_path="odom.csv"):
        self.output_path = output_path
        self.first_write = True
        self.csv_file = open(self.output_path, "w", newline="")
        self.writer = None
        rospy.Subscriber(odom_topic, Odometry, self.callback)
        rospy.loginfo("OdomCSVLogger started, saving to %s", self.output_path)

    def callback(self, msg):
        data = {
            "timestamp": msg.header.stamp.to_sec(),
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "z": msg.pose.pose.position.z,
            "qx": msg.pose.pose.orientation.x,
            "qy": msg.pose.pose.orientation.y,
            "qz": msg.pose.pose.orientation.z,
            "qw": msg.pose.pose.orientation.w,
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
    parser.add_argument("--odom_topic", default="/Odometry")
    args = parser.parse_args()

    rospy.init_node("odom_csv_logger", anonymous=True)
    logger = OdomCSVLogger(odom_topic=args.odom_topic, output_path=args.output)
    rospy.on_shutdown(logger.shutdown)
    rospy.spin()
