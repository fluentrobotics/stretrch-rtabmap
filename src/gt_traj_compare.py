#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
from nav_msgs.msg import Path

def extract_trajectory(bagfile, topic):
    bag = rosbag.Bag(bagfile)
    x = []
    y = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        for pose in msg.poses:
            x.append(pose.pose.position.x)
            y.append(pose.pose.position.y)
    bag.close()
    return x, y

def plot_trajectories(gt_path, rtabmap_path):
    x_gt, y_gt = extract_trajectory(gt_path, '/vicon_path')
    x_rtab, y_rtab = extract_trajectory(rtabmap_path, '/rtabmap/mapPath')

    plt.figure()
    plt.plot(x_gt, y_gt, label='Ground Truth Path', color='blue')
    plt.plot(x_rtab, y_rtab, label='RTAB-Map Path', color='red')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Trajectory Comparison')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    bagfile = 'path_to_your_bagfile.bag'  # Replace with your bag file path
    plot_trajectories(bagfile, bagfile)
