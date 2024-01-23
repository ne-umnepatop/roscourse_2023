#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

add_vel = 6 # no more daddy please

ns = 'project_robot_368201'

def round_int(x):
    if x in [float('-inf'),float('inf')]: return 3
    else: return int(round(x))


class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('lidar_processing_node', anonymous=True)
        rospy.Subscriber(f'/{ns}/lidar_main',
                         LaserScan, self.lidar_callback)

        # Publish joint velocity commands for the wheels
        self.right_wheel_controller = rospy.Publisher(
            f'/{ns}/right_wheel_controller/command', Float64, queue_size=1)
        self.left_wheel_controller = rospy.Publisher(
            f'/{ns}/left_wheel_controller/command', Float64, queue_size=1)

        self.joint_speed = 25.0 
        self.obstacle_distance_threshold = 0.8

    def move_forward(self, speed):
        self.right_wheel_controller.publish(-speed)
        self.left_wheel_controller.publish(-speed)

    def move_back(self):
        joint_speed = 4.0
        self.right_wheel_controller.publish(joint_speed)
        self.left_wheel_controller.publish(joint_speed)

    def turn_left(self, dist):
        self.right_wheel_controller.publish(-self.joint_speed-add_vel)
        self.left_wheel_controller.publish(self.joint_speed)

    def turn_right(self, dist):
        self.right_wheel_controller.publish(self.joint_speed)
        self.left_wheel_controller.publish(-self.joint_speed-add_vel)

    def lidar_callback(self, data):
        rospy.logwarn(
            f'dist: left:{data.ranges[-1]} mid:{data.ranges[4]} right:{data.ranges[0]}')

        if abs(data.ranges[4]) > 3 and round_int(data.ranges[0]) >= 1 and round_int(data.ranges[-1]) >= 1:
            self.move_forward(35)
        elif data.ranges[0] < self.obstacle_distance_threshold:
            self.turn_right(data.ranges[0])
        elif data.ranges[-1] < self.obstacle_distance_threshold:
            self.turn_left(data.ranges[-1])
        elif sum(data.ranges) < self.obstacle_distance_threshold*9:
            self.move_back()
        else:
            self.move_forward(self.joint_speed)


def main():
    try:
        ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
