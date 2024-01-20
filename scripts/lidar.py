#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


# Прикрутить Линейное управление скоростью? ?
LIMIT = 0.1
ACSSELERATION = 0.5


class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('lidar_processing_node', anonymous=True)
        rospy.Subscriber("/project_robot/lidar_main",
                         LaserScan, self.lidar_callback)

        # Publish joint velocity commands for the wheels
        self.right_wheel_controller = rospy.Publisher(
            '/project_robot/right_wheel_controller/command', Float64, queue_size=1)
        self.left_wheel_controller = rospy.Publisher(
            '/project_robot/left_wheel_controller/command', Float64, queue_size=1)

        self.max_linear_speed = 0.5  # max linear speed
        self.max_angular_speed = 1.0  # max angular speed
        self.joint_speed = 1.0
        self.obstacle_distance_threshold = 0.5

    def move_forward(self):
        joint_speed = 10.0
        self.right_wheel_controller.publish(joint_speed)
        self.left_wheel_controller.publish(joint_speed)

    def turn_left(self):
        joint_speed = 10.0
        self.right_wheel_controller.publish(joint_speed+ACSSELERATION)
        self.left_wheel_controller.publish(joint_speed)

    def turn_right(self):
        joint_speed = 10.0
        self.right_wheel_controller.publish(-joint_speed)
        self.left_wheel_controller.publish(joint_speed+ACSSELERATION)

    def is_wall(self, data):
        # подразумеваю data.ranges[0] за левое а data.ranges[1] за растсояние от правой стены
        rasstoyanie = (data.ranges[0]+data.ranges[1])/2
        # расстояние от левой стены проверяю
        if not (rasstoyanie-LIMIT < data.ranges[0] < rasstoyanie+LIMIT):
            if rasstoyanie-LIMIT < data.ranges[0]:
                self.turn_right()
            else:
                self.turn_left()
        else:
            self.move_forward()

    def lidar_callback(self, data):
        self.is_wall(data)


def main():
    try:
        ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
