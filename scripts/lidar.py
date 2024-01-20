#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


# Прикрутить Линейное управление скоростью? ?
LIMIT = 0.5
ACSSELERATION = -5
JOINT_SPEED = -10.0

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
        self.JOINT_SPEED = 1.0
        self.obstacle_distance_threshold = 0.5

    def move_forward(self):
        
        self.right_wheel_controller.publish(JOINT_SPEED)
        self.left_wheel_controller.publish(JOINT_SPEED)

    def turn_left(self):
        
        self.right_wheel_controller.publish(JOINT_SPEED+ACSSELERATION)
        self.left_wheel_controller.publish(JOINT_SPEED)

    def turn_right(self):
        
        self.right_wheel_controller.publish(-JOINT_SPEED)
        self.left_wheel_controller.publish(JOINT_SPEED+ACSSELERATION)

    def is_wall(self, data):
        # подразумеваю data.ranges[0] за левое а data.ranges[1] за растсояние от правой стены
        from_right_wall = data.ranges[0]
        from_left_wall = data.ranges[-1]
        if from_right_wall >= 2:
            from_right_wall = 2
        if from_left_wall >= 2:
            from_left_wall = 2
        rasstoyanie = (from_right_wall+from_left_wall)/2
        # расстояние от левой стены проверяю

        
        if LIMIT > from_left_wall:
            self.turn_right()
        elif LIMIT > from_right_wall:
            self.turn_left()
        else:
            self.move_forward()

    def lidar_callback(self, data):
        self.is_wall(data)
        self.move_forward()


def main():
    try:
        ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
