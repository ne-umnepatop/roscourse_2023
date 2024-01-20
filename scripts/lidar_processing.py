#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


# Прикрутить Линейное управление скоростью? ?

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('lidar_processing_node', anonymous=True)
        rospy.Subscriber("/project_robot/lidar_main", LaserScan, self.lidar_callback)

        # Publish joint velocity commands for the wheels
        self.right_wheel_controller = rospy.Publisher('/project_robot/right_wheel_controller/command', Float64, queue_size=1)
        self.left_wheel_controller = rospy.Publisher('/project_robot/left_wheel_controller/command', Float64, queue_size=1)

        self.max_linear_speed = 0.5  # max linear speed
        self.max_angular_speed = 1.0  # max angular speed
        self.joint_speed = 1.0 
        self.obstacle_distance_threshold = 0.5

    def move_forward(self):
        joint_speed = 10.0  
        self.right_wheel_controller.publish(-joint_speed)
        self.left_wheel_controller.publish(-joint_speed)

    def turn_left(self):
        joint_speed = 10.0  
        self.right_wheel_controller.publish(joint_speed)
        self.left_wheel_controller.publish(-joint_speed)

    def turn_right(self):
        joint_speed = 10.0  
        self.right_wheel_controller.publish(-joint_speed)
        self.left_wheel_controller.publish(joint_speed)

    def is_there_a_wall(self,data):
        rasstoyanie = (data.ranges[0]+data.ranges[1])/2
        


    def lidar_callback(self, data):
        distance_threshold = 2.0  
        if data.ranges[0] < distance_threshold:
            rospy.loginfo("Object detected in laser zone!")
            self.turn_left()
        else:
            rospy.loginfo("No object detected in laser zone.")
            self.move_forward()
            





def main():
    try:
        ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()