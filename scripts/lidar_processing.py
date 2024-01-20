#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


ACCELERATION = -5
K = 300
BOOST = -30


class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('lidar_processing_node', anonymous=True)
        rospy.Subscriber("/project_robot/lidar_main", LaserScan, self.lidar_callback)

        # Publish joint velocity commands for the wheels
        self.right_wheel_controller = rospy.Publisher('/project_robot/right_wheel_controller/command', Float64, queue_size=1)
        self.left_wheel_controller = rospy.Publisher('/project_robot/left_wheel_controller/command', Float64, queue_size=1)

        self.joint_speed = -20.0 
        self.obstacle_distance_threshold = 0.7

    def move_forward(self):
        self.right_wheel_controller.publish(self.joint_speed)
        self.left_wheel_controller.publish(self.joint_speed)

    def move_back(self):
        joint_speed = 4.0  
        self.right_wheel_controller.publish(joint_speed)
        self.left_wheel_controller.publish(joint_speed)

    def turn_left(self,a): 
        rospy.logwarn(f'turning left, {K*a}')
        self.right_wheel_controller.publish(self.joint_speed+ACCELERATION)
        self.left_wheel_controller.publish(-self.joint_speed)

    def turn_right(self,a):
        rospy.logwarn(f'turning right, {K*a}')
        self.right_wheel_controller.publish(-self.joint_speed)
        self.left_wheel_controller.publish(self.joint_speed+a*K*ACCELERATION)

    def lidar_callback(self, data):
        # distance_threshold = 2.0  
        if data.ranges[0] < self.obstacle_distance_threshold:
            self.turn_right(self.obstacle_distance_threshold-data.ranges[0])      
        elif data.ranges[-1] < self.obstacle_distance_threshold:
            self.turn_left(self.obstacle_distance_threshold-data.ranges[-1])
        else:
            if data.ranges[5] > 3:
                self.right_wheel_controller.publish(BOOST)
                self.left_wheel_controller.publish(BOOST)
            else:
                self.move_forward()
            

def main():
    try:
        ObstacleAvoidance()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    rospy.logwarn("TEST_STIRNG!")
    main()