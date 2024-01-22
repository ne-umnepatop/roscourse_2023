#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

BASIC_SPEED = -21
ACCELERATION = -5
K = 300
BOOST = -35
THRESHOLD_DISTANCE = 0.72



class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('lidar_processing_node', anonymous=True)
        rospy.Subscriber("/project_robot/lidar_main", LaserScan, self.lidar_callback)

        # Publish joint velocity commands for the wheels
        self.right_wheel_controller = rospy.Publisher('/project_robot/right_wheel_controller/command', Float64, queue_size=1)
        self.left_wheel_controller = rospy.Publisher('/project_robot/left_wheel_controller/command', Float64, queue_size=1)

        BASIC_SPEED = -20.0 

    def move_forward(self):
        self.right_wheel_controller.publish(BASIC_SPEED)
        self.left_wheel_controller.publish(BASIC_SPEED)

    def move_back(self):
        backward_joint_speed = 4.0
        self.right_wheel_controller.publish(backward_joint_speed)
        self.left_wheel_controller.publish(backward_joint_speed)

    def turn_left(self,a): 
        rospy.logwarn(f'turning left, {K*a}')
        self.right_wheel_controller.publish(BASIC_SPEED+ACCELERATION)
        self.left_wheel_controller.publish(-BASIC_SPEED)

    def turn_right(self,a):
        rospy.logwarn(f'turning right, {K*a}')
        self.right_wheel_controller.publish(-BASIC_SPEED)
        self.left_wheel_controller.publish(BASIC_SPEED+a*K*ACCELERATION)

    def lidar_callback(self, data):
        # distance_threshold = 2.0  
        if data.ranges[0] < THRESHOLD_DISTANCE:
            self.turn_right(THRESHOLD_DISTANCE-data.ranges[0])      
        elif data.ranges[-1] < THRESHOLD_DISTANCE:
            self.turn_left(THRESHOLD_DISTANCE-data.ranges[-1])
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
    rospy.logwarn("TEST_STRING!")
    main()