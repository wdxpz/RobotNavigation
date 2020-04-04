import rospy
from geometry_msgs.msg import Twist

PI = 3.1415926535897

class RotateController():
    def __init__(self):

        rospy.on_shutdown(self.shutdown)

        self.rotate_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10)
        self.rotate_command =Twist()

    def rotate(self, angle=90, speed=90, clockwise=True, stay=2):
        #Converting from angles to radians
        angular_speed = 1.0 #speed*PI/180
        relative_angle = angle*PI/180

        #We wont use linear components
        self.rotate_command.linear.x=0
        self.rotate_command.linear.y=0
        self.rotate_command.linear.z=0
        self.rotate_command.angular.x = 0
        self.rotate_command.angular.y = 0

        # Checking if our movement is CW or CCW
        if clockwise:
            self.rotate_command.angular.z = -abs(angular_speed)
        else:
            self.rotate_command.angular.z = abs(angular_speed)


        rate = 50
        r = rospy.Rate(rate)
        ticks = int(relative_angle*rate)
        for t in range(ticks):
            self.rotate_pub.publish(self.rotate_command)
            r.sleep()

        # # Setting the current time for distance calculus
        # t0 = rospy.Time.now().to_sec()
        # current_angle = 0


        
        # while(current_angle < relative_angle):
        #     print current_angle, relative_angle 
        #     self.rotate_pub.publish(self.rotate_command)
        #     rospy.sleep()
        #     t1 = rospy.Time.now().to_sec()
        #     current_angle = angular_speed*(t1-t0)

        #Forcing our robot to stop
        print 'send stop rotate command'
        self.rotate_command.angular.z = 0
        self.rotate_pub.publish(self.rotate_command)

    def shutdown(self):
        rospy.loginfo("Stop")
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = RotateController()

        navigator.rotate()

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
