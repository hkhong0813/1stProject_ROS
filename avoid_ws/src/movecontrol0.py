#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys,select,termios,tty
from sensor_msgs.msg import LaserScan
import time

BURGER_MAX_LIN_VEL = 0.17
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
w a s d x
"""

caution = """ 
속도가 너무 빠릅니다
"""

class Obstacle():
    def __init__(self):
        self.LIDAR_ERR = 0.05
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()

    def obstacle(self):
        self.twist = Twist()
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message("/scan", LaserScan)
            self.scan_filter = []
            for i in range(360):
                if i <=45 or i >315:
                    if msg.ranges[i] >= self.LIDAR_ERR:
                        self.scan_filter.append(msg.ranges[i])
            if min (self.scan_filter) < 0.10:
                self.twist.linear.x = 0.00
                self.twist.angular.z = 0.0
                self._cmd_pub.publish(self.twist)
                rospy.loginfo("stop")
                break
            else:
                self.twist.linear.x = 0.00
                self.twist.angular.z = 0.0
                rospy.loginfo("distance of the obstacle : %f", min(self.scan_filter))
                rospy.loginfo(time.time())
            self._cmd_pub.publish(self.twist)

def getkey():
    tty.setraw(sys.stdin.fileno())
    rlist, _,_ =select.select([sys.stdin],[],[],0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
    return key
def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s" % (target_linear_vel,target_angular_vel)
def Calculate_vel(output, input, slop):
    if input > output:
        output = min(input,output+slop)
    elif input < output:
        output = max(input,output-slop)
    else:
        output = input
    return output
def constrain(input, low, high):
    if input < low:
        input = low
    elif input > high:
        input = high
    else :
        input = input
    return input

def check_LIN_limit_VEL(vel):
    vel = constrain(vel, -BURGER_MAX_LIN_VEL,BURGER_MAX_LIN_VEL)
    return vel
def check_ANG_limit_VEL(vel):
    vel = constrain(vel, -BURGER_MAX_ANG_VEL,BURGER_MAX_ANG_VEL)
    return vel

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('liv')
    pub = rospy.Publisher('/cmd_vel ',Twist, queue_size=10)

    count = 0
    whole_count = 0
    status = 0
    target_linear_vel = 0
    target_angular_vel = 0
    control_linear_vel = 0
    control_angular_vel = 0
    # scan_filter = []
    try:
        while(1):
            mode_obstacle = Obstacle()
            print(msg)
            while(1):
                key = getkey()
                twist = Twist()

                if key == 'w':
                    target_linear_vel =  check_LIN_limit_VEL(target_linear_vel+LIN_VEL_STEP_SIZE)
                    status = status +1
                    print(target_linear_vel,target_angular_vel)
                    if target_linear_vel == check_LIN_limit_VEL(target_linear_vel +LIN_VEL_STEP_SIZE):
                        print(caution)
                        count = count +1
                        if count == 3:
                            target_linear_vel = target_linear_vel/2
                            print(vels(target_linear_vel,target_angular_vel))                            
                            count = 0
                elif key == 'x':
                    target_linear_vel =  check_LIN_limit_VEL(target_linear_vel-LIN_VEL_STEP_SIZE)
                    status = status +1
                    print(target_linear_vel,target_angular_vel)
                    if target_linear_vel == check_LIN_limit_VEL(target_linear_vel -LIN_VEL_STEP_SIZE):
                        print(caution)
                        count = count +1
                        if count == 3:
                            target_linear_vel = target_linear_vel/2
                            print(vels(target_linear_vel,target_angular_vel) )                                                   
                            count = 0
                elif key == 'a':
                    target_angular_vel =  check_LIN_limit_VEL(target_angular_vel+ANG_VEL_STEP_SIZE)
                    status = status +1
                    print(target_linear_vel,target_angular_vel)
                    if target_angular_vel == check_LIN_limit_VEL(target_angular_vel +ANG_VEL_STEP_SIZE):
                        print(caution)
                        count = count +1
                        if count == 3:
                            target_angular_vel = target_angular_vel/2
                            print(vels(target_linear_vel,target_angular_vel))
                            count = 0
                elif key == 'd':
                    target_angular_vel =  check_LIN_limit_VEL(target_angular_vel-ANG_VEL_STEP_SIZE)
                    status = status +1
                    print(target_linear_vel,target_angular_vel)
                    if target_angular_vel == check_LIN_limit_VEL(target_angular_vel -ANG_VEL_STEP_SIZE):
                        print(caution)
                        count = count +1
                        if count == 3:
                            target_angular_vel = target_angular_vel/2
                            print(vels(target_linear_vel,target_angular_vel))                            
                            count = 0
                # elif key == 'p':
                #     rospy.loginfo('distance of the obstacle : %f', min(scan_filter))
                elif key == 's':
                    target_linear_vel = 0
                    control_linear_vel = 0
                    target_angular_vel = 0
                    control_angular_vel = 0
                    print(vels(0,0))
                    
                elif status == 20:
                    print(msg)
                    status = 0
                else:
                    if (key == '\x03'):
                        break
                control_linear_vel = Calculate_vel(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z =0.0

                control_angular_vel = Calculate_vel(control_angular_vel,target_angular_vel,(ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 00; twist.angular.z = control_angular_vel

                pub.publish(twist)
    except:
        print('e')
    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z =0.0
        twist.angular.x = 0.0; twist.angular.y = 00; twist.angular.z = 0.0
        pub.publish(twist)
    termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
