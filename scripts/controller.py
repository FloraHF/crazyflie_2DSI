# !/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Int16
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped

class Controller():

    def __init__(self, cmd_topic, goal_topic, 
                        worldFrame, frame, 
                        cmd=1, goal=[0., 0., 0.5, 0.]):

        self._worldFrame = worldFrame
        # self._frame = frame
        # self._m_listener = tf.TransformListener()
        self._goal_msg = PoseStamped()
        self.updateGoal(goal=goal, init=True)

        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

        rospy.loginfo("waiting for land service")
        rospy.wait_for_service('land')
        rospy.loginfo("found land service")
        self._land = rospy.ServiceProxy('land', Empty)

        rospy.loginfo("waiting for takeoff service")
        rospy.wait_for_service('takeoff')
        rospy.loginfo("found takeoff service")
        self._takeoff = rospy.ServiceProxy('takeoff', Empty)

        self._cmd_sub = rospy.Subscriber(cmd_topic, Int16, self.sendCmd)
        self._goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)

    # def getLocation(self):
    #     transform = self._m_listener.lookupTransform(self._worldFrame, self._frame, rospy.Time(0))

    #     targetWorld = PoseStamped()
    #     targetWorld.header.stamp = transform.stamp_
    #     targetWorld.header.frame_id = self._worldFrame
    #     targetWorld.pose = self._goal_msg.pose

    #     targetDrone = self._m_listener.transformPose(self._frame, targetWorld)

    #     x = targetDrone.pose.position.x
    #     y = targetDrone.pose.position.y

    #     return np.array([x, y])


    def updateGoal(goal=None, init=False):
        
        if init:
            self._goal_msg.header.seq = 0
            self._goal_msg.header.frame_id = self._worldFrame
        else:
            self._goal_msg.header.seq += 1

        self._goal_msg.header.stamp = rospy.Time.now()

        if goal is not None:
            self._goal_msg.pose.position.x = goal[0]
            self._goal_msg.pose.position.y = goal[0]
            self._goal_msg.pose.position.z = goal[0]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
            self._goal_msg.pose.orientation.x = quaternion[0]
            self._goal_msg.pose.orientation.y = quaternion[1]
            self._goal_msg.pose.orientation.z = quaternion[2]
            self._goal_msg.pose.orientation.w = quaternion[3]

    def sendCmd(self, cmd):
        if cmd == 1:
            self._takeoff()
            print('received takeoff command')
        if cmd == 2:
            self._land()
            print('received landing command')
        if cmd == 0:
            self._emergency()
            print('received emergency command')

    def hover(self):
        while not rospy.is_shutdown():
            self.updateGoal()
            self._pub.publish(self._goal_msg)
            rate.sleep()

if __name__ == '__main__':

    rospy.init_node('cf2DSI_controller', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    cmd_topic  = rospy.get_param('cmd_topic', 'cf2/cmd_op')
    goal_topic = rospy.get_param('goal_topic', 'cf2/goal')
    controller = Controller(cmd_topic, goal_topic, worldFrame)
    controller.hover()


