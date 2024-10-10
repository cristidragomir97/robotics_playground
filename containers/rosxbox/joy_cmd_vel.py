
import rospy, subprocess

from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


SOURCE_ROOT = ". /opt/ros/noetic/setup.sh"
SOURCE_WS = ". /root/catkin_ws/devel/setup.sh"
COMMAND = "rosrun joy joy_node"
JOY_NODE = "{} && {} && {}".format(SOURCE_ROOT, SOURCE_WS, COMMAND)


class JoyToCommands(object):
    def __init__(self):
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/joy", Joy, callback=self.joy_cb, queue_size=1)
        self.linear_vel_multiplier = 3.5
        self.angular_vel_multiplier = 3.5
        self.enable_movement = False  # Tracks whether movement is enabled or not
        self.previous_button_state = 0  # Used to detect button press events (latching)

    def joy_cb(self, joy_msg):
        # Detect button press (latching behavior)
        current_button_state = joy_msg.buttons[0]

        if current_button_state == 1 and self.previous_button_state == 0:
            # Toggle the movement enable flag when the A button is pressed
            self.enable_movement = not self.enable_movement
            rospy.loginfo(f"Movement enabled: {self.enable_movement}")

        self.previous_button_state = current_button_state  # Update button state for next callback

        twist_cmd = Twist()

        if self.enable_movement:
            # Axes 1 controls forward/backward (linear.x)
            twist_cmd.linear.x = self.linear_vel_multiplier * joy_msg.axes[1]
            # Axes 0 controls left/right rotation (angular.z)
            twist_cmd.angular.z = self.angular_vel_multiplier * joy_msg.axes[0]
        else:
            # Stop the robot if movement is disabled
            twist_cmd.linear.x = 0
            twist_cmd.angular.z = 0

        # Publish the twist command
        self.twist_pub.publish(twist_cmd)




if __name__ == '__main__':    
    print(JOY_NODE)
    subprocess.Popen(JOY_NODE, shell=True)
    rospy.init_node("joy2commands", log_level=rospy.INFO)    
    JoyToCommands()    
    rospy.spin()