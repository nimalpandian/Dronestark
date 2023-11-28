import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import math
class HelicalTrajectoryController:
def __init__(self):
rospy.init_node('helical_trajectory_controller', anonymous=True)
self.rate = rospy.Rate(10) # 10 Hz
self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
self.takeoff_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
self.land_pub = rospy.Publisher('/drone/land', Empty, queue_size=1)
self.takeoff()
rospy.on_shutdown(self.land)
def takeoff(self):
rospy.loginfo("Taking off...")
takeoff_msg = Empty()
self.takeoff_pub.publish(takeoff_msg)
rospy.sleep(3) # Wait for the drone to take off
def land(self):
rospy.loginfo("Landing...")
land_msg = Empty()
self.land_pub.publish(land_msg)
rospy.sleep(3) # Wait for the drone to land
def execute_helical_trajectory(self, radius, pitch, frequency, duration):
start_time = rospy.Time.now()
while rospy.Time.now() - start_time < rospy.Duration.from_sec(duration):
current_time = rospy.Time.now()
elapsed_time = current_time - start_time
x = radius * math.cos(2 * math.pi * frequency * elapsed_time.to_sec())
y = radius * math.sin(2 * math.pi * frequency * elapsed_time.to_sec())
z = pitch * elapsed_time.to_sec()
cmd_vel_msg = Twist()
cmd_vel_msg.linear.x = x
cmd_vel_msg.linear.y = y
cmd_vel_msg.linear.z = z
self.cmd_vel_pub.publish(cmd_vel_msg)
self.rate.sleep()
if __name__ == '__main__':
try:
controller = HelicalTrajectoryController()
# Example parameters: radius=2, pitch=0.5, frequency=0.2, duration=30
controller.execute_helical_trajectory(radius=2, pitch=0.5, frequency=0.2, duration=30)
except rospy.ROSInterruptException:
pass
