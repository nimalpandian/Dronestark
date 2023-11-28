import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time
class DroneController:
def __init__(self):
rospy.init_node('drone_controller', anonymous=True)
self.state = State()
self.current_pose = PoseStamped()
self.rate = rospy.Rate(10) # 10 Hz
# Subscribers
self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
self.pose_callback)
# Publishers
self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped,
queue_size=10)
# Services
self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
self.set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
def state_callback(self, msg):
self.state = msg
def pose_callback(self, msg):
self.current_pose = msg
def arm(self):
return self.arm_service(True)
def disarm(self):
return self.arm_service(False)
def set_mode(self, mode):
return self.set_mode_service(custom_mode=mode)
def takeoff(self, altitude):
pose = PoseStamped(
header=Header(frame_id="map"),
pose=self.current_pose.pose
)
pose.pose.position.z = altitude
for _ in range(100): # Send setpoint for 10 seconds
  self.local_pos_pub.publish(pose)

self.rate.sleep()

def land(self):

pose = PoseStamped(

header=Header(frame_id="map"),

pose=self.current_pose.pose

)

pose.pose.position.z = 0

for _ in range(100): # Send setpoint for 10 seconds

self.local_pos_pub.publish(pose)

self.rate.sleep()

if __name__ == '__main__':

try:

controller = DroneController()

# Arm the drone

while not rospy.is_shutdown() and not controller.arm().success:

rospy.loginfo("Arming...")

controller.rate.sleep()

rospy.loginfo("Armed")

# Takeoff to an altitude of 2 meters
controller.takeoff(2)

rospy.loginfo("Taking off...")

# Hover for 5 seconds

time.sleep(5)

# Land the drone

controller.land()

rospy.loginfo("Landing...")

# Disarm the drone

while not rospy.is_shutdown() and not controller.disarm().success:

rospy.loginfo("Disarming...")

controller.rate.sleep()

rospy.loginfo("Disarmed")

except rospy.ROSInterruptException:

pass
