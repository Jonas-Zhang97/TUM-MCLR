import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
import pinocchio as pin
from visualization_msgs.msg import Marker

rospy.init_node('tf_publisher')

# Define the translation vectors and euler angles for the 8 frames of the cage
translations = [
    [-0.5, -0.3, -0.4],
    [0.5, -0.3, -0.4],
    [0.5, 0.3, -0.4],
    [-0.5, 0.3, -0.4],
    [-0.5, -0.3, 0.4],
    [0.5, -0.3, 0.4],
    [0.5, 0.3, 0.4],
    [-0.5, 0.3, 0.4]
]

euler_angles = [
    [0, 0, 0],
    [1.571, 0, 0],
    [3.146, 0, 0],
    [-1.571, 0, 0],
    [0, 0, -1.571],
    [3.146, 0, 3.146],
    [-1.571, 0, 3.146],
    [0, 0, 3.146]
]

def pub_transform(parent_frame, child_frame, M):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = M.translation[0]
    t.transform.translation.y = M.translation[1]
    t.transform.translation.z = M.translation[2]
    t.transform.rotation.x = pin.Quaternion(M.rotation).coeffs()[0]
    t.transform.rotation.y = pin.Quaternion(M.rotation).coeffs()[1]
    t.transform.rotation.z = pin.Quaternion(M.rotation).coeffs()[2]
    t.transform.rotation.w = pin.Quaternion(M.rotation).coeffs()[3]
    br.sendTransform(t)

# Define the initial linear and angular velocities of the center frame
linear_velocity = geometry_msgs.msg.Vector3(x=0.1, y=0, z=0)
angular_velocity = geometry_msgs.msg.Vector3(x=0, y=0.0, z=0.1)

# n= pin.Motion.Random()
# n.linear =np.array([1, 1, 1])
# n.angular =np.array([1, 1, 1])


# Define the time step for integration
dt = 0.1  # seconds

# Define the initial pose of the center frame with respect to the world frame
t0 = np.array([1.0, 0.0, 1.0])
e0 = np.array([0, 0, 0])


# Compute the new position and orientation of the center frame using linear and angular integration
def update_center(position, eulerangle):
    position[0] += linear_velocity.x * dt
    position[1] += linear_velocity.y * dt
    position[2] += linear_velocity.z * dt

    eulerangle[0] += angular_velocity.x * dt
    eulerangle[1] += angular_velocity.y * dt
    eulerangle[2] += angular_velocity.z * dt

def pub_marker(marker_id, p, frame_id, marker_type, color):
      
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Define the visualization marker
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.id = marker_id
    marker.pose.position.x = p[0]
    marker.pose.position.y = p[1]
    marker.pose.position.z = p[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.4   # 改变箭头的长度
    marker.scale.y = 0.03   # 改变箭头的宽度
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    # scale = geometry_msgs.msg.Vector3(1.5,5.2,0.2)
    # Publish the visualization marker
    marker_pub.publish(marker)

# Define the point vector p as a numpy array 
p0_cage1 = np.array([0.5, 0.5, 0.5])

while not rospy.is_shutdown():
    
    M0 = pin.SE3(pin.utils.rpyToMatrix(e0), t0)
    pub_transform("world", "center", M0)

    # Publish the transformations for the cage frames
    for i in range(8):
        # Create a Pinocchio SE(3) transformation matrix from the translation and euler angles
        M_ = pin.SE3(pin.utils.rotate('z', euler_angles[i][0]).dot(pin.utils.rotate('y', euler_angles[i][1])).dot(pin.utils.rotate('x', euler_angles[i][2])), np.array(translations[i]))
        #M_ = pin.SE3(pin.utils.rpyToMatrix(np.array(euler_angles[i])), np.array(translations[i]))
        # Publish the transformation for the current cage frame
        pub_transform("center", f"cage_{i+1}", M_)
        
        
    update_center(t0, e0)

    p0_world = t0 + translations[0] + p0_cage1
    # M0 = pin.exp6(n)* M0 
    color1 = [1.0, 0.0, 0.0]  # red
    color2 = [0.0, 1.0, 0.0]  # green

    pub_marker(0, p0_world, "world", 0, color1 )
    pub_marker(1, p0_cage1, "cage_1", 0, color2)

    rospy.sleep(0.1)  # sleep for a bit to control the publishing rate

rospy.spin()  # spin the ROS node to keep it running
