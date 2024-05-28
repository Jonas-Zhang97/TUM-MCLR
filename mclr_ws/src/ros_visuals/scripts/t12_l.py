import rospy
import tf2_ros
import geometry_msgs.msg
import numpy as np
import pinocchio as pin
from visualization_msgs.msg import Marker

rospy.init_node('tf_twist_publisher')

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

# Define the initial pose of the center frame with respect to the world frame
t0 = np.array([1.0, 0.0, 1.0])
e0 = np.array([0, 0, 0])
M0 = pin.SE3(pin.utils.rpyToMatrix(e0), t0)

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

def publish_twist(msg_id, twist, frame_id):
    
    twist_pub = rospy.Publisher('twist', geometry_msgs.msg.TwistStamped, queue_size=10)
      
    # Create a TwistStamped message
    twist_msg = geometry_msgs.msg.TwistStamped()
    twist_msg.header.frame_id = frame_id
    twist_msg.header.stamp = rospy.Time.now()
    twist_msg.header.seq = msg_id
    twist_msg.twist.linear.x = twist.linear[0] #twist[3]
    twist_msg.twist.linear.y = twist.linear[1] #twist[4]
    twist_msg.twist.linear.z = twist.linear[2] #twist[5]
    twist_msg.twist.angular.x = twist.angular[0] #twist[0]
    twist_msg.twist.angular.y = twist.angular[1] #twist[1]
    twist_msg.twist.angular.z = twist.angular[2] #twist[2]

    # Publish the TwistStamped message
    twist_pub.publish(twist_msg)       

# Define the twist as a numpy array
cV0 = np.array([0.01, 0, 0, 0.01, 0, 0.0])
cV = pin.Motion(cV0)
#cV = pin.Motion(v_linear, v_angular)
#cVc = pin.Motion(cV0+np.hstack((translations[1],euler_angles[1])))
while not rospy.is_shutdown():

    pub_transform("world", "center", M0)
    
    # Publish the transformations for the cage frames
    for i in range(8): #starts from 0 
        # Create a Pinocchio SE(3) transformation matrix from the translation and euler angles
        M_ = pin.SE3(pin.utils.rotate('z', euler_angles[i][0]).dot(pin.utils.rotate('y', euler_angles[i][1])).dot(pin.utils.rotate('x', euler_angles[i][2])), np.array(translations[i]))
        #M_ = pin.SE3(pin.utils.rpyToMatrix(np.array(euler_angles[i])), np.array(translations[i]))
        # Publish the transformation for the current cage frame
        pub_transform("center", f"cage_{i+1}", M_)
        if i == 3: # i = n, M_: cage_(n+1)
            cVc = M_.inverse().act(cV)
            #print(cVc)    
    
    # Update the center frame using matrix multiplication
    M0 = pin.exp6(cV)* M0 #rotation matrix * current frame
    
    # Call the publish_twist function with the twist and frame ID
    publish_twist(1, cV*100, "world")
    #publish_twist(1, cVc*100, "cage_4")
    
    # publish_twist(twist1, "center")
    rospy.sleep(0.1)  # sleep for a bit to control the publishing rate

rospy.spin()  # spin the ROS node to keep it running