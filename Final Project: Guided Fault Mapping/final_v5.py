import rospy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import math
import numpy
import csv

from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

    #automatic location extraction
waypoints = csv.reader(open('/root/Desktop/waypoints.csv', 'r'), delimiter=",")
column1, column2 = [], []
for row in waypoints:
    column1.append(row[0])
    column2.append(row[1])
column1 = column1[1:]
column2 = column2[1:]
x = [float (x) for x in column1]
y = [float (y) for y in column2]
z = [25] #altitude in meter
    
#set homepoint
xh = [0] #edit this line to home point's x coordinate
yh = [0] #edit this line to home point's y coordinate   

#create loop for waypoints
xr = list(reversed(x))
xr.pop(0)
yr = list(reversed(y))
yr.pop(0)
xl = x + xr + xh + [xh[0]+1] 
yl = y + yr + yh + [yh[0]+1]
global x,y,z,l
x = xl 
y = yl
z = z
l = len(x)-1
class OffbPosCtl:
    curr_drone_pose = PoseStamped()
    waypointIndex = 0
    distThreshold = 1
    sim_ctr = 1

    des_pose = PoseStamped()
    isReadyToFly = False
    """
    # manual location
    x = [5.302,15.667686462402344, 17.667686462402344, 60.667686462402344]
    y = [-35.794,83.42528533935547, 81.42528533935547, -67.57471466064453]
    z = 30 #drone altitude in meter
    yaw = numpy.deg2rad(300) #drone horizontal orientation from east point, counterclockwise, input in degrees
    orientation = quaternion_from_euler(0, 0, yaw) #conversion from euler angles(roll, pitch, yaw) to quaternion
    # matrix containing waypoints:
    # four set points: locations =[x, y, z, quat[0], quat[1], quat[2], quat[3]]
    locations = numpy.matrix([[x[0], y[0], z, orientation[0], orientation[1], orientation[2], orientation[3]],
                              [x[1], y[1], z, orientation[0], orientation[1], orientation[2], orientation[3]],                       
                              [x[2], y[2], z, orientation[0], orientation[1], orientation[2], orientation[3]]
                              ])
    
    """
    #automatic orientation
    yaw = []
    for i in range(len(x)-1):
       if y[i+1]>y[i] and x[i+1]>x[i]:
          yaw.append(math.atan((y[i+1]-y[i])/(x[i+1]-x[i]))) #1st quadrant, angle in radian, from east point
       if y[i+1]>y[i] and x[i+1]<x[i]:
          yaw.append(3.14+math.atan((y[i+1]-y[i])/(x[i+1]-x[i]))) #2nd quadrant
       if y[i+1]<y[i] and x[i+1]>x[i]:
          yaw.append(math.atan((y[i+1]-y[i])/(x[i+1]-x[i]))) #3rd quadrant
       if y[i+1]<y[i] and x[i+1]<x[i]:
          yaw.append(3.14-math.atan((y[i+1]-y[i])/(x[i+1]-x[i]))) #4th quadrant
    yaw.insert(0,0)
    yaw.append(0)
    orientations = [quaternion_from_euler(0, 0, yaw[i]) for i in range(len(yaw))] #conversion from euler angles(roll, pitch, yaw) to quaternion
    
    #create locations matrix
    list_loc = []
    for i in range(len(x)):
        list_loc.append([x[i], y[i], z[0], orientations[i][0], orientations[i][1], orientations[i][2], orientations[i][3]])
    # location matrix
    locations = numpy.matrix(list_loc)
        
    def mavrosTopicStringRoot(self, uavID=0):
        mav_topic_string = '/mavros/'
        return mav_topic_string

    def __init__(self):
        rospy.init_node('offboard_test', anonymous=True)
        pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        drone_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.drone_pose_cb)
        rover_pose_subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                                                 callback=self.rover_pose_cb)
        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.drone_state_cb)
        attach = rospy.Publisher('/attach', String, queue_size=10)
        NUM_UAV = 2
        mode_proxy = [None for i in range(NUM_UAV)] # initially, mode_proxy = [None, None]
        arm_proxy = [None for i in range(NUM_UAV)]

        # Comm for drones
        for uavID in range(0, NUM_UAV): #uavID is like "i" in for i in range
            mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
            arm_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/cmd/arming', CommandBool)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()
        self.des_pose = self.copy_pose(self.curr_drone_pose)
        shape = self.locations.shape

        while not rospy.is_shutdown():
            print(self.sim_ctr, shape[0], self.waypointIndex)
            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                try:
                    mode_proxy[uavID] = rospy.ServiceProxy(self.mavrosTopicStringRoot(uavID) + '/set_mode', SetMode)
                    if self.waypointIndex is l:
                        success[uavID] = mode_proxy[uavID](1, 'AUTO.LAND')
                        print("landing")
                    else:
                        success[uavID] = mode_proxy[uavID](1, 'OFFBOARD')
                except rospy.ServiceException as e:
                    print ("mavros/set_mode service call failed: %s" % e)


            success = [None for i in range(NUM_UAV)]
            for uavID in range(0, NUM_UAV):
                rospy.wait_for_service(self.mavrosTopicStringRoot(uavID) + '/cmd/arming')

            for uavID in range(0, NUM_UAV):
                try:
                    success[uavID] = arm_proxy[uavID](True)
                except rospy.ServiceException as e:
                    print ("mavros1/set_mode service call failed: %s" % e)

            if self.waypointIndex is shape[0]:
                self.waypointIndex = 0
                self.sim_ctr += 1

            if self.waypointIndex is l:
                attach.publish("ATTACH")
                print("checkpoint 1")

            if self.isReadyToFly:
                des = self.set_desired_pose().position
                azimuth = math.atan2(self.curr_rover_pose.pose.position.y - self.curr_drone_pose.pose.position.y,
                                     self.curr_rover_pose.pose.position.x - self.curr_drone_pose.pose.position.x)
                az_quat = quaternion_from_euler(0, 0, azimuth)

                curr = self.curr_drone_pose.pose.position
                dist = math.sqrt(
                    (curr.x - des.x) * (curr.x - des.x) + (curr.y - des.y) * (curr.y - des.y) + (curr.z - des.z) * (
                            curr.z - des.z))
                if dist < self.distThreshold:
                    self.waypointIndex += 1

            pose_pub.publish(self.des_pose)
            rate.sleep()

    def set_desired_pose(self):
        self.des_pose.pose.position.x = self.locations[self.waypointIndex, 0]
        self.des_pose.pose.position.y = self.locations[self.waypointIndex, 1]
        self.des_pose.pose.position.z = self.locations[self.waypointIndex, 2]
        self.des_pose.pose.orientation.x = self.locations[self.waypointIndex, 3]
        self.des_pose.pose.orientation.y = self.locations[self.waypointIndex, 4]
        self.des_pose.pose.orientation.z = self.locations[self.waypointIndex, 5]
        self.des_pose.pose.orientation.w = self.locations[self.waypointIndex, 6]
        if self.locations[self.waypointIndex, :].sum() == 0:
            self.des_pose.pose.position.x = self.curr_rover_pose.pose.position.x
            self.des_pose.pose.position.y = self.curr_rover_pose.pose.position.y
            self.des_pose.pose.position.z = max(self.curr_rover_pose.pose.position.z, 10)
            orientation = quaternion_from_euler(0, 0, 3.14/2)
            self.des_pose.pose.orientation = Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        return self.des_pose.pose

    def copy_pose(self, pose):
        pt = pose.pose.position
        quat = pose.pose.orientation
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = pose.header.frame_id
        copied_pose.pose.position = Point(pt.x, pt.y, pt.z)
        copied_pose.pose.orientation = Quaternion(quat.x, quat.y, quat.z, quat.w)
        return copied_pose
        #print(copied_pose)
    def drone_pose_cb(self, msg):
        self.curr_drone_pose = msg
        print(msg.pose)
        print(self.waypointIndex)
	#print(len(x))

    def rover_pose_cb(self, msg):
        self.curr_rover_pose = msg

    def drone_state_cb(self, msg):
        #print(msg.mode)
        if msg.mode == 'OFFBOARD':
            self.isReadyToFly = True
        #    print("readyToFly")


if __name__ == "__main__":
    OffbPosCtl()
