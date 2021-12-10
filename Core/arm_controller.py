#!/usr/bin/env python

import rospy
from al5d_gazebo.msg import TransformStampedList
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading
from Queue import Queue
import numpy as np
from tf.transformations import quaternion_matrix
from copy import deepcopy
from time import sleep

import re
import sys

np.set_printoptions(suppress=True)

from std_msgs.msg import Empty, Bool, Header
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState, ModelStates

#class managing separate ROS loop
class ROSInterface:
    def __init__(self, cmd_q, status_q, pose_q, num_joints,namespace=''):
        #load up queues for async comm
        self.cmd_q = cmd_q
        self.status_q = status_q
        self.pose_q = pose_q
        self.num_joints = num_joints
        self.pos = None
        self.vel = None
        self.move_ind = 0
        self.move_seq = 0
        self.vel_target = None
        self.touch = rospy.Time.from_sec(0)
        self.model_data = None

        self.namespace = namespace

        self.joint_limits = np.array([[-1.4, 1.4],
                                      [-1.2, 1.4],
                                      [-1.8, 1.7],
                                      [-1.9, 1.7],
                                      [-2, 1.5],
                                      [-15, 30]]).T

        self.start = False # for wait_for_start Start Gun



    def state_cb(self, state):
        #add state to queue
        #note in original message joints are in alphabetical order
        #here we reorder to agree with command format
        self.pos = np.array([state.position[0],
                             state.position[4],
                             state.position[1],
                             state.position[6],
                             state.position[5],
                             state.position[2]])
        self.vel = np.array([state.velocity[0],
                             state.velocity[4],
                             state.velocity[1],
                             state.velocity[6],
                             state.velocity[5],
                             state.velocity[2]])

        #clear queue
        while not self.status_q.empty():
            self.status_q.get()
        self.status_q.put((self.pos, self.vel))

    def model_cb(self, msg):
        self.model_data = msg;

    def start_cb(self, msg):
        self.start = True

    def opponent_cb(self, msg):
        self.q_opponent = msg.position;
        self.qd_opponent = msg.velocity;

    def contact_cb(self, msg):

        collision = False
        # if any are a true collision, we are in collision. Otherwise not

        if len(self.namespace) > 0:
            model_name = "al5d_"+self.namespace[1:-1]
        else:
            model_name = "al5d"
        for state in msg.states:
            if ( not state.collision1_name.startswith(model_name) ) and ( not state.collision2_name.startswith(model_name) ):
                # not a collision with this robot
                pass
            elif (state.collision1_name == model_name+"::gripper_leftfinger::gripper_leftfinger_collision") and (state.collision2_name == model_name+"::gripper_rightfinger::gripper_rightfinger_collision"):
                # don't consider self collisions between gripper jaws
                pass
            elif (state.collision2_name == model_name+"al5d::gripper_leftfinger::gripper_leftfinger_collision") and (state.collision1_name == model_name+"::gripper_rightfinger::gripper_rightfinger_collision"):
                # order swapped
                pass
            else:
                collision = True
        if collision:
            self.touch = rospy.get_rostime()

    def pose_cb(self, pose):
        #we know the transforms are already in the order we want
        transforms = []
        for trans in pose.transforms:
            translation = 1000*np.array([trans.transform.translation.x,
                                         trans.transform.translation.y,
                                         trans.transform.translation.z])



            matrix = quaternion_matrix([trans.transform.rotation.x,
                                           trans.transform.rotation.y,
                                           trans.transform.rotation.z,
                                           trans.transform.rotation.w])
            matrix[:3,3] = translation
            transforms.append(matrix)

        #clear queue
        while not self.pose_q.empty():
            self.pose_q.get()
        self.pose_q.put(transforms)

    # handle starting either an interpolating waypoint list or a target velocity
    def start_command(self, state):
        if state[0] == "move":
            self.vel_target = None
            if self.pos is not None:
                #interpolate states between current state and goal
                start = self.pos
                diff = state[1] - start
                dist = np.max(np.abs(diff))
                interp = np.linspace(0, 1, max(np.ceil(80*dist), 2))
                self.move_seq = start + interp[:,None]*diff[None,:]
                self.move_seq[:,5] = self.move_seq[-1,5]; # don't interpolate gripper
                self.move_ind = 0
        elif state[0] == "velocity":
            self.move_seq = 0
            self.vel_target = deepcopy(state[1])
            self.pos_target = deepcopy(self.pos)
        elif state[0] == "setpoint":
            self.move_seq = np.array([state[1]])
            self.move_ind = 0

    # update the waypoint list or integrate the targeted pose due to target velocity
    def update_move(self):
        if self.move_seq is not 0:
            #if we are interpolating, move to next state
            if self.move_ind < self.move_seq.shape[0]:
                self.set_state(self.move_seq[self.move_ind,:])
                self.move_ind += 1
            else:
                self.move_ind = 0
                self.move_seq = 0
        elif self.vel_target is not None:
            # integrate
            dt = self.rate.sleep_dur.to_sec()
            self.pos_target = self.pos_target + dt * np.array(self.vel_target)
            # keep target within the joint limits
            self.pos_target[-1] = self.pos_target[-1] * 45.0/0.03
            # if we go all the way to the edge, it causes jitter, so we don't
            self.pos_target = np.maximum(self.pos_target, self.joint_limits[0]+.1)
            self.pos_target = np.minimum(self.pos_target, self.joint_limits[1]-.1)
            self.pos_target[-1] = self.pos_target[-1] * 0.03 / 45.0

            self.set_state(self.pos_target)

    # actually send the set points over ROS interface
    def set_state(self, state):
        for ind, s in enumerate(state[:-1]):
            msg = Float64(s)
            self.pos_pubs[ind].publish(msg)

        self.gripper_pubs[0].publish(Float64(-state[-1]))
        self.gripper_pubs[1].publish(Float64(state[-1]))

    def loop(self):
        self.node = rospy.init_node(self.namespace[2:-2]+'_arm_controller', disable_signals=True)
        self.rate = rospy.Rate(50)
        self.pos_pubs = []
        self.gripper_pubs = []
        for i in range(self.num_joints):
            pub = rospy.Publisher(self.namespace+"al5d_arm_position_controller"+str(i+1)+"/command",
                                  Float64, queue_size=1, latch=True)
            self.pos_pubs.append(pub)
        for i in range(2):
            pub = rospy.Publisher(self.namespace+"al5d_gripper_controller"+str(i+1)+"/command",
                                  Float64, queue_size=1, latch=True)
            self.gripper_pubs.append(pub)

        state_sub = rospy.Subscriber(self.namespace+"joint_states", JointState, self.state_cb)
        pose_sub = rospy.Subscriber(self.namespace+"joint_poses", TransformStampedList, self.pose_cb)
        contact_sub = rospy.Subscriber("/collision", ContactsState, self.contact_cb)

        start_sub = rospy.Subscriber("/start", Empty, self.start_cb)

        self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_cb);

        # identify opponent's state topic from the list of published topics to
        # allow us to read opponent state
        for name, type in rospy.get_published_topics():
            match = re.search('(\/.*\/)arm_interface\/state',name)
            if match is not None:
                name = match.group(1)
                if name != self.namespace:
                    # choose the first topic that isn't our own
                    self.opponent_sub = rospy.Subscriber(match.group(), JointState, self.opponent_cb);
                    break



        #poll at 50Hz

        while not rospy.is_shutdown():
            try:
                if not self.status_q.empty() and not self.pose_q.empty():
                    if not self.cmd_q.empty():
                        cmd = self.cmd_q.get()
                        if cmd == "stop":
                            break
                        self.start_command(cmd)
                    self.update_move()
                else:
                    pass
                self.rate.sleep()
            except KeyboardInterrupt:
                break

class ArmController:
    def __init__(self,  namespace='', num_joints=5):
        self.num_joints = num_joints
        self.cur_state = ()
        self.cur_pose = ()
        self.cmd_q = Queue()
        self.status_q = Queue()
        self.pose_q = Queue()

        if len(namespace) > 0:
            namespace = '/' + namespace + '/'


        #ROS runs in a separate thread because it needs to always
        #check the message buffers
        rospy.loginfo("Starting ros thread...")
        self.ros = ROSInterface(self.cmd_q, self.status_q, self.pose_q, num_joints,namespace=namespace)
        self.spin_t = threading.Thread(target=self.ros.loop)
        self.spin_t.start()
        rospy.loginfo("Ros thread started")

    # set a commanded velocity in joint space
    def set_vel(self, state):
        scaled_state = deepcopy(state)
        scaled_state[-1] = (-scaled_state[-1])/45.*0.03
        self.cmd_q.put(("velocity", scaled_state))

    # set a commanded joint torque
    def set_tau(self, state):
        rospy.logwarn("Torque control not yet implemented")

    # set a commanded position in joint space
    def set_pos(self, state):
        self.set_state(state)

    # directly modify the PID setpoint
    def command(self, state):
        scaled_state = deepcopy(state)
        scaled_state[-1] = (-scaled_state[-1]+30.)/45.*0.03
        self.cmd_q.put(("setpoint", scaled_state))

    # handle joint limits and scaling for gripper. Note that we left this function
    # interface intact (despite "state" being a misnomer) for some level of
    # backwards compatibility
    def set_state(self, state):
        #num joints + gripper
        if len(state) == self.num_joints + 1:
            scaled_state = deepcopy(state)
            #check limits
            if np.any(scaled_state < self.ros.joint_limits[0]):
                bad_joints = np.where(scaled_state < self.ros.joint_limits[0])[0]
                for bad_joint in bad_joints:
                    rospy.logwarn("Joint " + str(bad_joint) + " is below the limit " +
                          str(self.ros.joint_limits[0, bad_joint]))
                scaled_state = np.maximum(scaled_state, self.ros.joint_limits[0])
            if np.any(scaled_state > self.ros.joint_limits[1]):
                bad_joints = np.where(scaled_state > self.ros.joint_limits[1])[0]
                for bad_joint in bad_joints:
                    rospy.logwarn("Joint " + str(bad_joint) + " is above the limit " +
                          str(self.ros.joint_limits[1, bad_joint]))
                scaled_state = np.minimum(scaled_state, self.ros.joint_limits[1])


            scaled_state[-1] = (-scaled_state[-1]+30.)/45.*0.03
            self.cmd_q.put(("move", scaled_state))

        else:
            raise Exception("Invalid state command")

    # output: the joint value and its velocity
    def get_state(self):

        if not self.status_q.empty():
            # catching exception which might be due to multithreading
            try:
                self.cur_state = self.status_q.queue[0]
            except IndexError as error:
                pass
                # rospy.logwarn(error) # should remove before giving to students
                # don't update the current state, keep old value


        try:
            scaled_state = []
            scaled_state.append(deepcopy(self.cur_state[0]))
            scaled_state.append(deepcopy(self.cur_state[1]))
            scaled_state[0][-1] = -scaled_state[0][-1]*45./0.03 + 30.
            scaled_state[1][-1] = -scaled_state[1][-1]*45./0.03

            pos = np.around(scaled_state[0], decimals=3).tolist()
            vel = np.around(scaled_state[1], decimals=3).tolist()
        except IndexError as error:
            # have not yet received state from Gazebo
            # rospy.logwarn(error)
            pos = np.array([])
            vel = np.array([])

        return pos, vel

    # output: the T matrix of each joint
    def get_poses(self):

        if not self.pose_q.empty():
            self.cur_pose = self.pose_q.queue[0]

        return np.around(self.cur_pose, decimals=3)

    def wait_for_start(self):
        print("\n\n\nWaiting for start gun...")
        while not self.ros.start:
            sleep(.05)
        print("\nGo!\n\n\n")

    # halt ROS interface so script can terminate
    def stop(self):
        self.cmd_q.put("stop")
        self.spin_t.join()

    # if there has been a collision event in the last interval, assume collided
    def is_collided(self):
        since_touch = (rospy.get_rostime() - self.ros.touch).to_sec()
        return since_touch < .3


    def get_opponent_state(self):
        return self.ros.q_opponent, self.ros.qd_opponent

    def get_object_state(self):

        data = self.ros.model_data

        if data is None:
            return [], [], []

        cubes = [i for i, name in enumerate(data.name) if "cube" in name]
        name =  [name for name in data.name if "cube" in name]
        twist = []
        pose = []
        for cube in cubes:
            twist.append( np.array([
                data.twist[cube].linear.x,
                data.twist[cube].linear.y,
                data.twist[cube].linear.z,
                data.twist[cube].angular.x,
                data.twist[cube].angular.y,
                data.twist[cube].angular.z ]) )
            p = 1000*np.array([
                data.pose[cube].position.x,
                data.pose[cube].position.y,
                data.pose[cube].position.z ]) # in mm
            T = quaternion_matrix([
                data.pose[cube].orientation.x,
                data.pose[cube].orientation.y,
                data.pose[cube].orientation.z,
                data.pose[cube].orientation.w ])
            T[:3,3] = p;
            pose.append(T)

        return name, pose, twist





# We run an arm controller node to expose the API to MATLAB over the ROS messaging interface

if __name__ == '__main__':

    try:


        if len(sys.argv) < 4:
            lynx = ArmController()
            namespace = ''
        else:
            namespace = sys.argv[1]

            lynx = ArmController(namespace=namespace)

            namespace = '/' + namespace + '/'

        def position(msg):
            pos = np.array(msg.position)
            lynx.set_pos(pos)

        def velocity(msg):
            vel = np.array(msg.velocity)
            lynx.set_vel(vel)

        def setpoint(msg):
            q = np.array(msg.position)
            lynx.command(q)

        def torque(msg):
            tau = np.array(msg.effort)
            lynx.set_tau(tau)

        def stop(msg):
            lynx.stop()


        # Wait for ROS
        rospy.sleep(rospy.Duration.from_sec(3))


        pos_sub = rospy.Subscriber(namespace + "arm_interface/position", JointState, position)
        vel_sub = rospy.Subscriber(namespace + "arm_interface/velocity", JointState, velocity)
        command_sub = rospy.Subscriber(namespace + "arm_interface/setpoint", JointState, setpoint)
        torque_sub = rospy.Subscriber(namespace + "arm_interface/effort", JointState, torque)
        stop_sub = rospy.Subscriber(namespace + "arm_interface/stop", Empty, stop)

        collision_pub = rospy.Publisher(namespace + "arm_interface/collided", Bool, queue_size=1, latch=True)
        joint_pub = rospy.Publisher(namespace + "arm_interface/state", JointState, queue_size=1, latch=True)

        rate = rospy.Rate(100)

        # loop to publish data for MATLAB
        while not rospy.is_shutdown():
            try:
                collision_pub.publish(Bool(lynx.is_collided()))

                # we republish joint data due to reordering and scaling
                msg = JointState();
                pos, vel = lynx.get_state()
                msg.position = pos
                msg.velocity = vel
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'world'
                msg.name = ["upper_base", "upper_arm", "lower_arm", "wrist", "gripper_base", "end"];
                joint_pub.publish(msg)


                rate.sleep()
            except KeyboardInterrupt:
                break


    except rospy.ROSInterruptException:
        lynx.stop()
        pass
