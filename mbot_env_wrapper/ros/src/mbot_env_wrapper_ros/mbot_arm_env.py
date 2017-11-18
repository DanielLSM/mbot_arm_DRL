import gym
import rospy
import roslaunch
import time
import numpy as np

from gym import utils, spaces
from gym.spaces.box import Box
#from gym_gazebo.envs import gazebo_env
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from sensor_msgs.msg import LaserScan

from gym.utils import seeding

from gazebo_env_baseclass import GazeboEnv

from std_msgs.msg import Float64,Float32

from sensor_msgs.msg import JointState 

from gazebo_msgs.srv import SetModelConfiguration

import random



class GazeboMbotEnv(GazeboEnv):

    def __init__(self):
        # Launch the simulation with the given launchfile name
        GazeboEnv.__init__(self, "robot.launch",log = "rospy.FATAL")

        # Topic to publish velocity in each joint 
        self.joint_vel0 = rospy.Publisher('/left_arm_joint0_velocity_controller/command', Float64, queue_size=5)
        self.joint_vel1 = rospy.Publisher('/left_arm_joint1_velocity_controller/command', Float64, queue_size=5)
        self.joint_vel2 = rospy.Publisher('/left_arm_joint2_velocity_controller/command', Float64, queue_size=5)
        self.joint_vel3 = rospy.Publisher('/left_arm_joint3_velocity_controller/command', Float64, queue_size=5)
        self.joint_vel4 = rospy.Publisher('/left_arm_joint4_velocity_controller/command', Float64, queue_size=5)
        self.joint_vel5 = rospy.Publisher('/left_arm_joint5_velocity_controller/command', Float64, queue_size=5)
        self.joint_vel6 = rospy.Publisher('/left_arm_joint6_velocity_controller/command', Float64, queue_size=5)


        # self.torque_pub = rospy.Publisher('/acrobat/joint1/effort/command', Float32, queue_size=5)

        #Topic to read the state of the joint
        # self.joint_state = rospy.Subscriber('/joint_states', JointState)

        # Gazebo Service that resets simulation
        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        # Gazebo service to pause
        rospy.wait_for_service('/gazebo/pause_physics')
        p = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.pausa = p

        # Gazebo service to unpause
        rospy.wait_for_service('/gazebo/unpause_physics')
        up = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.fora_pausa = up

        rospy.wait_for_service('/gazebo/set_model_configuration')
        angus = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.set_joint1_angle_service = angus

        self.action_space = Box(low=-1, high=1, shape=(7,))
        self.reward_range = (-np.inf, np.inf)

        self._seed()

    def setJointAngle(self, angle):
        '''
        Uses /gazebo/set_model_configuration to set a position in the acrobat without using any controllers
        '''
        # call service /gazebo/set_model_configuration
        return self.set_joint1_angle_service('mbot', 'robot_description',   ['left_arm_joint0','left_arm_joint1','left_arm_joint2','left_arm_joint3','left_arm_joint4','left_arm_joint5','left_arm_joint6'], [angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6]]
                                                                          ) 

    @property
    def get_arm_data(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/joint_states', JointState, timeout=5)
            except:
                pass
        #rospy.loginfo("I heard {} radians on our joint from the top".format(data.position))
        return data

    @property
    def get_state(self):
        data = self.get_arm_data
        angle = data.position
        velocity = data.velocity
        l = []
        l.append(np.cos(angle[0]))
        l.append(np.sin(angle[0]))
        l.append(velocity[0])
        l.append(np.cos(angle[1]))
        l.append(np.sin(angle[1]))
        l.append(velocity[1])
        l.append(np.cos(angle[2]))
        l.append(np.sin(angle[2]))
        l.append(velocity[2])
        l.append(np.cos(angle[3]))
        l.append(np.sin(angle[3]))
        l.append(velocity[3])
        l.append(np.cos(angle[4]))
        l.append(np.sin(angle[4]))
        l.append(velocity[4])
        l.append(np.cos(angle[5]))
        l.append(np.sin(angle[5]))
        l.append(velocity[5])
        l.append(np.cos(angle[6]))
        l.append(np.sin(angle[6]))
        l.append(velocity[6])

        return np.asarray(l)

    @property
    def get_state_server(self):
        self.unpause
        r = self.get_state
        self.pause
        return r

    # Not needed because there are never angles superior to pi like we had in the acrobat
    #@property
    #def get_angle_server(self):
    #    self.unpause
    #    state = self.get_state
    #    angle = np.arccos(state[0])
    #    self.pause
    #    return angle

    @property
    def get_reward_euclidean(self):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/tf_euclidean_distance_node/euclidean_distance', Float32, timeout=1)
            except:
                pass
        return data


    def publish_velocities(self, velocities):
        #self.torque_pub.publish(data=torque)
        self.joint_vel0.publish(data=velocities[0])
        self.joint_vel1.publish(data=velocities[1])
        self.joint_vel2.publish(data=velocities[2])
        self.joint_vel3.publish(data=velocities[3])
        self.joint_vel4.publish(data=velocities[4])
        self.joint_vel5.publish(data=velocities[5])
        self.joint_vel6.publish(data=velocities[6])
        rospy.sleep(0.1)

    # Defines a seed
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # Makes a step
    def step(self, action):

        self.unpause
        self.publish_torque(action[0])
        state = self.get_state
        reward = self.get_reward_euclidean
        self.pause
        reward_alternative = self.reward(reward)
        done = False

        return state, reward_alternative, done


    def reward(self,reward):
        #print("Current angle is {}".format(angle))
        #return -10 * (reward**2)
        return -1 *(reward)

    # Resets
    @property
    def reset(self):

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        self.unpause
        
        # Reset Topic
        l=[0.0,1.18,0.12,0.92,0.24,0.63,0.44]
        #l=[random.uniform(-1, 1),
        #   random.uniform(-1, 1),
        #   random.uniform(-1, 1),
        #   random.uniform(-1, 1),
        #   random.uniform(-1, 1),
        #   random.uniform(-1, 1),
        #   random.uniform(-1, 1)]
        #random.uniform(-3.1415, 3.1415)
        self.setJointAngle(l)
        self.publish_velocities([0.0,0.0,0.0,0.0,0.0,0.0,0.0])
        
        # Read joint states
        state = self.get_state

        #Pause for next step
        self.pause

        return state


    def set_joint_server(self,angle):
        self.unpause
        self.setJointAngle(angle)
        self.pause

    # Pauses
    @property
    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pausa()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

    # Unpauses
    @property
    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.fora_pausa()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

    


