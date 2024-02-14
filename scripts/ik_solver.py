#!/usr/bin/env python3

import math
import yaml 
import rospy
import numpy as np
import PyKDL as kdl
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import JointState
import kdl_parser_py.urdf as kdl_parser

with open("/home/gokul/ROS/iroc_ws/src/custom_ik/config/ik_params.yaml","r") as file:
    link_dict = yaml.safe_load(file)
    baselink = link_dict.get('arm').get('baselink')
    endlink = link_dict.get('arm').get('endlink')

class custom_ik:

    def __init__(self):
        self.joint_angles = [0,0,0,0,0,0]
        self.joint_velocities = []
        self.joint_efforts = []
        rospy.Subscriber("/joint_states",JointState, self.state_callback)
        self.kdl_setup()
    
    def num_joints(self):
        return self.num_joints

    def kdl_setup(self):
        self.baselink = baselink
        self.endlink = endlink
        self.arm = URDF.from_parameter_server(key='robot_description')
        _, self.tree = kdl_parser.treeFromParam("/robot_description")
        
        self.chain = self.tree.getChain(self.baselink, self.endlink)
        self.fk_p_ee = kdl.ChainFkSolverPos_recursive(self.chain)
        self.fk_v_ee = kdl.ChainFkSolverVel_recursive(self.chain)
        self.ik_v_ee = kdl.ChainIkSolverVel_pinv(self.chain)
        self.ik_p_ee = kdl.ChainIkSolverPos_NR(self.chain, self.fk_p_ee, self.ik_v_ee)
        
        self.num_joints = self.chain.getNrOfJoints()
        self.joint_limits = self.get_joint_limits()

    def state_callback(self, msg):
        for i in range(len(msg.position)):
            self.joint_angles[i] = msg.position[i]
            if len(msg.velocity) != 0: 
                self.joint_velocities[i] = msg.velocity[i]
            if len(msg.effort) != 0:
                self.joint_efforts[i] = msg.effort[i]
    
    def robot_description_output(self):
        nonfixed_joints = 0
        for j in self.arm.joints:
            if j.type != "fixed":
                nonfixed_joints += 1
        print("Non-Fixed Joints: ", nonfixed_joints)
        print("Total Joints: ", len(self.arm.joints))
        print("Manipulator Links: ", len(self.arm.links))
        print("No of Joints: ", self.num_joints)

    def get_joint_limits(self):
        joint_limits = {}
        for joint in self.arm.joints:
            if joint.type != "fixed":
                if joint.limit != None:
                    joint_limits[joint.name] = (-math.pi, math.pi)
                else:
                    joint_limits[joint.name] = (-math.pi, math.pi)
        return joint_limits

    def check_joint_limits(self, joint_angle):
        for joint_name, (lower_limit, upper_limit) in self.joint_limits.items():
            id = list(self.joint_limits).index(joint_name)
            range = upper_limit - lower_limit
            if joint_angle[id] < lower_limit:
                joint_angle[id] = joint_angle[id] + abs(round(joint_angle[id]/range))*range
            elif joint_angle[id] > upper_limit:
                joint_angle[id] = joint_angle[id] - abs(round(joint_angle[id]/range))*range
        return joint_angle
    
    def joints_to_kdl(self, type, values=None):
        kdl_array = kdl.JntArray(self.num_joints)
        if values is None:
            if type == "positions":
                cur_type_values = self.joint_angles
            elif type == "velocities":
                cur_type_values = self.joint_velocities
            elif type == "effort":
                cur_type_values = self.joint_efforts
        else:
            cur_type_values = values
        
        for i in range(self.num_joints):
            kdl_array[i] = cur_type_values[i]
        if type == "velocities":
            kdl_array = kdl.JntArrayVel(kdl_array)
        return kdl_array

    def forward_kinematics(self, joint_values = None):
        end_frame = kdl.Frame()
        self.fk_p_ee.JntToCart(self.joints_to_kdl('positions', joint_values),end_frame)
        pos = end_frame.p
        rot = kdl.Rotation(end_frame.M)
        rot = rot.GetQuaternion()
        return np.array([pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3]])
    
    def inverse_kinematics(self, position, orientation=None, seed=None):
        ik = kdl.ChainIkSolverVel_pinv(self.chain)
        pos = kdl.Vector(position[0], position[1], position[2])
        if orientation != None:
            rot = kdl.Rotation()
            rot = rot.Quaternion(orientation[0], orientation[1], orientation[2], orientation[3])
        seed_array = kdl.JntArray(self.num_joints)
        if seed != None:
            seed_array.resize(len(seed))
            for idx, jnt in enumerate(seed):
                seed_array[idx] = jnt
        else:
            seed_array = self.joints_to_kdl('positions')
        
        if orientation:
            goal_pose = kdl.Frame(rot, pos)
        else:
            goal_pose = kdl.Frame(pos)
        result_angles = kdl.JntArray(self.num_joints)

        if self.ik_p_ee.CartToJnt(seed_array, goal_pose, result_angles) >= 0:
            result = np.array(list(result_angles))
            result = self.check_joint_limits(result)
            return result
        else:
            return rospy.logerr("No Solution Found")


def main():
    arm = custom_ik()
    arm.robot_description_output()
    result = arm.inverse_kinematics(position=[0.109342, -0.0107926, -0.195518], orientation=None)
    result_degrees = [round((round(i,2)*180)/math.pi) for i in result]
    result_radians = [round(i,2) for i in result]
    joints = JointState()
    print(f"Joint Angles (radians) : {result_radians}")
    print(f"Joint Angles (degrees) : {result_degrees}")

    while not rospy.is_shutdown():
        joints.position = result
        pub.publish(joints)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("ik_solver", anonymous=False)
    pub = rospy.Publisher("/goal_state", JointState, queue_size=10)
    main()
    
