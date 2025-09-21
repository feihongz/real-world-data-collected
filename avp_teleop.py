import os
import time
import numpy as np
import pybullet as p
from avp_stream import VisionProStreamer
from scipy.spatial.transform import Rotation as R


def SE3_inv(SE3):
    SE3_inv = np.eye(4, dtype=np.float32)
    SE3_inv[:3, :3] = SE3[:3, :3].T
    SE3_inv[:3, 3] = -SE3[:3, :3].T @ SE3[:3, 3]
    return SE3_inv


#Set this to your IP address of you AVP
AVP_IP = "192.168.31.207"


class AVPTeleop():
    def __init__(self, is_left = False):
        # start AVP         
        self.vps = VisionProStreamer(ip = AVP_IP, record = True)
        
        # start pybullet
        p.connect(p.DIRECT)
        
        # load right leap hand           
        path_src = os.path.dirname(os.path.abspath(__file__))
        self.is_left = is_left
        self.leapEndEffectorIndex = [3, 4, 8, 9, 13, 14, 18, 19]
        if self.is_left:
            path_src = os.path.join(path_src, "leaphand/leap_hand_mesh_left/robot_pybullet.urdf")
            self.LeapId = p.loadURDF(
                path_src,
                [0.31, 0.01, 0.06],
                p.getQuaternionFromEuler([1.57, 0, 0]),
                useFixedBase = True
            )
        else:
            path_src = os.path.join(path_src, "leaphand/leap_hand_mesh_right/robot_pybullet.urdf")
            self.LeapId = p.loadURDF(
                path_src,
                [-0.22, 0.01, 0.03],
                p.getQuaternionFromEuler([1.57, 0, 3.14]),
                useFixedBase = True
            )

        self.numJoints = p.getNumJoints(self.LeapId)
    
    def get_wrist_pose(self):
        r = self.vps.latest
        wrist_pose = r['left_wrist'] if self.is_left else r['right_wrist']
        return wrist_pose[0]
    
    def set_init_pose(self, robot_init_pose):
        self.robot_init_pose = robot_init_pose
        self.hand_init_pose = self.get_wrist_pose()
        
    def print_frame(self, SE3):
        print("Translation:", SE3[:3, 3])
        print("Rotation (Euler xyz):", R.from_matrix(SE3[:3, :3]).as_euler('xyz', True))
        print("*" * 20)
        
    def debug(self):
        r = self.vps.latest
        print(r['left_pinch_distance'])
        # print('*' * 20)
    
    def get_mode_data(self):
        r = self.vps.latest
        return 'hand' if r['left_pinch_distance'] < 0.03 else 'both'
        
    def get_avp_data(self):
        r = self.vps.latest
        return self.get_arm_pose(), self.get_hand_joints(r)
        
    def get_arm_pose(self):
        X_VR2Robot = np.array([
            [-1,  0,  0,  0],
            [ 0, -1,  0,  0],
            [ 0,  0,  1,  0],
            [ 0,  0,  0,  1],
        ])
        
        hand_pose = self.get_wrist_pose()
        hand_transform_VR = np.eye(4, dtype=np.float32)
        hand_transform_VR[:3, :3] = hand_pose[:3, :3] @ self.hand_init_pose[:3, :3].T
        hand_transform_VR[:3, 3] = hand_pose[:3, 3] - self.hand_init_pose[:3, 3]
        hand_transform_robot = X_VR2Robot @ hand_transform_VR @ SE3_inv(X_VR2Robot)
        robot_pose = np.eye(4, dtype=np.float32)
        robot_pose[:3, :3] = hand_transform_robot[:3, :3] @ self.robot_init_pose[:3, :3]
        robot_pose[:3, 3] = self.robot_init_pose[:3, 3] + hand_transform_robot[:3, 3]
        self.print_frame(robot_pose)
        return robot_pose
    
    def get_hand_joints(self, r):          
        if self.is_left:
            hand_pose = np.asarray(r['left_fingers']).astype(float)  
        else:
            hand_pose = np.asarray(r['right_fingers']).astype(float) 
        indices = [3,4,8,9,13,14,18,19,23,24]
        hand_pos = hand_pose[indices, :3, 3]
        for i in range(0, 10):
            hand_pos[i][0] = hand_pos[i][0] * 1.35 * 1.5
            hand_pos[i][1] = hand_pos[i][1] * 1.5 * 1.5
            hand_pos[i][2] = hand_pos[i][2] * 1.5
        output = self.compute_IK(hand_pos)
        return output
        
    def compute_IK(self, hand_pos):
        rightHandIndex_middle_pos = hand_pos[2]
        rightHandIndex_pos = hand_pos[3]
        
        rightHandMiddle_middle_pos = hand_pos[4]
        rightHandMiddle_pos = hand_pos[5]
        
        rightHandRing_middle_pos = hand_pos[6]
        rightHandRing_pos = hand_pos[7]
        
        rightHandThumb_middle_pos = hand_pos[0]
        rightHandThumb_pos = hand_pos[1]
        
        leapEndEffectorPos = [
            rightHandIndex_middle_pos,
            rightHandIndex_pos,
            rightHandMiddle_middle_pos,
            rightHandMiddle_pos,
            rightHandRing_middle_pos,
            rightHandRing_pos,
            rightHandThumb_middle_pos,
            rightHandThumb_pos
        ]

        jointPoses = p.calculateInverseKinematics2(
            self.LeapId,
            self.leapEndEffectorIndex,
            leapEndEffectorPos,
            solver=p.IK_DLS,
            maxNumIterations=50,
            residualThreshold=0.0001,
        )
        
        combined_jointPoses = (jointPoses[0:4] + (0.0,) + jointPoses[4:8] + (0.0,) + jointPoses[8:12] + (0.0,) + jointPoses[12:16] + (0.0,))
        combined_jointPoses = list(combined_jointPoses)

        # update the hand joints
        for i in range(20):
            p.setJointMotorControl2(
                bodyIndex=self.LeapId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=combined_jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        # map results to real robot
        real_robot_hand_q = np.array([float(0.0) for _ in range(16)])
        real_robot_hand_q[:16] = jointPoses[:16]
        real_robot_hand_q[0:2] = real_robot_hand_q[0:2][::-1]
        real_robot_hand_q[4:6] = real_robot_hand_q[4:6][::-1]
        real_robot_hand_q[8:10] = real_robot_hand_q[8:10][::-1]
        return [float(i) for i in real_robot_hand_q]


if __name__ == '__main__':
    avp_teleop = AVPTeleop()
    robot_init_pose = np.array([
        [-0.00733604, -0.04980284, -0.99873215,  0.30379993],
        [ 0.8478434 ,  0.52924246, -0.03261895, -0.3310991 ],
        [ 0.530196  , -0.8470077 ,  0.03834246,  0.29632932],
        [ 0.        ,  0.        ,  0.        ,  1.        ]
    ])
    avp_teleop.set_init_pose(robot_init_pose)
    
    while True:
        arm_pose, hand_joints = avp_teleop.get_avp_data()
        time.sleep(0.1)
