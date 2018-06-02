#! /usr/bin/env python
import numpy as np
import math
import time
import ikpy
from pyquaternion import Quaternion

import kinematics_math   

class KinematicSolver:
    def __init__(self):
        # coordinate system
        # 0-config should lie along x axis
        # y -z
        # |/
        # -- x
        links = [
            ikpy.link.URDFLink(
                name="rotational_base",
                translation_vector=np.array([0,0,0]),
                orientation=np.array([0,0,0]),
                rotation=np.array([0,1,0])),
            ikpy.link.URDFLink(
                name="base_joint",
                translation_vector=np.array([0,0,0]),
                orientation=np.array([0,0,0]),
                rotation=np.array([0,0,1])),
            ikpy.link.URDFLink(
                name="to_elbow",
                translation_vector=np.array([1,0,0]),
                orientation=np.array([0,0,0]),
                rotation=np.array([0,0,1])),
            ikpy.link.URDFLink(
                name="to_wrist",
                translation_vector=np.array([1,0,0]),
                orientation=np.array([0,0,0]),
                rotation=np.array([0,0,1])),
            ikpy.link.URDFLink(
                name="to_manipulator",
                translation_vector=np.array([1,0,0]),
                orientation=np.array([0,0,0]),
                rotation=np.array([0,0,1])),
        ]

        self.chain = ikpy.chain.Chain(links)

    def generate_path_to_point(self,start_config,end_position,step_size=10,final_orientation=np.eye(3)):
        matrix = self.end_affector(start_config)
        positions = kinematics_math.generate_path(
                    matrix[:,3][0:3],
                    end_position,
                    step_size),
        start = Quaternion(matrix=matrix)
        final = Quaternion(matrix=final_orientation)
        orientations = kinematics_math.generate_path_orientations(
                start, final, len(positions))

        path = kinematics_math.make_translation_matrix(positions, orientations)
        ik = start_config
        for wp in path:
            ik = self.chain.inverse_kinematics(wp,np.array(ik))
            yield ik

    def end_affector(self,configuration):
        return self.chain.forward_kinematics(configuration)

    def ee_translation(self,configuration):
        return self.end_affector(configuration)[:,3][0:3]


if __name__ == "__main__":
    PI = math.pi
    lengths = [1,1,1,0]
    thetas = np.array([[PI/2],[-PI/2],[0],[0]])
    adjustment = np.array([[0],[1],[0]])
    goal = kinematics_math.forward_kinematics_4(lengths,thetas)+adjustment

    print(kinematics_math.jacobian_4(lengths,thetas))

    print("goal: {}".format(goal))
    start = time.time()
    for pose in kinematics_math.inverse_kinematics_numerical(lengths,thetas,adjustment,0.0001):
        position = kinematics_math.forward_kinematics_4(lengths,pose)
        print("("),
        for i in range(0,3):
            print("{0:-.2f},".format(position[i][0])),
        print(")")
    total_time = time.time() - start
    print("Finished in {}s".format(total_time))
    print("Final Position: {}\nError: {}".format(position,np.linalg.norm(position-goal)))

    links = [
        ikpy.link.URDFLink(
            name="rotational_base",
            translation_vector=np.array([0,0,0]),
            orientation=np.array([0,0,0]),
            rotation=np.array([0,0,1])),
        ikpy.link.URDFLink(
            name="base_joint",
            translation_vector=np.array([0,0,0]),
            orientation=np.array([0,0,0]),
            rotation=np.array([0,1,0])),
        ikpy.link.URDFLink(
            name="to_elbow",
            translation_vector=np.array([1,0,0]),
            orientation=np.array([0,0,0]),
            rotation=np.array([0,1,0])),
        ikpy.link.URDFLink(
            name="to_wrist",
            translation_vector=np.array([1,0,0]),
            orientation=np.array([0,0,0]),
            rotation=np.array([0,1,0])),
        ikpy.link.URDFLink(
            name="to_manipulator",
            translation_vector=np.array([1,0,0]),
            orientation=np.array([0,0,0]),
            rotation=np.array([0,1,0])),


    ]

    chain = ikpy.chain.Chain(links)

    solver = KinematicSolver()
    ik = [0,-PI/2,PI/2,PI/2,0]
    origin = solver.end_affector(ik)
    origin_v = origin[:,3][0:3]
    ori = Quaternion(matrix=origin)

    goal = np.array([1,1,0])
    g_ori = Quaternion(axis=[1,0,0], radians=math.pi/4)
    print("goal_ori: {}".format(kinematics_math.rotationMatrixToEulerAngles(g_ori.rotation_matrix)))
    print("ori_ori: {}".format(kinematics_math.rotationMatrixToEulerAngles(ori.rotation_matrix)))

    print("goal: {}".format(goal))

    positions = kinematics_math.generate_path(origin_v,goal,10)
    oris = kinematics_math.generate_path_orientations(ori,g_ori,len(positions))
    path = kinematics_math.make_translation_matrix(positions, oris)

    start = time.time()
    for wp in path:
        mat = solver.end_affector(ik)
        print("result: pos = {}, ori = {}".format(mat[:,3][0:3],kinematics_math.rotationMatrixToEulerAngles(mat)))
        ik = chain.inverse_kinematics(wp,np.array(ik))
    total_time = time.time() - start

    print("Computed {} points in {}s".format(len(path),total_time))

    print("solution: {}".format(ik))
    mat = chain.forward_kinematics(ik)
    print("result: pos = {}, ori = {}".format(mat[:,3][0:3],kinematics_math.rotationMatrixToEulerAngles(mat)))

    # lengths = tf.constant([[1,1,1]],dtype=tf.float32)
    # thetas  = tf.constant([[PI/2,-PI/2,0]],dtype=tf.float32)
    # base_length = tf.constant(0,dtype=tf.float32)
    # base_theta  = tf.constant(0,dtype=tf.float32)
    # arm = kinematics_math.arm_3_planar_1_base(0,[1,1,1])
    # fwd_kin = arm.fwd_kinematics_tf(base_length,base_theta,lengths,thetas)
    # jacobian = arm.jacobian_tf(base_length,base_theta,lengths,thetas)

    # opt = kinematics_math.manipulability_optimization()

    # with tf.Session() as sess:
    #     print(sess.run([fwd_kin,jacobian]))
    #     soln = kinematics_math.inverse_kinematics_lagrangian(sess,arm,opt)
