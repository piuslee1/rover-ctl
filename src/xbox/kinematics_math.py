#! /usr/bin/env python
import numpy as np
import math
from pyquaternion import Quaternion
# import tensorflow as tf


def forward_kinematics_4(lengths,thetas,joint=None):
    t = thetas
    l = lengths
    joint = len(t)-1 if not joint else joint
    return np.array([
        [math.cos(t[-1])*(sum([l[i]*math.cos(sum(t[0:i+1])) for i in range(0,joint)]))],
        [math.sin(t[-1])*(sum([l[i]*math.cos(sum(t[0:i+1])) for i in range(0,joint)]))],
        [sum([l[i]*math.sin(sum(t[0:i+1])) for i in range(0,len(l))])]
        ])

def jacobian_4(lengths,thetas):
    # rename stuff to make code shorter
    t = thetas
    l = lengths
    sin = math.sin
    cos = math.cos

    js0 = -l[0]*sin(t[0])
    js1 = -l[1]*sin(t[0]+t[1])
    js2 = -l[2]*sin(t[0]+t[1]+t[2])

    jc0 = l[0]*cos(t[0])
    jc1 = l[1]*cos(t[0]+t[1])
    jc2 = l[2]*cos(t[0]+t[1]+t[2])
    xy_comps = [
        js0+js1+js2,
            js1+js2,
                js2,
                  0
    ]
    xy_plane = np.array(xy_comps) + l[-1]
    x = math.cos(t[-1])*xy_plane
    x[-1] = -sin(t[-1])*sum([jc0,jc1,jc2])
    y = math.sin(t[-1])*xy_plane
    y[-1] =  cos(t[-1])*sum([jc0,jc1,jc2])

    z_comps = [
        jc0+jc1+jc2,
            jc1+jc2,
                jc2,
                  0
    ]

    z = np.array(z_comps)
    return np.matrix([x,y,z])

def generate_path(start,end,steps_per_unit):
    start = np.array(start)
    end = np.array(end)
    diff = end - start
    steps = int(np.linalg.norm(diff)*steps_per_unit)+1
    u_diff = diff / float(steps)
    return [start+u_diff*i for i in range(1,steps+1)]  

def rotationMatrixToEulerAngles(R) :
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
     
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def generate_path_orientations(start, end, steps):
    qs = Quaternion.intermediates(start, end, steps)
    ms = []
    for q in qs:
        ms.append(q.rotation_matrix)
    return ms

def make_translation_matrix(t_vectors, orientations):
    assert(len(t_vectors) == len(orientations))
    eye = np.eye(3)
    zeros = np.array([[0,0,0,0]])
    matrices = []
    for i, vector in enumerate(t_vectors):
        top_part = np.concatenate((orientations[i],vector.reshape(3,1)),1)
        mat = np.concatenate((top_part,zeros),0)
        print(rotationMatrixToEulerAngles(mat))
        matrices.append(mat)
    return matrices

# numerical solver for inverse kinematics
# defaults to 4 joint 3-axis manipulator
def inverse_kinematics_numerical(lengths,thetas,adjust,
                                    precision=0.01,
                                    scale=0.05,
                                    steps_per_unit=10,
                                    fwd_kin=forward_kinematics_4,
                                    jacobian=jacobian_4):
    curr_pos =fwd_kin(lengths,thetas) 
    target = curr_pos + adjust
    waypoints = generate_path(curr_pos,target,steps_per_unit)
    for wp in waypoints:
        diff = wp-fwd_kin(lengths,thetas) 
        while np.linalg.norm(diff) >= precision:
            j = jacobian(lengths,thetas)
            j_inv = np.linalg.pinv(j)
            t_adjust = (j_inv * diff)*scale
            thetas = thetas + t_adjust 
            npos = fwd_kin(lengths,thetas)
            diff = wp-npos
        yield thetas

"""
#generates tensorflow dataflows for kinematics calculations
class arm_3_planar_1_base:
    def __init__(self,base_length,lengths):
        self.name = "arm_3_planar_1_base"
        self.lengths = lengths
        self.base_length = base_length
    #assumes rotational base
    def fwd_kinematics_tf(self,base_length,base_angle,lengths,thetas):
        sins = tf.sin(tf.cumsum(thetas,1))
        coss = tf.cos(tf.cumsum(thetas,1))
        xz_comps = tf.multiply(lengths,coss)
        y_comps = tf.multiply(lengths,sins)

        xz_plane = tf.add(tf.reduce_sum(xz_comps,1,keepdims=True),base_length)
        x_final = tf.multiply(xz_plane,tf.cos(base_angle),name="x_final")
        y_final = tf.reduce_sum(y_comps,1,keepdims=True,name="y_final")
        z_final = tf.multiply(xz_plane,tf.sin(base_angle),name="z_final")

        return tf.concat([x_final,y_final,z_final],axis=1,name=self.name+'_forward_kinematics')

    def fwd_kinematics_np(self,base_length,base_angle,lengths,thetas):
        sins = np.sin(np.cumsum(thetas,1))
        coss = np.cos(np.cumsum(thetas,1))
        xz_comps = np.multiply(lengths,coss)
        y_comps = np.multiply(lengths,sins)

        xz_plane = np.add(np.sum(xz_comps,axis=1,keepdims=True),base_length)
        x_final  = np.multiply(xz_plane,np.cos(base_angle))
        y_final  = np.sum(y_comps,axis=1,keepdims=True)
        z_final  = np.multiply(xz_plane,np.sin(base_angle))

        return tf.concat([x_final,y_final,z_final],axis=1,name=self.name+'_forward_kinematics')

    def jacobian_tf(self,base_length,base_angle,lengths,thetas):
        angles_cumulative = tf.cumsum(thetas,1)
        sins = tf.sin(angles_cumulative)
        coss = tf.cos(angles_cumulative)

        xz_comps = tf.cumsum(tf.multiply(lengths,-sins),axis=1,reverse=True)
        y_comps  = tf.cumsum(tf.multiply(lengths, coss),axis=1,reverse=True)

        xz_base_comp = tf.add(
            tf.reduce_sum(tf.multiply(lengths,coss),1,keepdims=True),
            base_length)

        x_final = tf.concat([
            tf.multiply(xz_comps,tf.cos(base_angle)),
            tf.multiply(xz_base_comp,-tf.sin(base_angle))],axis=1)
        y_final = tf.concat([y_comps,tf.zeros([1,1],dtype=tf.float32)],axis=1)
        z_final = tf.concat([
            tf.multiply(xz_comps,tf.sin(base_angle)),
            tf.multiply(xz_base_comp, tf.cos(base_angle))],axis=1)

        return tf.concat([x_final,y_final,z_final],axis=0,name=self.name+'_jacobian')

class manipulability_optimization:
    def value(self,J):
        return tf.matrix_determinant(tf.matmul(J,tf.transpose(J)))
    def partial_derivatives(self,J):
        pass

def inverse_kinematics_lagrangian(solver,tf_arm,tf_criterion):
    # https://dspace.mit.edu/bitstream/handle/1721.1/6425/AIM-854.pdf?sequence=2
    #solving system:
    #   x  = f(theta)
    #   Zh = 0
    # for   h = column of <partial derivatives of criterion>
    #       Z = [J_{n-m}J_m^{-1}:-I_{n-m}]

    base_length = tf.constant(tf_arm.base_length,shape=[1,1],dtype=tf.float32,name="base_length")
    base_theta  = tf.placeholder(shape=[1,1],dtype=tf.float32,name="base_theta")
    lengths = tf.constant(tf_arm.lengths,shape=[1,3],dtype=tf.float32,name="lengths")
    thetas  = tf.placeholder(shape=[1,3],dtype=tf.float32,name="thetas")

    J = tf_arm.jacobian_tf(base_length,base_theta,lengths,thetas)

    n = tf.shape(J)[0]
    m = 3 # workspace DOF: just cartesian coordinates for now

    J_t = tf.transpose(J,name="jacobian_transpose")
    J_m = J_t[0:m,:] # get first m rows of jacobian transpose
    J_n_m = J_t[m:,:] # get the remaining rows of jacobian transpose

    Z = tf.concat([tf.matmul(J_n_m,tf.matrix_inverse(J_m)),tf.eye(n-m)],axis=1) # eqn 6a

    h = tf_criterion.partial_derivatives(J)

    # Zh = tf.matmul(Z,h) # eqn 7

    def fn_to_optimize(args):
        feed_dict = {
            'base_theta:0':args[0],
            'thetas:0':args[1:],
        }
        return solver.run([sys],feed_dict=feed_dict)

    optimize = lambda args: solver.run([],feed_dict={})


    pass
"""
