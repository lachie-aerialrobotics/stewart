#!/usr/bin/env python3

import numpy as np
from scipy.optimize import fsolve
from tf.transformations import euler_from_matrix

class Kinematics:
    def __init__(self, rb, rp, sb, sp, ra, rs):
        self.rb = rb
        self.rp = rp
        self.sb = sb
        self.sp = sp
        self.ra = ra
        self.rs = rs
        self.beta = np.asarray([np.deg2rad(30), np.deg2rad(30), np.deg2rad(150), np.deg2rad(150), np.deg2rad(270), np.deg2rad(270)])
        self.p_p, self.b_w = self._get_joints(self.beta)

    def _get_platform_rotation_matrix(self, Q):
        #calculate platform rotation matrix wRp
        cphi = np.cos(Q[4])
        sphi = np.sin(Q[4])
        cpsi = np.cos(Q[5])
        spsi = np.sin(Q[5])
        ctheta = np.cos(Q[3])
        stheta = np.sin(Q[3])

        Rpsi = np.asarray([[cpsi, -spsi, 0],
                            [spsi, cpsi, 0],
                            [0, 0, 1]])

        Rtheta = np.asarray([[1, 0, 0],
                            [0, ctheta, -stheta],
                            [0, stheta, ctheta]])

        Rphi = np.asarray([[cphi, 0, sphi],
                            [0, 1, 0],
                            [-sphi, 0, cphi]])

        wRp = np.matmul(np.matmul(Rpsi, Rtheta), Rphi)

        return wRp

    def _get_joints(self, beta):
        #calculate coordinates of wrist and shoulder joints in their respective body coordinates
        p_p = np.zeros((6,3))
        b_w = np.zeros((6,3))

        for i in range(3):
            index = 2*i
            angle = beta[index]
            c = np.cos(angle)
            s = np.sin(angle)
            p_p[index,:] = np.asarray([self.rp * c + self.sp * s, self.rp * s - self.sp * c, 0])
            b_w[index,:] = np.asarray([self.rb * c + self.sb * s, self.rb * s - self.sb * c, 0])

            index = 2*i+1
            angle = beta[index]
            c = np.cos(angle)
            s = np.sin(angle)
            p_p[index,:] = np.asarray([self.rp * c - self.sp * s, self.rp * s + self.sp * c, 0])
            b_w[index,:] = np.asarray([self.rb * c - self.sb * s, self.rb * s + self.sb * c, 0])
        return p_p, b_w

    def IPK(self, Q): 
        X = Q[0:3]
        Theta = np.zeros(6)

        wRp = self._get_platform_rotation_matrix(Q)

        for i in range(6):
            #calculate distances from platform and base joints
            p_w = X + np.matmul(wRp, self.p_p[i,:])
            L_w = p_w - self.b_w[i,:]
            rl = np.linalg.norm(L_w)
            L = rl**2 - (self.rs**2 - self.ra**2)

            #convert distances to servo angles
            M = 2 * self.ra * p_w[2]
            N = 2 * self.ra * (np.cos(self.beta[i]) * (p_w[0] - self.b_w[i,0]) + np.sin(self.beta[i]) * (p_w[1] - self.b_w[i,1]))
            disc = L / np.sqrt(M**2 + N**2)

            #check real solution exists -> disc must be in domain of arcsin(), {-1,1}
            if (disc >= 1.0) or (disc <= -1.0):
                Theta[i] = np.nan
            else:
                Theta[i] = np.arcsin(disc) - np.arctan(N / M)
        return Theta
    
    def CheckLims(self, Q, translation_limit, rotation_limit):
        if np.any(np.abs(Q[0:3]) > translation_limit) or np.any(np.abs(Q[3:6]) > np.deg2rad(rotation_limit)):
            is_within_lims = False
        else:
            is_within_lims = True
        return is_within_lims
            
    def IFS(self, Q, Theta, T):
        X = Q[0:3]

        n = np.zeros((6,3))
        p_w_cross_n = np.zeros((6,3))
        f = np.zeros(6)
        wRp = self._get_platform_rotation_matrix(Q)

        for i in range(6):
            p_w = X + np.matmul(wRp, self.p_p[i,:])
            
            l_a = np.asarray([self.ra * np.cos(Theta[i]) * np.cos(self.beta[i]),
                                    self.ra * np.cos(Theta[i]) * np.sin(self.beta[i]),
                                    self.ra * np.sin(Theta[i])])  
                
            l_b = p_w - self.b_w[i,:] - l_a
    
            T_axis = np.asarray([np.sin(self.beta[i]), -np.cos(self.beta[i]), 0])
            
            n[i,:] = l_b / np.linalg.norm(l_b)
            p_w_cross_n[i,:] = np.cross(np.matmul(wRp, self.p_p[i,:]), n[i,:])

            f[i] = T[i] * (1 / np.dot(np.cross(l_a, T_axis), n[i,:]))

        A = np.column_stack((n, p_w_cross_n))
        F = -np.matmul(np.transpose(A), f)

        return F

    def FPK(self, Theta, Q0):       
        Q = fsolve(lambda Qi : self.IPK(Qi) - Theta, Q0, xtol=0.0001)
        return Q
    
    def DefineHomePos(self):
        Qhome = self.FPK(np.zeros(6), np.asarray([0,0,self.rs,0,0,0]))
        return Qhome
    

    def CoMs(self, Q, Theta): 
        X = Q[0:3]
        wRp = self._get_platform_rotation_matrix(Q)

        proximal_CoM_state = np.zeros((6,6))
        distal_CoM_state  = np.zeros((6,6))

        for i in range(6):
            #calculate distances from platform and base joints
            p_w = X + np.matmul(wRp, self.p_p[i,:])
            proximal_link = self.ra * np.asarray([np.cos(Theta[i]) * np.cos(self.beta[i]),  np.cos(Theta[i]) * np.sin(self.beta[i]), np.sin(Theta[i])])
            proximal_CoM = 0.5 *  proximal_link + self.b_w[i]
            elbow_joint = proximal_link + self.b_w[i]
            wrist_joint = p_w
            distal_link = wrist_joint - elbow_joint
            distal_CoM = elbow_joint + 0.5 * distal_link

            proximal_angles = rotation_from_vector(proximal_link)
            distal_angles = rotation_from_vector(distal_link)

            proximal_CoM_state[i,:] = np.hstack((proximal_CoM, proximal_angles))
            distal_CoM_state[i,:] = np.hstack((distal_CoM, distal_angles))

        EE_CoM_state = Q

        return EE_CoM_state, proximal_CoM_state, distal_CoM_state
    
    def inertial(self, mass, I, x_ddot):
        M = np.eye(6)
        M[0,0] = mass
        M[1,1] = mass
        M[2,2] = mass
        M[3:6,3:6] = I
        return np.matmul(M,x_ddot)
    
    def coriolis():
        2 * M * np.cross(omega,v)


    
def rotation_from_vector(vec):
    nx = vec[0]
    ny = vec[1]
    nz = vec[2]
    R = np.asarray([[ny/np.sqrt(nx**2 + ny**2), nx*nz/np.sqrt(nx**2 + ny**2), nx],[-nx/np.sqrt(nx**2+ny**2), ny*nz / np.sqrt(nx**2 + ny**2), ny],[0,-np.sqrt(nx**2 + ny**2), nz]])
    psi, theta, phi = euler_from_matrix(R)

    return np.asarray([psi, theta, phi])