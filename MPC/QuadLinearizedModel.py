# import coppelia sim library to interact with the simulator
import sim
import numpy as np
import time


class Model:
    def __init__(self):

        # Physical parameters (to be updated)
        self.d = 0.4 # arm length
        self.m = 3.7 # mass
        self.g = 9.81 # gravity
        
        # Inertia matrix (to be updated)
        self.Ixx = 0.15 
        self.Iyy = 0.18
        self.Izz = 0.15
        self.Izx = 0.01
        self.Ixz = self.Izx


        # Propeller force and torque constants (to retrive from datasheet)
        self.kf = 1.0
        self.km = 0.0245

        # Drag force and torque constants (to be updated)
        self.bf = 0.0001

        # Propeller thrusts
        self.f = np.zeros(4)

    def attitude_control_action(self, u):
        u = np.array(u)

        # Roll control action
        tauX = self.d * (u[2] - u[4])

        # Pitch control action
        tauY = self.d * (u[1] - u[3])

        # Yaw control action
        tauZ = self.km * (-u[1] + u[2] - u[3] + u[4])

        return tauX, tauY, tauZ
    
    def system_matrices(self):
        A = np.zeros((12, 12))
        B = np.zeros((12, 4))
        C = np.zeros((6, 12))
        D = np.zeros((6, 4))

        # A matrix
        A[0, 3] = 1
        A[1, 4] = 1
        A[2, 5] = 1
        A[3, 3] = -self.bf / self.m
        A[3, 7] = -self.g
        A[4, 4] = -self.bf / self.m
        A[4, 6] = self.g
        A[5, 5] = -self.bf / self.m
        A[6, 9] = 1
        A[7, 10] = 1
        A[8, 11] = 1
        A[9, 9] = -self.bf / self.Ixx
        A[10, 10] = -self.bf / self.Iyy
        A[11, 11] = -self.bf / self.Izz

        # B matrix
        B[4, 0] = 1 / self.m
        B[9, 1] = 1 / self.Ixx
        B[10, 2] = 1 / self.Iyy
        B[11, 3] = 1 / self.Izz

        # C matrix
        C[0, 0] = 1
        C[1, 1] = 1
        C[2, 2] = 1
        C[3, 6] = 1
        C[4, 7] = 1
        C[5, 8] = 1

        return A, B, C, D
    



