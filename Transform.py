#!/usr/bin/python3

# Author: LI SHIDI
# Date: July 20, 2018
# This module can calculate the robot pose transformation

import numpy as np
import matplotlib.pyplot as plt

MATRIX = 0
EULER = 1
QUAT = 2

class Transform:

    def __init__(self, translation=np.zeros([3, 1]), rotation=np.eye(3)):

        # check the way of rotation definition
        self.__translation = translation
        self.style = np.array([0, 0, 0])
        if rotation.size == 9:
            self.__matrix3x3 = rotation
            self.style[MATRIX] = 1

        elif rotation.size == 3:
            self.__YPR = rotation
            self.style[EULER] = 1

        elif rotation.size == 4:
            self.__quat = rotation
            self.style[QUAT] = 1

        else:
            print('Transform: Invalid rotation input!')

    def __YPRtoMatrix3x3(self):
        yaw = self.__YPR[0]
        pitch = self.__YPR[1]
        roll = self.__YPR[2]
        cg = np.cos(roll)
        sg = np.sin(roll)
        cb = np.cos(pitch)
        sb = np.sin(pitch)
        ca = np.cos(yaw)
        sa = np.sin(yaw)
        cc = cg * ca
        cs = cg * sa
        sc = sg * ca
        ss = sg * sa

        self.__matrix3x3 = np.matrix([[ca*cb, sb*sc-cs, sb*cc+ss],
                                      [sa*cb, sb*ss+cc, sb*cs-sc],
                                      [-sb,   cb*sg,    cb*cg]])

    def __QuattoMatrix3x3(self):
        x = self.__quat[0]
        y = self.__quat[1]
        z = self.__quat[2]
        w = self.__quat[3]
        d = x*x + y*y + z*z + w*w
        if d == 0:
            print('Transform: Invalid quaternion!')
            return
        s = 2.0 / d
        xs = x * s
        ys = y * s
        zs = z * s
        wx = w * xs
        wy = w * ys
        wz = w * zs
        xx = x * xs
        xy = x * ys
        xz = x * zs
        yy = y * ys
        yz = y * zs
        zz = z * zs

        self.__matrix3x3 = np.matrix([[1 - yy - zz, xy - wz, xz + wy],
                                      [xy + wz, 1 - xx - zz, yz - wx],
                                      [xz - wy, yz + wx, 1 - xx - yy]])

    def __Matrix3x3toYPR(self, solu_style):
        r = self.__matrix3x3
        output1 = np.zeros([1, 3])
        output0 = np.zeros([1, 3])

        # if pitch is not at a singularity
        if np.abs(r[2, 0]) >= 1:
            output1[0, 0] = 0
            output0[0, 0] = 0
            # if gimbal locked down
            if r[2, 0] < 0:
                gamma = np.arctan2(r[0, 1], r[0, 2])
                output1[0, 2] = gamma
                output0[0, 2] = gamma
                output1[0, 1] = np.pi / 2
                output0[0, 1] = np.pi / 2
            # gimbal locked up
            else:
                alpha = np.arctan2(-r[0, 1], -r[0, 2])
                output1[0, 2] = gamma
                output0[0, 2] = gamma
                output1[0, 1] = -np.pi / 2
                output0[0, 1] = -np.pi / 2
        # pitch is not at a singularity
        else:
            # calculate pitch (beta)
            output1[0, 1] = -np.arcsin(r[2, 0])
            output0[0, 1] = np.pi - output1[0, 1]
            # calculate roll (gamma)
            output1[0, 2] = np.arctan2(r[2, 1] / np.cos(output1[0, 1]), r[2, 2] / np.cos(output1[0, 1]))
            output0[0, 2] = np.arctan2(r[2, 1] / np.cos(output0[0, 1]), r[2, 2] / np.cos(output0[0, 1]))
            # calculate yaw (alpha)
            output1[0, 0] = np.arctan2(r[1, 0] / np.cos(output1[0, 1]), r[0, 0] / np.cos(output1[0, 1]))
            output0[0, 0] = np.arctan2(r[1, 0] / np.cos(output0[0, 1]), r[0, 0] / np.cos(output0[0, 1]))

        if solu_style:
            self.__YPR = output1
        else:
            self.__YPR = output0

    def __Matrix3x3toQuat(self):
        r = self.__matrix3x3
        trace = r[0, 0] + r[1, 1] + r[2, 2]
        output = np.zeros([1, 4])
        # if we set exactly 1 + trace > 0 then problems will occur
        if 1 + trace > 1e-8:
            s = np.sqrt(1 + trace)
            output[0, 3] = s * 0.5
            s = 0.5 / s
            output[0, 0] = (r[2, 1] - r[1, 2]) * s
            output[0, 1] = (r[0, 2] - r[2, 0]) * s
            output[0, 2] = (r[1, 0] - r[0, 1]) * s
        # when trace(R) approaches zero
        else:
            i = (2 if r[1, 1] < r[2, 2] else 1) if r[0, 0] < r[1, 1] else (2 if r[0, 0] < r[2, 2] else 0)
            j = (i + 1) % 3
            k = (j + 1) % 3
            s = np.sqrt(r[i, i] - r[j, j] - r[k, k] + 1)
            output[0, i] = s * 0.5
            s = 0.5 / s
            output[0, 3] = (r[k, j] - r[j, k]) * s
            output[0, j] = (r[j, i] + r[i, j]) * s
            output[0, k] = (r[k, i] + r[i, k]) * s

        self.__quat = output

    def setTranslation(self, input):
        if input.size == 3:
            self.__translation = input
        else:
            print('Transform: Invalid input!')

    def setMatrix3x3(self, input):
        if input.size == 9:
            self.__matrix3x3 = input
            self.style = np.array([1, 0, 0])
        else:
            print('Transform: Invalid input!')

    def setYPR(self, input):
        if input.size == 3:
            self.__YPR = input
            self.style = np.array([0, 1, 0])
        else:
            print('Transform: Invalid input!')

    def setQuaternion(self, input):
        if input.size == 4:
            self.__quat = input
            self.style = np.array([0, 0, 1])
        else:
            print('Transform: Invalid input!')

    def getTranslation(self):
        return self.__translation

    def getMatrix3x3(self):
        if self.style[MATRIX]:
            return self.__matrix3x3
        elif self.style[EULER]:
            self.__YPRtoMatrix3x3()
            self.style[MATRIX] = 1
            return self.__matrix3x3
        elif self.style[QUAT]:
            self.__QuattoMatrix3x3()
            self.style[MATRIX] = 1
            return self.__matrix3x3

    def getYPR(self, solu_style):
        if self.style[EULER]:
            return self.__YPR
        elif self.style[MATRIX]:
            self.__Matrix3x3toYPR(solu_style)
            self.style[EULER] = 1
            return self.__YPR
        elif self.style[QUAT]:
            self.__QuattoMatrix3x3()
            self.style[MATRIX] = 1
            self.__Matrix3x3toYPR(solu_style)
            self.style[EULER] = 1
            return self.__YPR

    def getQuaternion(self):
        if self.style[QUAT]:
            return self.__quat
        elif self.style[MATRIX]:
            self.__Matrix3x3toQuat()
            self.style[QUAT] = 1
            return self.__quat
        elif self.style[EULER]:
            self.__YPRtoMatrix3x3()
            self.style[MATRIX] = 1
            self.__Matrix3x3toQuat()
            self.style[QUAT] = 1
            return self.__quat

    def getMatrix4x4(self):
        # create an empty 4x4 matrix
        trans = np.matrix(np.zeros([4, 4]))
        # put in the translation
        trans[0:3, 3] = self.__translation.reshape(3, 1)
        # put in the rotation matrix
        trans[0:3, 0:3] = self.getMatrix3x3()
        # put the constant 1
        trans[3, 3] = 1
        return trans

    def __mul__(self, other):
        # get 4x4 matrix and multiply
        New = self.getMatrix4x4() * other.getMatrix4x4()
        return Transform(New[0:3, 3], New[0:3, 0:3])

    def inverse(self):
        # calculate the transform inverse
        inv = self.getMatrix4x4().I
        return Transform(inv[0:3, 3], inv[0:3, 0:3])
    
