#!/usr/bin/python
# -*- coding: utf-8 -*-

## UR5 Inverse Kinematics

# ***** lib
import numpy as np
from numpy import linalg
from numpy import array as arr
import cmath
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi

#definition of UR5 parameters
d    = [0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0903 + 0.1628]
a    = [0.00000,  -0.42500, -0.39225,  0.00000,  0.00000,  0.0000]
alph = [pi/2,      0,        0,        pi/2,     -pi/2,    0]

# ************************************************** FORWARD KINEMATICS

def AH(n, th):
    T_a = np.identity(4)
    T_a[0, 3] = a[n - 1]

    T_d = np.identity(4)
    T_d[2, 3] = d[n - 1]

    Rzt = arr(
        [[cos(th[n - 1]), -sin(th[n - 1]), 0, 0],
         [sin(th[n - 1]), cos(th[n - 1]),  0, 0],
         [0,                 0,                  1, 0],
         [0,                 0,                  0, 1]])

    Rxa = arr(
        [[1, 0,                0,                 0],
         [0, cos(alph[n - 1]), -sin(alph[n - 1]), 0],
         [0, sin(alph[n - 1]), cos(alph[n - 1]),  0],
         [0, 0,                0,                 1]])

    return T_d @ Rzt @ T_a @ Rxa

"""def AH(a, alpha, d, theta):
    T_a = np.identity(4)
    T_a[0, 3] = a
    T_d = np.identity(4)
    T_d[2, 3] = d
    Rzt = arr(
        [[cos(theta),       -sin(theta),         0, 0],
         [sin(theta),        cos(theta),         0, 0],
         [0,                 0,                  1, 0],
         [0,                 0,                  0, 1]])
    Rxa = arr(
        [[1, 0,                0,                 0],
         [0, cos(alpha),       -sin(alpha),       0],
         [0, sin(alpha),       cos(alpha),        0],
         [0, 0,                0,                 1]])
    return T_d @ Rzt @ T_a @ Rxa"""


def forward(th):
    A_1 = AH(1, th)
    A_2 = AH(2, th)
    A_3 = AH(3, th)
    A_4 = AH(4, th)
    A_5 = AH(5, th)
    A_6 = AH(6, th)

    T_06 = A_1 @ A_2 @ A_3 @ A_4 @ A_5 @ A_6

    return T_06


# ************************************************** INVERSE KINEMATICS

def inverse(desired_pos):  # T60
    th = np.zeros((6, 8), dtype=np.float64)
    P_05 = desired_pos @ [0, 0, -d[5], 1] - [0, 0, 0, 1]

    P_05 = np.asarray(P_05).flatten()

    # **** theta1 ****

    psi = atan2(P_05[2 - 1], P_05[1 - 1])
    phi = acos(d[3] / sqrt(P_05[2 - 1] * P_05[2 - 1] + P_05[1 - 1] * P_05[1 - 1]))

    # The two solutions for theta1 correspond to the shoulder
    # being either left or right

    th[0, 0:4] = pi / 2 + psi + phi
    th[0, 4:8] = pi / 2 + psi - phi
    th = th.real

    # **** theta5 ****

    cl = [0, 4]  # wrist up or down
    for c in cl:
        T_10 = linalg.inv(AH(1, th[:, c]))
        T_16 = T_10 @ desired_pos
        th[4, c:c + 2] = +acos((T_16[2, 3] - d[3]) / d[5])
        th[4, c + 2:c + 4] = -acos((T_16[2, 3] - d[3]) / d[5])

    th = th.real

    # **** theta6 ****
    # theta6 is not well-defined when sin(theta5) = 0 or when T16(1,3), T16(2,3) = 0.

    cl = [0, 2, 4, 6]
    for c in cl:
        T_10 = linalg.inv(AH(1, th[:, c]))
        T_16 = linalg.inv(T_10 @ desired_pos)
        th[5, c:c + 2] = atan2(-T_16[1, 2] / sin(th[4, c]), T_16[0, 2] / sin(th[4, c]))

    th = th.real

    # **** theta3 ****

    cl = [0, 2, 4, 6]
    for c in cl:
        T_10 = linalg.inv(AH(1, th[:, c]))
        T_65 = AH(6, th[:, c])
        T_54 = AH(5, th[:, c])
        T_14 = T_10 @ desired_pos @ linalg.inv(T_54 @ T_65)
        P_13 = T_14 @ [0, -d[3], 0, 1] - [0, 0, 0, 1]
        t3 = cmath.acos((linalg.norm(P_13) ** 2 - a[1] ** 2 - a[2] ** 2) / (2 * a[1] * a[2]))  # norm ?
        th[2, c] = t3.real
        th[2, c + 1] = -t3.real

    # **** theta2 and theta 4 ****

    cl = [0, 1, 2, 3, 4, 5, 6, 7]
    for c in cl:
        T_10 = linalg.inv(AH(1, th[:, c]))
        T_65 = linalg.inv(AH(6, th[:, c]))
        T_54 = linalg.inv(AH(5, th[:, c]))
        T_14 = T_10 @ desired_pos @ T_65 @ T_54
        P_13 = T_14 @ [0, -d[3], 0, 1] - [0, 0, 0, 1]

        # theta 2

        th[1, c] = -atan2(P_13[1], -P_13[0]) + asin(a[2] * sin(th[2, c])
                / linalg.norm(P_13))

        # theta 4

        T_32 = linalg.inv(AH(3, th[:, c]))
        T_21 = linalg.inv(AH(2, th[:, c]))
        T_34 = T_32 @ T_21 @ T_14
        th[3, c] = atan2(T_34[1, 0], T_34[0, 0])
    th = th.real
    return th


def get_joints(x, y, z, rot):
    # base offset
    z -= 0.771347#-0.016300

    # create trasform matrix
    pose = np.zeros((4, 4), dtype=np.float64)
    pose[:, 3] = (x, y, z, 1)
    pose[:3, :3] = rot
    th_res = inverse(pose)

    # normalize
    th_res = (th_res + pi) % (2 * pi) - pi

    # return 5th kinematic solution
    return th_res[:, 5]


def get_pose(joints):
    th = forward(joints)

    x, y, z = th[:3, 3]
    rot = th[:3, :3]

    z += 0.771347
    return x, y, z, rot