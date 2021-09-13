#!/bin/python3

import cv2
import numpy as np
import quaternion


def intrinsic_matrix(fx, fy, cx, cy):
    return np.matrix(
        [
            [fx, 0,  cx],
            [0,  fy, cy],
            [0,  0,  1]
        ])


def distorsion_coeffiecients(k1=0, k2=0, k3=0, k4=0, p1=0, p2=0, p3=0, p4=0):
    return np.array([k1, k2, p1, p2, k3, k4, 0, 0])


def roto_translation_matrix(q, t):
    rr = quaternion.as_rotation_matrix(q)
    tt = np.array([*t])
    rt_matrix = np.matrix([
        [rr[0, 0], rr[1, 0], rr[2, 0], tt[0]],
        [rr[0, 1], rr[1, 1], rr[2, 1], tt[1]],
        [rr[0, 2], rr[1, 2], rr[2, 2], tt[2]],
        [0,       0,       0,       1]
    ])
    return rt_matrix


#
# camera 0
c0_cx = 703.6097253263
c0_cy = 530.4665279367
c0_fx = 701.6682111281
c0_fy = 701.55526909

c0_k1 = -0.0395909069
c0_k2 = -0.0041727433
c0_k3 = 0.0030288415
c0_k4 = -0.0012784168

c0_cols = 1440
c0_rows = 1080

cam0_intrinsic_matrix = intrinsic_matrix(c0_fx, c0_fy, c0_cx, c0_cy)
cam0_distorsion_coeffs = distorsion_coeffiecients(
    k1=c0_k1,
    k2=c0_k2,
    k3=c0_k3,
    k4=c0_k4
)

c0_rot = np.quaternion(-0.5003218001035493, 0.5012125349997221, -
                       0.5001966939080825, 0.49826434600894337)
c0_tr = np.array(
    [0.05067834857850693, 0.0458784339890185, -0.005943648304780761])

c0_ref_to_cam = roto_translation_matrix(c0_rot, c0_tr)

#
# camera 1
c1_cx = 708.4206218964
c1_cy = 535.6712007522
c1_fx = 696.7174426776
c1_fy = 696.4862496732

c1_k1 = -0.0465180371
c1_k2 = 0.0160363324
c1_k3 = -0.0170339468
c1_k4 = 0.0050095489

c1_cols = c0_cols
c1_rows = c0_rows

cam1_intrinsic_matrix = intrinsic_matrix(c1_fx, c1_fy, c1_cx, c1_cy)
cam1_distorsion_coeffs = distorsion_coeffiecients(
    k1=c1_k1,
    k2=c1_k2,
    k3=c1_k3,
    k4=c1_k4
)

c1_rot = np.quaternion(0.4987817834395415, 0.5018413260691813,
                       0.4995206821044571, 0.49985108193130245)
c1_tr = np.array(
    [0.0507054642910155, -0.060959522169800155, -0.005930631162279414])

c1_ref_to_cam = roto_translation_matrix(c1_rot, c1_tr)

c0_to_c1_transform = np.linalg.inv(c1_ref_to_cam) * c1_ref_to_cam
c0_to_c1_rotation = c0_to_c1_transform[0:3, 0:3]
c0_to_c1_translation = c0_to_c1_transform[0:3, 3]

image_size = (c0_cols, c0_rows)

cam0_rotation, cam1_rotation, cam0_pose, cam1_pose, Q = cv2.stereoRectify(
    cam0_intrinsic_matrix, cam0_distorsion_coeffs,
    cam1_intrinsic_matrix, cam1_distorsion_coeffs,
    imageSize=image_size,
    R=c0_to_c1_rotation,
    T=c0_to_c1_translation
)[0:5]

print("cam0 R: {}".format(cam0_rotation))
print("cam0 P: {}".format(cam0_pose))

print("cam1 R: {}".format(cam1_rotation))
print("cam1 P: {}".format(cam1_pose))

print("Q: {}".format(Q))
