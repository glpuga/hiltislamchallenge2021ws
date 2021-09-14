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
        [rr[0, 0], rr[0, 1], rr[0, 2], tt[0]],
        [rr[1, 0], rr[1, 1], rr[1, 2], tt[1]],
        [rr[2, 0], rr[2, 1], rr[2, 2], tt[2]],
        [0,        0,        0,        1]
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

c0_rot = np.quaternion(-0.5003218001035493, 0.5012125349997221,
                       -0.5001966939080825, 0.49826434600894337)
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

c0_to_c1_transform = np.linalg.inv(c0_ref_to_cam) * c1_ref_to_cam
c0_to_c1_rotation = c0_to_c1_transform[0:3, 0:3]
c0_to_c1_translation = c0_to_c1_transform[0:3, 3]

print()
print(c0_ref_to_cam)
print()
print(c1_ref_to_cam)
print()
print(c0_to_c1_transform)
print()
image_size = (c0_cols, c0_rows)

R1, R2, P1, P2, Q = cv2.stereoRectify(
    cam0_intrinsic_matrix, cam0_distorsion_coeffs,
    cam1_intrinsic_matrix, cam1_distorsion_coeffs,
    imageSize=image_size,
    R=c0_to_c1_rotation,
    T=c0_to_c1_translation
)[0:5]

print('cam0 R: {}'.format(R1))
print('cam0 P: {}'.format(P1))

print('cam1 R: {}'.format(R2))
print('cam1 P: {}'.format(P2))

print('Q: {}'.format(Q))


def to_string(x):
    return "[{}]".format(",".join([str(i) for i in x]))


def to_vector(x):
    return np.asarray(x).reshape(-1)


print('')
print('Camera:')
print('  name: "Hilti Challenge 2021 stereo camera"')
print('  setup: "stereo"')
print('  model: "perspective"')
print('  ')
print('  # new "rectified" matrices is the first three cols of the projection matrix which is calculated with cv::stereoRectify()')
print('  # e.g. fx = P1[0][0] or P2[0][0], cx = P1[0][2] or P2[0][2]')
print('  #      fy = P1[1][1] or P2[1][1], cy = P1[1][2] or P2[1][2]')
print('  ')
print('  fx: {}'.format(P1[0][0]))
print('  fy: {}'.format(P1[1][1]))
print('  cx: {}'.format(P1[0][2]))
print('  cy: {}'.format(P1[1][2]))
print('')
print('  # there is no distortion after stereo rectification')
print('  k1: 0')
print('  k2: 0')
print('  p1: 0')
print('  p2: 0')
print('  k3: 0')
print('')
print('  # Frames per second, as recorded with rostopic hz')
print('  fps: 10.0')
print('  # Image resolution')
print('  cols: {}'.format(c0_cols))
print('  rows: {}'.format(c0_rows))
print('')
print('  # focal_x_baseline is -P2[0][3] which is calculated with cv::stereoRectify()')
print('  focal_x_baseline: {}'.format(np.sum(np.array(c0_to_c1_translation))))
print('')
print('  color_order: "Gray"')
print('')
print('  #======================#')
print('  # Stereo Rectification #')
print('  #======================#')
print('')
print('  # original intrinsic parameters (K, D) and stereo-recitification parameters (R)')
print('  # matrices (K, R) are written in row-major order')
print('')
print('  # The camera setup is such that the left camera is camera 1 and the right one camera 0')
print('StereoRectifier:')
print('  K_right: {}'.format(to_string(to_vector(cam0_intrinsic_matrix))))
print('  D_right: {}'.format(to_string(cam0_distorsion_coeffs)))
print('  R_right: {}'.format(to_string(to_vector(R1))))
print('  K_left:  {}'.format(to_string(to_vector(cam1_intrinsic_matrix))))
print('  D_left:  {}'.format(to_string(cam1_distorsion_coeffs)))
print('  R_left:  {}'.format(to_string(to_vector(R2))))
print('')
