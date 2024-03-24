from scipy.spatial.transform import Rotation
import numpy as np

def get_euler(T_pose, D_pose, axis = 'zxy'):
    # Normalize vectors
    T_pose = T_pose / np.linalg.norm(T_pose)
    D_pose = D_pose / np.linalg.norm(D_pose)

    # Compute the rotation quaternion
    rot, _ = Rotation.align_vectors(D_pose, T_pose )
    euler_angles = rot.as_euler(axis)

    return euler_angles