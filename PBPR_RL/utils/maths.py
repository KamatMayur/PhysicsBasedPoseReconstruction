import numpy as np

def quatprod(q1, q2):
    """
    Multiply two quaternions.
    :param q1: Quaternion 1 as a NumPy array [w, x, y, z]
    :param q2: Quaternion 2 as a NumPy array [w, x, y, z]
    :return: Quaternion product as a NumPy array [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z])


def euler2quat(ax, ay, az):
    """Converts euler angles to a quaternion.

    Note: rotation order is zyx

    Args:
        ax: Roll angle (deg)
        ay: Pitch angle (deg).
        az: Yaw angle (deg).

    Returns:
        A numpy array representing the rotation as a quaternion.
    """
    r1 = az
    r2 = ay
    r3 = ax

    c1 = np.cos(np.deg2rad(r1 / 2))
    s1 = np.sin(np.deg2rad(r1 / 2))
    c2 = np.cos(np.deg2rad(r2 / 2))
    s2 = np.sin(np.deg2rad(r2 / 2))
    c3 = np.cos(np.deg2rad(r3 / 2))
    s3 = np.sin(np.deg2rad(r3 / 2))

    q0 = c1 * c2 * c3 + s1 * s2 * s3
    q1 = c1 * c2 * s3 - s1 * s2 * c3
    q2 = c1 * s2 * c3 + s1 * c2 * s3
    q3 = s1 * c2 * c3 - c1 * s2 * s3

    return np.array([q0, q1, q2, q3])

def quaternion_inverse(quaternion):
    """Return inverse of quaternion.

    >>> q0 = random_quaternion()
    >>> q1 = quaternion_inverse(q0)
    >>> np.allclose(quaternion_multiply(q0, q1), [1, 0, 0, 0])
    True

    """
    q = np.array(quaternion, dtype=np.float64, copy=True)
    np.negative(q[1:], q[1:])
    return q / np.dot(q, q)

def multi_quat_diff(q1, q2):
    """
    Calculate the quaternion difference between two arrays of quaternions.

    Parameters:
    q1, q2: numpy arrays of shape (N, 4) containing quaternions.

    Returns:
    diff: numpy array of shape (N, 4) containing the quaternion differences.
    """
    # Ensure inputs are numpy arrays
    q1 = np.array(q1)
    q2 = np.array(q2)
    
    # Normalize quaternions
    q1 /= np.linalg.norm(q1, axis=1, keepdims=True)
    q2 /= np.linalg.norm(q2, axis=1, keepdims=True)
    
    # Compute the quaternion product of q2 conjugated by q1
    q_diff = q1 * q2
    q_diff[:, 1:] *= -1
    q_diff = np.sum(q_diff, axis=1)
    
    return q_diff

def multi_quat_norm(nq):
    """return the scalar rotation of a N joints"""

    nq_norm = np.arccos(np.clip(abs(nq[::4]), -1.0, 1.0))
    return nq_norm

def rotation_from_quaternion(quaternion, separate=False):
    if (1 - abs(quaternion[0]) < 1e-8).all():
        axis = np.array([1.0, 0.0, 0.0])
        angle = 0.0
    else:
        s = np.sqrt(1 - quaternion[0]*quaternion[0])
        axis = quaternion[1:4] / s
        angle = 2 * np.arccos(quaternion[0])
    return (axis, angle) if separate else axis * angle
