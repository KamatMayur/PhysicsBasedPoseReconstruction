import time
import mujoco
import mujoco.viewer
import collections

# from dm_control.mujoco import math as mjmath
import numpy as np
# from scipy import interpolate
# from dm_control import mujoco

MOCAP_DT = 1.0/120.0
CONVERSION_LENGTH = 0.056444

# physics = mujoco.Physics.from_xml_path('cmu_humanoid.xml')
  

_CMU_MOCAP_JOINT_ORDER = (
    'root0', 'root1', 'root2', 'root3', 'root4', 'root5', 'lowerbackrx',
    'lowerbackry', 'lowerbackrz', 'upperbackrx', 'upperbackry', 'upperbackrz',
    'thoraxrx', 'thoraxry', 'thoraxrz', 'lowerneckrx', 'lowerneckry',
    'lowerneckrz', 'upperneckrx', 'upperneckry', 'upperneckrz', 'headrx',
    'headry', 'headrz', 'rclaviclery', 'rclaviclerz', 'rhumerusrx',
    'rhumerusry', 'rhumerusrz', 'rradiusrx', 'rwristry', 'rhandrx', 'rhandrz',
    'rfingersrx', 'rthumbrx', 'rthumbrz', 'lclaviclery', 'lclaviclerz',
    'lhumerusrx', 'lhumerusry', 'lhumerusrz', 'lradiusrx', 'lwristry',
    'lhandrx', 'lhandrz', 'lfingersrx', 'lthumbrx', 'lthumbrz', 'rfemurrx',
    'rfemurry', 'rfemurrz', 'rtibiarx', 'rfootrx', 'rfootrz', 'rtoesrx',
    'lfemurrx', 'lfemurry', 'lfemurrz', 'ltibiarx', 'lfootrx', 'lfootrz',
    'ltoesrx'
)

Converted = collections.namedtuple('Converted',
                                   ['qpos', 'qvel', 'time'])




def parse(file_name):
  """Parses the amc file format."""
  values = []
  fid = open(file_name, 'r')
  line = fid.readline().strip()
  frame_ind = 1
  first_frame = True
  while True:
    # Parse first frame.
    if first_frame and line[0] == str(frame_ind):
      first_frame = False
      frame_ind += 1
      frame_vals = []
      while True:
        line = fid.readline().strip()
        if not line or line == str(frame_ind):
          values.append(np.array(frame_vals, dtype=float))
          break
        tokens = line.split()
        frame_vals.extend(tokens[1:])
    # Parse other frames.
    elif line == str(frame_ind):
      frame_ind += 1
      frame_vals = []
      while True:
        line = fid.readline().strip()
        if not line or line == str(frame_ind):
          values.append(np.array(frame_vals, dtype=float))
          break
        tokens = line.split()
        frame_vals.extend(tokens[1:])
    else:
      line = fid.readline().strip()
      if not line:
        break
  return values


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

class Amcvals2qpos:
  """Callable that converts .amc values for a frame and to MuJoCo qpos format.
  """

  def __init__(self, index2joint, joint_order):
    """Initializes a new Amcvals2qpos instance.

    Args:
      index2joint: List of joint angles in .amc file.
      joint_order: List of joint names in MuJoco MJCF.
    """
    # Root is x,y,z, then quat.
    # need to get indices of qpos that order for amc default order
    self.qpos_root_xyz_ind = [0, 1, 2]
    self.root_xyz_ransform = np.array(
        [[1, 0, 0], [0, 0, -1], [0, 1, 0]]) * CONVERSION_LENGTH
    self.qpos_root_quat_ind = [3, 4, 5, 6]
    amc2qpos_transform = np.zeros((len(index2joint), len(joint_order)))
    for i in range(len(index2joint)):
      for j in range(len(joint_order)):
        if index2joint[i] == joint_order[j]:
          if 'rx' in index2joint[i]:
            amc2qpos_transform[i][j] = 1
          elif 'ry' in index2joint[i]:
            amc2qpos_transform[i][j] = 1
          elif 'rz' in index2joint[i]:
            amc2qpos_transform[i][j] = 1
    self.amc2qpos_transform = amc2qpos_transform

  def __call__(self, amc_val):
    """Converts a `.amc` frame to MuJoCo qpos format."""
    amc_val_rad = np.deg2rad(amc_val)
    qpos = np.dot(self.amc2qpos_transform, amc_val_rad)

    # Root.
    qpos[:3] = np.dot(self.root_xyz_ransform, amc_val[:3])
    qpos_quat = euler2quat(amc_val[3], amc_val[4], amc_val[5])
    qpos_quat = quatprod(euler2quat(90, 0, 0), qpos_quat)

    for i, ind in enumerate(self.qpos_root_quat_ind):
      qpos[ind] = qpos_quat[i]

    return qpos

def get_qpos(amc_file):
  frame_values = parse(amc_file)

  # joint2index = {}
  # for name in physics.named.data.qpos.axes.row.names:
  #   joint2index[name] = physics.named.data.qpos.axes.row.convert_key_item(name)
 
  joint2index = {'root': slice(0, 7, None), 'lfemurrz': slice(7, 8, None), 'lfemurry': slice(8, 9, None), 'lfemurrx': slice(9, 10, None), 'ltibiarx': slice(10, 11, None), 'lfootrz': slice(11, 12, None), 'lfootrx': slice(12, 13, None), 'ltoesrx': slice(13, 14, None), 'rfemurrz': slice(14, 15, None), 'rfemurry': slice(15, 16, None), 'rfemurrx': slice(16, 17, None), 'rtibiarx': slice(17, 18, None), 'rfootrz': slice(18, 19, None), 'rfootrx': slice(19, 20, None), 'rtoesrx': slice(20, 21, None), 'lowerbackrz': slice(21, 22, None), 'lowerbackry': slice(22, 23, None), 'lowerbackrx': slice(23, 24, None), 'upperbackrz': slice(24, 25, None), 'upperbackry': slice(25, 26, None), 'upperbackrx': slice(26, 27, None), 'thoraxrz': slice(27, 28, None), 'thoraxry': slice(28, 29, None), 'thoraxrx': slice(29, 30, None), 'lowerneckrz': slice(30, 31, None), 'lowerneckry': slice(31, 32, None), 'lowerneckrx': slice(32, 33, None), 'upperneckrz': slice(33, 34, None), 'upperneckry': slice(34, 35, None), 'upperneckrx': slice(35, 36, None), 'headrz': slice(36, 37, None), 'headry': slice(37, 38, None), 'headrx': slice(38, 39, None), 'lclaviclerz': slice(39, 40, None), 'lclaviclery': slice(40, 41, None), 'lhumerusrz': slice(41, 42, None), 'lhumerusry': slice(42, 43, None), 'lhumerusrx': slice(43, 44, None), 'lradiusrx': slice(44, 45, None), 'lwristry': slice(45, 46, None), 'lhandrz': slice(46, 47, None), 'lhandrx': slice(47, 48, None), 'lfingersrx': slice(48, 49, None), 'lthumbrz': slice(49, 50, None), 'lthumbrx': slice(50, 51, None), 'rclaviclerz': slice(51, 52, None), 'rclaviclery': slice(52, 53, None), 'rhumerusrz': slice(53, 54, None), 'rhumerusry': slice(54, 55, None), 'rhumerusrx': slice(55, 56, None), 'rradiusrx': slice(56, 57, None), 'rwristry': slice(57, 58, None), 'rhandrz': slice(58, 59, None), 'rhandrx': slice(59, 60, None), 'rfingersrx': slice(60, 61, None), 'rthumbrz': slice(61, 62, None), 'rthumbrx': slice(62, 63, None)}
  index2joint = {}
  for joint, index in joint2index.items():
      if isinstance(index, slice):
          indices = range(index.start, index.stop)
      else:
          indices = [index]
      for ii in indices:
          index2joint[ii] = joint

  amcvals2qpos_transformer = Amcvals2qpos(index2joint, _CMU_MOCAP_JOINT_ORDER)
  qpos_values = []
  for frame_value in frame_values:
      qpos_values.append(amcvals2qpos_transformer(frame_value))
  qpos_values = np.stack(qpos_values) 
  
  return qpos_values


# qpos_values = get_qpos('new_amc.amc' )

# import mujoco
# import mujoco.viewer


# model = mujoco.MjModel.from_xml_path('cmu_humanoid.xml')
# data = mujoco.MjData(model)

# with mujoco.viewer.launch_passive(model , data) as viewer:
#   while viewer.is_running():
#     for i in range(len(qpos_values)):
#       data.qpos = qpos_values[i]
#       time.sleep(1/120)
#       mujoco.mj_forward(model , data)
#       viewer.sync()
