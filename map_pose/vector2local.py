import numpy as np
from scipy.spatial.transform import Rotation 


# import mujoco

# model = mujoco.MjModel.from_xml_path('cmu_humanoid.xml')
# data = mujoco.MjData(model)
# mujoco.mj_step(model , data)

# print(data.body(1).xquat)

def convert_vector_2_local(vector, quat):
    '''vector: the vector to be converted
        quat: the quaternion that transformed the basis vector'''

    quat = np.concatenate((quat[1:], quat[:1]))
    quat[-1] = -quat[-1]
    # print(quat)
    rotation = Rotation.from_quat(quat)
    # inverse_rotation = rotation.inv()
    # print(inverse_rotation.as_quat())

    rotated_vector = rotation.apply(vector)

    return rotated_vector

# convert_vector_2_local(vector= np.array([1,1,1]), quat = [0.92387953 , 0.38268343,  0.         ,0.])
