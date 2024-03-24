import mujoco
import mujoco.viewer

import json

from scipy.spatial.transform import Rotation
import numpy as np

from get_dof import get_euler
import vector2local
import time

def normalize(vector):
    return vector/np.linalg.norm(vector)

def read_pose(filepath):
    with open(filepath, 'r') as file:
        pose = json.load(file)
    return pose

def get_initial_vectors():
    children_dict = {}
    for i in range(model.nbody):
        parent_id = model.body_parentid[i]  # Get the parent ID of the current body
        if parent_id not in children_dict:  # If parent ID not in dictionary, initialize an empty list
            children_dict[parent_id] = []
        children_dict[parent_id].append(i)

    initial_vector = {}
    for i in range(model.nbody):
        if i in [0,1,6 ,11,17,23,24,30,31 ]:
            pass
        elif i == 14:
            initial_vector[model.body(i).name] = model.body(15).pos
        else:
            initial_vector[model.body(i).name] = model.body(children_dict[i][0]).pos

    return initial_vector


def get_corrected_frame(frame):
    correct = {}

    for name , value in frame.items():
        if name == 'pelv':
            pass
        else:
            # correct[name] =  np.array(frame[name]) - np.array(frame[parent[name]])
            correct[name] = np.array(frame[parent[name]]) - np.array(frame[name])

    key_mapping = {'pelv':'root' , 'lhip':'lhipjoint' , 'rhip':'rhipjoint' , 'spi1':'lowerback', 
                'lkne':'lfemur', 'rkne':'rfemur', 'spi2':'upperback',
                'lank': 'ltibia', 'rank':'rtibia', 'spi3':'thorax', 
                'ltoe':'lfoot' , 'rtoe':'rfoot', 'neck': 'lowerneck', 'head':'upperneck',
                'rcla':'rclavicle' , 'lcla':'lclavicle', 'lelb':'lhumerus' , 'relb':'rhumerus',
                'lwri':'lradius', 'rwri': 'rradius', 'lhan': 'lhand', 'rhan':'rhand'}
                


    corrected = {key_mapping.get(old_key, old_key): value for old_key, value in correct.items()}
    del corrected['rsho']
    del corrected['lsho']

    del corrected['lhipjoint']
    del corrected['rhipjoint']

    # del corrected['lclavicle']
    # del corrected['rclavicle']

    # del corrected['lowerback']
    # del corrected['upperback']

    # del corrected['thorax']

    return corrected

model = mujoco.MjModel.from_xml_path('cmu_humanoid.xml')
data = mujoco.MjData(model)
mujoco.mj_kinematics(model , data)

heirarchy = {
    "pelv":["lhip","rhip","spi1"],
    "lhip":["lkne"],
    "rhip":["rkne"],
    "spi1":["spi2"],
    "lkne":["lank"],
    "rkne":["rank"],
    "spi2":["spi3"],
    "lank":["ltoe"],
    "rank":["rtoe"],
    "spi3":["neck","lcla","rcla"],
    "ltoe":[],
    "rtoe":[],
    "neck":["head"],
    "lcla":["lsho"],
    "rcla":["rsho"],
    "head":[],
    "lsho":["lelb"],
    "rsho":["relb"],
    "lelb":["lwri"],
    "relb":["rwri"],
    "lwri":["lhan"],
    "rwri":["rhan"],
    "lhan":[],
    "rhan":[],
}


parent = {'pelv': None}
for p, children in heirarchy.items():
    for child in children:
        parent[child] = p

# print([parent])
# def get_initial_vectors():
#     children_dict = {}
#     for i in range(model.nbody):
#         parent_id = model.body_parentid[i]  # Get the parent ID of the current body
#         if parent_id not in children_dict:  # If parent ID not in dictionary, initialize an empty list
#             children_dict[parent_id] = []
#         children_dict[parent_id].append(i)

#     initial_vector = {}
#     for i in range(model.nbody):
#         if i in [0,1,6 ,11,17,23,24,30,31 ]:
#             pass
#         elif i == 14:
#             initial_vector[model.body(i).name] = model.body(15).pos
#         else:
#             initial_vector[model.body(i).name] = model.body(children_dict[i][0]).pos

#     return initial_vector

# def get_euler(T_pose, D_pose, axis = 'zxy'):
#     # Normalize vectors
#     T_pose = T_pose / np.linalg.norm(T_pose)
#     D_pose = D_pose / np.linalg.norm(D_pose)

#     # Compute the rotation quaternion
#     rot, _ = Rotation.align_vectors(D_pose, T_pose )
#     euler_angles = rot.as_euler(axis)

#     return euler_angles

# initial_vector_lhumerus = np.array([0, -0.277 ,0])
# initial_vector_lradius = np.array([0, -0.17, 0])

# pose_vector_lhumerus = np.array([25.9475708 , 70.96630859, -258.64605713])
# pose_vector_lradius = np.array([39.28666687,  -43.26953125, -249.75333786])

# quat1 = data.body(19).xquat
# # print('initntial', quat1, '------------', data.body(20).xquat)
# conv_lhumerus = vector2local.convert_vector_2_local(vector=pose_vector_lhumerus, quat=quat1)

# euler_angles1 = get_euler(T_pose= initial_vector_lhumerus, D_pose=pose_vector_lhumerus )
# print(euler_angles1)
# # data.qpos[41] = euler_angles1[0]
# # data.qpos[41] = 1.57079633
# # data.qpos[42] = euler_angles1[2]
# # data.qpos[43] = 1.57079633
# # data.qpos[43] = euler_angles1[1]

# mujoco.mj_kinematics(model , data)
# # print('finala', data.body(19).xquat, '---------------', data.body(20).xquat)
# quat1 = data.body(20).xquat
# conv_radis = vector2local.convert_vector_2_local(vector=pose_vector_lradius, quat=quat1)
# euler_angles1 = get_euler(T_pose=initial_vector_lradius, D_pose=conv_radis, axis='xzy')
# print(euler_angles1)
# data.qpos[44] = euler_angles1[0]
# mujoco.mj_kinematics(model, data)

# initial_vector_lfemur = np.array([0 ,-0.404945 ,0])
# pose_vector_lfemur = np.array([105.39110756,   -36.00244141,  -362.42189026])

# quat1 = data.body(3).xquat

# conv_lfemur = vector2local.convert_vector_2_local(vector=pose_vector_lfemur, quat=quat1)
# euler_angles1 = get_euler(T_pose=initial_vector_lfemur, D_pose=conv_lfemur)


# # euler_angles1 = r.as_euler('zxy')
# # data.qpos[7] = euler_angles1[0]
# # data.qpos[8] = euler_angles1[2]
# # data.qpos[9] = euler_angles1[1]
# # print(euler_angles1)

# with open('poses.json', 'r') as file:
#     poses = json.load(file)

# def get_corrected_frame(frame):
#     correct = {}

#     for name , value in frame.items():
#         if name == 'pelv':
#             pass
#         else:
#             # correct[name] =  np.array(frame[name]) - np.array(frame[parent[name]])
#             correct[name] = np.array(frame[parent[name]]) - np.array(frame[name])

#     key_mapping = {'pelv':'root' , 'lhip':'lhipjoint' , 'rhip':'rhipjoint' , 'spi1':'lowerback', 
#                 'lkne':'lfemur', 'rkne':'rfemur', 'spi2':'upperback',
#                 'lank': 'ltibia', 'rank':'rtibia', 'spi3':'thorax', 
#                 'ltoe':'lfoot' , 'rtoe':'rfoot', 'neck': 'lowerneck', 'head':'upperneck',
#                 'rcla':'rclavicle' , 'lcla':'lclavicle', 'lelb':'lhumerus' , 'relb':'rhumerus',
#                 'lwri':'lradius', 'rwri': 'rradius', 'lhan': 'lhand', 'rhan':'rhand'}
                


#     corrected = {key_mapping.get(old_key, old_key): value for old_key, value in correct.items()}
#     del corrected['rsho']
#     del corrected['lsho']

#     del corrected['lhipjoint']
#     del corrected['rhipjoint']

#     # del corrected['lclavicle']
#     # del corrected['rclavicle']

#     # del corrected['lowerback']
#     # del corrected['upperback']

#     # del corrected['thorax']

#     return corrected


# T_lhumerus = normalize(np.array([0, -0.277 ,0]))
# local_humerus = -normalize(vector2local.convert_vector_2_local(np.array([-221.90647888,   73.67578125,    0.64351273]), quat=data.body(19).xquat))

# euler = get_euler(T_pose=T_lhumerus, D_pose=local_humerus)
# data.qpos[41] = euler[0]
# data.qpos[42] = euler[2]
# data.qpos[43] = euler[1]

# print('lhumerus', euler)
# mujoco.mj_kinematics(model, data)

# T_lradius = normalize(np.array([0 ,-0.17 ,0]))
# local_lrad = -normalize(vector2local.convert_vector_2_local(np.array([155.03848267,   94.33154297, -178.33586121]), quat=data.body(20).xquat))

# euler = get_euler(T_pose=T_lradius, D_pose=local_lrad)
# data.qpos[44] = euler[1]
# print('lradius', euler)
# mujoco.mj_kinematics(model, data)

# T_rhum = normalize(np.array([0 ,-0.277 ,0]))
# T_rrad = normalize(np.array([0, -0.17 ,0]))

# loca_rhum = -normalize(vector2local.convert_vector_2_local(np.array([160.70953369,  19.59619141, 211.26015472]), quat=data.body(26).xquat))
# euler = get_euler(T_pose=T_rhum, D_pose=loca_rhum)
# print('rhume', euler)
# data.qpos[53] = euler[0]
# data.qpos[54] = euler[2]
# data.qpos[55] = euler[1]

# mujoco.mj_kinematics(model, data)

# loca_rrad = -normalize(vector2local.convert_vector_2_local(np.array([-102.5930481 ,  220.47021484,  -77.64592743]), quat=data.body(27).xquat))
# euler = get_euler(T_pose=T_rrad, D_pose=loca_rrad)
# data.qpos[56] = euler[1]


# T_lfemur = normalize(np.array([0 ,-0.404945 ,0]))
# T_ltibia = normalize(np.array([0 ,-0.405693 ,0]))

# T_rfemur = normalize(np.array([0 ,-0.404945 ,0]))
# T_rtibia = normalize(np.array([0 ,-0.405693 ,0]))


# local_lfemur = -normalize(vector2local.convert_vector_2_local(np.array([-78.32650757,  54.5546875 , 344.78094482]), quat=data.body(3).xquat))
# euler = get_euler(T_pose=T_lfemur, D_pose=local_lfemur)

# data.qpos[7] = euler[0]
# data.qpos[8] = euler[2]
# data.qpos[9] = euler[1]
# print('lfem', euler)
# mujoco.mj_kinematics(model, data)



# local_ltibia = -normalize(vector2local.convert_vector_2_local(np.array([-20.17140198, -232.6015625 ,  339.76525879]), quat=data.body(4).xquat))
# euler = get_euler(T_pose=T_lfemur, D_pose=local_ltibia)

# # data.qpos[7] = euler[0]
# # data.qpos[8] = euler[2]
# data.qpos[10] = euler[1]
# print('ltibi', euler)
# mujoco.mj_kinematics(model, data)


# with mujoco.viewer.launch_passive(model , data) as viewer:
#   while viewer.is_running():
#     mujoco.mj_forward(model , data)
#     viewer.sync()

def simulation(frame):

    new_frame = get_corrected_frame(frame=frame)
    initial_vector = get_initial_vectors()

    mujoco.mj_resetData(model, data)
    mujoco.mj_kinematics(model, data)

    for name , position in new_frame.items():
        body_idx = mujoco.mj_name2id(model, 1, name)
        quat = data.body(body_idx).xquat

        T_pos = (initial_vector[name])

        # print('name', name, 'position:',position, 'Tpos', T_pos)
        local_D_pos = -normalize(vector2local.convert_vector_2_local(position, quat))
        euler = get_euler(T_pose=T_pos, D_pose=local_D_pos)
        # print(name, '----------', euler)

        jntadr = model.body(body_idx).jntadr[0]
        jntnum = model.body(body_idx).jntnum[0]

        for j in range(jntadr, jntnum+jntadr):
            jnt_name = model.jnt(j).name
            jnt_range = model.jnt(j).range

            if 'rz' in jnt_name:
                if max(jnt_range) < euler[0]:
                    data.qpos[j+6] = max(jnt_range)
                elif min(jnt_range) > euler[0]:
                    data.qpos[j+6] = min(jnt_range)
                else:
                    data.qpos[j+6] = euler[0]

            elif 'ry' in jnt_name:

                if max(jnt_range) < euler[2]:
                    data.qpos[j+6] = max(jnt_range)
                elif min(jnt_range) > euler[2]:
                    data.qpos[j+6] = min(jnt_range)
                else:
                    data.qpos[j+6] = euler[2]

            elif 'rx' in jnt_name:
                if max(jnt_range) < euler[1]:
                    data.qpos[j+6] = max(jnt_range)
                elif min(jnt_range) > euler[1]:
                    data.qpos[j+6] = min(jnt_range)
                else:
                    data.qpos[j+6] = euler[1]
            else:
                print('isjjios')
            mujoco.mj_kinematics(model, data)
    return model, data




pose = read_pose('poses22.json')

with mujoco.viewer.launch_passive(model , data) as viewer:
    while viewer.is_running():
        for frame in pose:
            model, data = simulation(frame=frame)
            time.sleep(1/30)
            viewer.sync()