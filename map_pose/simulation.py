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
        if name in ['pelv', 'lhip', 'rhip']:
            correct[name] = np.array(frame[name])
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
    # del corrected['rsho']
    # del corrected['lsho']

    # del corrected['lhipjoint']
    # del corrected['rhipjoint']

    # del corrected['lclavicle']
    # del corrected['rclavicle']

    # del corrected['lowerback']
    # del corrected['upperback']

    # del corrected['thorax']

    return corrected

def del_bones(frame):
    del frame['rsho']
    del frame['lsho']

    del frame['lhipjoint']
    del frame['rhipjoint']

    del frame['root']

    return frame



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


def get_root_pos_rot(pelv, lhip, rhip):
        # calculate the root orientation with respect to the global frame
        v1 = lhip - pelv
        v2 = rhip - pelv
        front = np.cross(v1, v2)
        # Normalize the front vector
        front /= np.linalg.norm(front)
        # calculate the right vector
        right = lhip-rhip
        right /= np.linalg.norm(right)
        #calculate the up vector
        up = np.cross(front, right)
        up /= np.linalg.norm(up)  

        # Create rotation matrix
        quat = np.array([7.07109449e-01, -2.33627864e-10,  1.50568659e-09, 7.07104113e-01])
        rotation_matrix = np.column_stack((right, -up, -front))
        r = Rotation.from_matrix(rotation_matrix)

        return pelv, r.as_quat()


def simulation(frame):

    mujoco.mj_resetData(model, data)
    mujoco.mj_kinematics(model, data)

    new_frame = get_corrected_frame(frame=frame)
    initial_vector = get_initial_vectors()

    _, root_quat = get_root_pos_rot(np.array(new_frame["root"]),np.array(new_frame["lhipjoint"]), np.array(new_frame["rhipjoint"])) # Retruns the quat in x y z w format
    data.qpos[3:7] = [root_quat[3], root_quat[0], root_quat[1], root_quat[2],]
    mujoco.mj_kinematics(model, data)


    new_frame = del_bones(frame=new_frame)


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
                print('error')
            mujoco.mj_kinematics(model, data)
    return model, data


model = mujoco.MjModel.from_xml_path('cmu_humanoid.xml')
data = mujoco.MjData(model)
mujoco.mj_kinematics(model , data)


pose = read_pose('poses22.json')
# pose = pose[0:20]
with mujoco.viewer.launch_passive(model , data) as viewer:
    while viewer.is_running():
        for frame in pose:
            model, data = simulation(frame=frame)
            time.sleep(1/30)
            viewer.sync()