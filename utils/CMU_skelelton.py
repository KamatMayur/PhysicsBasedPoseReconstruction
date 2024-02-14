import numpy as np
import matplotlib.pyplot as plt
from transforms3d.euler import euler2mat
from mpl_toolkits.mplot3d import Axes3D


class Joint:
    def __init__(self, name, direction, length, axis, dof, limits):
        self.name = name
        self.direction = np.reshape(direction, [3,1])
        self.length = length
        axis = np.deg2rad(axis)
        self.C = euler2mat(*axis)
        self.Cinv = np.linalg.inv(self.C)
        self.limits = np.zeros([3, 2])
        
        for lm, nm in zip(limits, dof):
            if nm == 'rx':
                self.limits[0] = lm
            if nm == 'ry':
                self.limits[1] = lm
            if nm == 'rz':
                self.limits[2] = lm

        self.parent = None
        self.children = []
        self.coordinate = None
        self.matrix = None

    def set_coordinates(self, motion = None):
        if motion is None:
            root_base = (4.69394, 16.0366, -32.9835)
            if self.name == 'root':
                self.coordinate = np.reshape(np.array(root_base), [3, 1])
                rotation = np.deg2rad(np.zeros(3))
                self.matrix = self.C.dot(euler2mat(*rotation)).dot(self.Cinv)

            else:
                idx = 0
                rotation = np.zeros(3)
                rotation = np.deg2rad(rotation)
                self.matrix = self.parent.matrix.dot(self.C).dot(euler2mat(*rotation)).dot(self.Cinv)
                self.coordinate = self.parent.coordinate + self.length * self.matrix.dot(self.direction)


            for child in self.children:
                child.set_coordinates()

        else:
            if self.name == 'root':
                self.coordinate = np.reshape(np.array(motion['root'][:3]), [3, 1])
                self.coordinate = np.reshape(np.array(root_base), [3, 1])
                rotation = np.deg2rad(np.zeros(3))
                self.matrix = self.C.dot(euler2mat(*rotation)).dot(self.Cinv)

            else:
                idx = 0
                rotation = np.zeros(3)
                rotation = np.deg2rad(rotation)
                self.matrix = self.parent.matrix.dot(self.C).dot(euler2mat(*rotation)).dot(self.Cinv)
                self.coordinate = self.parent.coordinate + self.length * self.matrix.dot(self.direction)


            for child in self.children:
                child.set_coordinates()
            






    def draw(self):
        joints = self.to_dict()
        fig = plt.figure()
        ax = Axes3D(fig,auto_add_to_figure=False)
        fig.add_axes(ax)

        ax.set_xlim3d(-50, 10)
        ax.set_ylim3d(-20, 40)
        ax.set_zlim3d(-20, 40)

        xs, ys, zs = [], [], []
        for joint in joints.values():
            xs.append(joint.coordinate[0, 0])
            ys.append(joint.coordinate[1, 0])
            zs.append(joint.coordinate[2, 0])
        plt.plot(zs, xs, ys, 'b.')

        for joint in joints.values():
            child = joint
            if child.parent is not None:
                parent = child.parent
                xs = [child.coordinate[0, 0], parent.coordinate[0, 0]]
                ys = [child.coordinate[1, 0], parent.coordinate[1, 0]]
                zs = [child.coordinate[2, 0], parent.coordinate[2, 0]]
                plt.plot(zs, xs, ys, 'r')
        plt.show()
            # return joints
        
    def to_dict(self):
        ret = {self.name: self}
        for child in self.children:
            ret.update(child.to_dict())
        return ret
    

def read_line(stream, idx):
    if idx >= len(stream):
        return None, idx
    line = stream[idx].strip().split()
    idx += 1
    return line , idx


def parse_asf(file_path):
    with open(file_path) as f:
        content = f.read().splitlines()
    for idx, line in enumerate(content):
        # meta infomation is ignored
        if line == ':bonedata':
            content = content[idx+1:]
            break
    
    joints = {'root': Joint('root', np.zeros(3), 0, np.zeros(3), [], [])}
    idx = 0
    while True:
        line, idx = read_line(content, idx)
        if line[0] == ':hierarchy':
            break
        assert line[0] == 'begin'

        line, idx = read_line(content, idx)
        assert line[0] == 'id'

        line, idx = read_line(content, idx)
        assert line[0] == 'name'
        name = line[1]

        line, idx = read_line(content, idx)
        assert line[0] == 'direction'
        direction = np.array([float(axis) for axis in line[1:]])

        # skip length
        line, idx = read_line(content, idx)
        assert line[0] == 'length'
        length = float(line[1])

        line, idx = read_line(content, idx)
        assert line[0] == 'axis'
        assert line[4] == 'XYZ'

        axis = np.array([float(axis) for axis in line[1:-1]])

        dof = []
        limits = []

        line, idx = read_line(content, idx)
        if line[0] == 'dof':
            dof = line[1:]
            for i in range(len(dof)):
                line, idx = read_line(content, idx)
                if i == 0:
                    assert line[0] == 'limits'
                    line = line[1:]
                assert len(line) == 2
                mini = float(line[0][1:])
                maxi = float(line[1][:-1])
                limits.append((mini, maxi))

            line, idx = read_line(content, idx)

        assert line[0] == 'end'
        joints[name] = Joint(name,direction,length,axis,dof,limits)

    assert line[0] == ':hierarchy'

    line, idx = read_line(content, idx)

    assert line[0] == 'begin'

    while True:
        line, idx = read_line(content, idx)
        if line[0] == 'end':
            break
        assert len(line) >= 2
        for joint_name in line[1:]:
            joints[line[0]].children.append(joints[joint_name])
        for nm in line[1:]:
            joints[nm].parent = joints[line[0]]
    return joints

if __name__ == '__main__':
    asf_path = '18.asx'
    joints = parse_asf(asf_path)
    joints['root'].set_coordinates(motion= None)
    joints['root'].draw()