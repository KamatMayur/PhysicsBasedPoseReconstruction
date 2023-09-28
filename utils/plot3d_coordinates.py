import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plot_pose(results):

    x = []
    y= []
    z= []
    for idx , landmark in enumerate(results.pose_landmarks.landmark):
        x.append(landmark.x)
        y.append(landmark.y)
        z.append(landmark.z)


    fig = plt.figure()
    ax = fig.add_subplot(111 , projection='3d')
    ax.scatter(x, y, z, marker='o', color='b')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

